/*
	Copyright 2020 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "comm_can.h"
#include "adc_read.h"
#include "buffer.h"
#include "as5047.h"
#include <string.h>
#include <math.h>

// Settings
#define RX_FRAMES_SIZE				100

// Private variables
static mutex_t can_mtx;
static CANRxFrame rx_frames[RX_FRAMES_SIZE];
static int rx_frame_read;
static int rx_frame_write;
static thread_t *process_tp;

// Threads
static THD_WORKING_AREA(cancom_read_thread_wa, 512);
static THD_WORKING_AREA(cancom_process_thread_wa, 2048);
static THD_WORKING_AREA(cancom_send_thread_wa, 2048);
static THD_FUNCTION(cancom_read_thread, arg);
static THD_FUNCTION(cancom_process_thread, arg);
static THD_FUNCTION(cancom_send_thread, arg);

// PWM for steering valve
static PWMConfig pwmcfg = {
		21e6,
		1000,
		NULL,
		{
				{PWM_OUTPUT_ACTIVE_LOW, NULL},
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_DISABLED, NULL}
		},
		0,
		0
};

/*
 * 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 */
static CANConfig cancfg = {
		CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
		CAN_BTR_SJW(3) | CAN_BTR_TS2(2) |
		CAN_BTR_TS1(9) | CAN_BTR_BRP(5)
};

static void set_pwm_out(float val) {
	pwmEnableChannel(&PWMD4, 0, (uint32_t)(powf(val, 0.6) * 1000.0));
}

void comm_can_init(void) {
	chMtxObjectInit(&can_mtx);

	palSetLineMode(LINE_CAN_RX, PAL_MODE_ALTERNATE(HW_CAN_AF));
	palSetLineMode(LINE_CAN_TX, PAL_MODE_ALTERNATE(HW_CAN_AF));

	canStart(&HW_CAN_DEV, &cancfg);

	chThdCreateStatic(cancom_read_thread_wa, sizeof(cancom_read_thread_wa), NORMALPRIO + 1,
			cancom_read_thread, NULL);
	chThdCreateStatic(cancom_process_thread_wa, sizeof(cancom_process_thread_wa), NORMALPRIO,
			cancom_process_thread, NULL);
	chThdCreateStatic(cancom_send_thread_wa, sizeof(cancom_send_thread_wa), NORMALPRIO,
			cancom_send_thread, NULL);

	pwmStart(&PWMD4, &pwmcfg);
	set_pwm_out(0.5);

	palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(2));
}

void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
	if (len > 8) {
		len = 8;
	}

	CANTxFrame txmsg;
	txmsg.IDE = CAN_IDE_EXT;
	txmsg.EID = id;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = len;
	memcpy(txmsg.data8, data, len);

	chMtxLock(&can_mtx);
	canTransmit(&HW_CAN_DEV, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(5));
	chMtxUnlock(&can_mtx);
}

void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len) {
	if (len > 8) {
		len = 8;
	}

	CANTxFrame txmsg;
	txmsg.IDE = CAN_IDE_STD;
	txmsg.SID = id;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = len;
	memcpy(txmsg.data8, data, len);

	chMtxLock(&can_mtx);
	canTransmit(&HW_CAN_DEV, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(5));
	chMtxUnlock(&can_mtx);
}

static THD_FUNCTION(cancom_read_thread, arg) {
	(void)arg;
	chRegSetThreadName("CAN read");

	event_listener_t el;
	CANRxFrame rxmsg;

	chEvtRegister(&HW_CAN_DEV.rxfull_event, &el, 0);

	while(!chThdShouldTerminateX()) {
		if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10)) == 0) {
			continue;
		}

		msg_t result = canReceive(&HW_CAN_DEV, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE);

		while (result == MSG_OK) {
			rx_frames[rx_frame_write++] = rxmsg;
			if (rx_frame_write == RX_FRAMES_SIZE) {
				rx_frame_write = 0;
			}

			chEvtSignal(process_tp, (eventmask_t) 1);

			result = canReceive(&HW_CAN_DEV, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE);
		}
	}

	chEvtUnregister(&HW_CAN_DEV.rxfull_event, &el);
}

static THD_FUNCTION(cancom_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("CAN process");
	process_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		while (rx_frame_read != rx_frame_write) {
			CANRxFrame rxmsg = rx_frames[rx_frame_read++];
			if (rx_frame_read == RX_FRAMES_SIZE) {
				rx_frame_read = 0;
			}

			if (rxmsg.IDE == CAN_IDE_STD) {
				uint16_t id = rxmsg.SID & 0x0F;
				uint16_t msg = (rxmsg.SID >> 4) & 0x0F;
				uint16_t mask = (rxmsg.SID >> 8) & 0x07;

				if (mask == BOARD_MASK && (id == BOARD_ID || id == BOARD_ID_ALL)) {
					switch (msg) {
					case CAN_IO_PACKET_SET_VALVE: {
						switch (rxmsg.data8[0]) {
						case 1: rxmsg.data8[1] ? VALVE_ON(LINE_VALVE_1) : VALVE_OFF(LINE_VALVE_1); break;
						case 2: rxmsg.data8[1] ? VALVE_ON(LINE_VALVE_2) : VALVE_OFF(LINE_VALVE_2); break;
						case 3: rxmsg.data8[1] ? VALVE_ON(LINE_VALVE_3) : VALVE_OFF(LINE_VALVE_3); break;
						case 4: rxmsg.data8[1] ? VALVE_ON(LINE_VALVE_4) : VALVE_OFF(LINE_VALVE_4); break;
						case 5: rxmsg.data8[1] ? VALVE_ON(LINE_VALVE_5) : VALVE_OFF(LINE_VALVE_5); break;
						case 6: rxmsg.data8[1] ? VALVE_ON(LINE_VALVE_6) : VALVE_OFF(LINE_VALVE_6); break;
						default: break;
						}
					} break;

					case CAN_IO_PACKET_SET_VALVES_ALL:
						rxmsg.data8[0] & (1 << 0) ? VALVE_ON(LINE_VALVE_1) : VALVE_OFF(LINE_VALVE_1);
						rxmsg.data8[0] & (1 << 1) ? VALVE_ON(LINE_VALVE_2) : VALVE_OFF(LINE_VALVE_2);
						rxmsg.data8[0] & (1 << 2) ? VALVE_ON(LINE_VALVE_3) : VALVE_OFF(LINE_VALVE_3);
						rxmsg.data8[0] & (1 << 3) ? VALVE_ON(LINE_VALVE_4) : VALVE_OFF(LINE_VALVE_4);
						rxmsg.data8[0] & (1 << 4) ? VALVE_ON(LINE_VALVE_5) : VALVE_OFF(LINE_VALVE_5);
						rxmsg.data8[0] & (1 << 5) ? VALVE_ON(LINE_VALVE_6) : VALVE_OFF(LINE_VALVE_6);
						break;

					case CAN_IO_PACKET_SET_VALVE_PWM_DUTY: {
						int32_t ind = 0;
						set_pwm_out(buffer_get_float16(rxmsg.data8, 1e3, &ind));
					} break;

					default:
						break;
					}
				}
			}
		}
	}
}

static THD_FUNCTION(cancom_send_thread, arg) {
	(void)arg;

	chRegSetThreadName("CAN send");

	for(;;) {
		float adc_div0 = adc_read_get_voltage(0) * ((ADC_DIV_R1 + ADC_DIV_R2) / ADC_DIV_R2);
		float adc_div1 = adc_read_get_voltage(1) * ((ADC_DIV_R1 + ADC_DIV_R2) / ADC_DIV_R2);
		float adc_div2 = adc_read_get_voltage(2) * ((ADC_DIV_R1 + ADC_DIV_R2) / ADC_DIV_R2);
		float adc_div3 = adc_read_get_voltage(3) * ((ADC_DIV_R1 + ADC_DIV_R2) / ADC_DIV_R2);
		float adc_div4 = adc_read_get_voltage(4);
		float adc_div5 = adc_read_get_voltage(5);
		float adc_div6 = adc_read_get_voltage(6) * ((ADC_DIV_R1 + ADC_DIV_R2) / ADC_DIV_R2);

		uint8_t packet[8];
		int32_t ind = 0;
		buffer_append_float16(packet, adc_div0, 0.5e3, &ind);
		buffer_append_float16(packet, adc_div1, 0.5e3, &ind);
		buffer_append_float16(packet, adc_div2, 0.5e3, &ind);
		buffer_append_float16(packet, adc_div3, 0.5e3, &ind);
		comm_can_transmit_sid(BOARD_ID | (BOARD_MASK << 8) | (CAN_IO_PACKET_ADC_VOLTAGES_0_1_2_3 << 4), packet, ind);

		ind = 0;
		buffer_append_float16(packet, adc_div4, 0.5e3, &ind);
		buffer_append_float16(packet, adc_div5, 0.5e3, &ind);
		buffer_append_float16(packet, adc_div6, 0.5e3, &ind);
		buffer_append_float16(packet, 0.0, 0.5e3, &ind);
		comm_can_transmit_sid(BOARD_ID | (BOARD_MASK << 8) | (CAN_IO_PACKET_ADC_VOLTAGES_4_5_6_7 << 4), packet, ind);

		ind = 0;
		buffer_append_float32(packet, as5047_angle(), 1.0e3, &ind);
		comm_can_transmit_sid(BOARD_ID | (BOARD_MASK << 8) | (CAN_IO_PACKET_AS5047_ANGLE << 4), packet, ind);

		ind = 0;
		packet[ind++] = LIM_SW_READ(LINE_LIM_SW_1);
		packet[ind++] = LIM_SW_READ(LINE_LIM_SW_2);
		packet[ind++] = LIM_SW_READ(LINE_LIM_SW_3);
		packet[ind++] = LIM_SW_READ(LINE_LIM_SW_4);
		comm_can_transmit_sid(BOARD_ID | (BOARD_MASK << 8) | (CAN_IO_PACKET_LIM_SW << 4), packet, ind);

		ind = 0;
		buffer_append_float32_auto(packet, adc_read_get_counter(0)->high_time_last, &ind);
		buffer_append_float32_auto(packet, adc_read_get_counter(0)->high_time_current, &ind);
		comm_can_transmit_sid(BOARD_ID | (BOARD_MASK << 8) | (CAN_IO_PACKET_ADC0_HIGH_TIME << 4), packet, ind);

		ind = 0;
		buffer_append_float32_auto(packet, adc_read_get_counter(0)->low_time_last, &ind);
		buffer_append_float32_auto(packet, adc_read_get_counter(0)->low_time_current, &ind);
		comm_can_transmit_sid(BOARD_ID | (BOARD_MASK << 8) | (CAN_IO_PACKET_ADC0_LOW_TIME << 4), packet, ind);

		ind = 0;
		buffer_append_uint32(packet, adc_read_get_counter(0)->toggle_high_cnt, &ind);
		buffer_append_uint32(packet, adc_read_get_counter(0)->toggle_low_cnt, &ind);
		comm_can_transmit_sid(BOARD_ID | (BOARD_MASK << 8) | (CAN_IO_PACKET_ADC0_HIGH_LOW_CNT << 4), packet, ind);

		chThdSleepMilliseconds(1000 / CAN_TX_HZ);
	}
}
