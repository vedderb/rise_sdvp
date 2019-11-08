/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#include <string.h>
#include "comm_can.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "buffer.h"
#include "crc.h"
#include "packet.h"
#include "deca_range.h"
#include "buffer.h"

// Settings
#define CANDx						CAND1
#define RX_FRAMES_SIZE				100

// Threads
static THD_WORKING_AREA(cancom_read_thread_wa, 512);
static THD_WORKING_AREA(cancom_process_thread_wa, 4096);
static THD_FUNCTION(cancom_read_thread, arg);
static THD_FUNCTION(cancom_process_thread, arg);

// Variables
static mutex_t can_mtx;
static CANRxFrame rx_frames[RX_FRAMES_SIZE];
static int rx_frame_read;
static int rx_frame_write;
static thread_t *process_tp;

// Private functions
static void range_func(float dist, uint8_t id);
static void data_func(uint8_t sender, uint8_t *buffer, int len);

/*
 * 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 */
static const CANConfig cancfg = {
		CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
		CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
		CAN_BTR_TS1(8) | CAN_BTR_BRP(6)
};

void comm_can_init(void) {
	rx_frame_read = 0;
	rx_frame_write = 0;

	chMtxObjectInit(&can_mtx);

	palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(GPIO_AF_CAN1));
	palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(GPIO_AF_CAN1));

	canStart(&CANDx, &cancfg);

	IWDG->KR = 0x5555;
	IWDG->PR = 0;
	IWDG->RLR = 140;
	IWDG->KR = 0xAAAA;
	IWDG->KR = 0xCCCC;

	chThdCreateStatic(cancom_read_thread_wa, sizeof(cancom_read_thread_wa), NORMALPRIO + 1,
			cancom_read_thread, NULL);
	chThdCreateStatic(cancom_process_thread_wa, sizeof(cancom_process_thread_wa), NORMALPRIO,
			cancom_process_thread, NULL);
}

static THD_FUNCTION(cancom_read_thread, arg) {
	(void)arg;
	chRegSetThreadName("CAN read");

	event_listener_t el;
	CANRxFrame rxmsg;

	chEvtRegister(&CANDx.rxfull_event, &el, 0);

	while(!chThdShouldTerminateX()) {
		if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(10)) == 0) {
			IWDG->KR = 0xAAAA;
			continue;
		}

		msg_t result = canReceive(&CANDx, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE);

		while (result == MSG_OK) {
			rx_frames[rx_frame_write++] = rxmsg;
			if (rx_frame_write == RX_FRAMES_SIZE) {
				rx_frame_write = 0;
			}

			chEvtSignal(process_tp, (eventmask_t) 1);

			result = canReceive(&CANDx, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE);
		}
	}

	chEvtUnregister(&CANDx.rxfull_event, &el);
}

static THD_FUNCTION(cancom_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("CAN process");
	process_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		while (rx_frame_read != rx_frame_write) {
			CANRxFrame rxmsg = rx_frames[rx_frame_read++];

			if (rxmsg.IDE == CAN_IDE_STD &&
					(rxmsg.SID == (main_id | CAN_MASK_DW) || rxmsg.SID == (CAN_DW_ID_ANY | CAN_MASK_DW))) {

				deca_range_set_range_func(range_func);
				deca_range_set_data_func(data_func);

				switch (rxmsg.data8[0]) {
				case CMD_DW_RANGE: {
					deca_range_measure(rxmsg.data8[1], rxmsg.data8[2]);
				} break;

				case CMD_DW_PING: {
					uint8_t buffer[8];
					int32_t ind = 0;
					buffer[ind++] = CMD_DW_PING;
					comm_can_transmit_sid(main_id | CAN_MASK_DW, buffer, ind);
				} break;

				case CMD_DW_REBOOT: {
					// Wait for watchdog to reboot
					__disable_irq();
					for(;;){};
				} break;

				case CMD_DW_UPTIME: {
					uint8_t buffer[8];
					int32_t ind = 0;
					buffer[ind++] = CMD_DW_UPTIME;
					buffer_append_uint32(buffer, ST2MS(chVTGetSystemTimeX()), &ind);
					comm_can_transmit_sid(main_id | CAN_MASK_DW, buffer, ind);
				} break;

				default:
					break;
				}
			}

			if (rx_frame_read == RX_FRAMES_SIZE) {
				rx_frame_read = 0;
			}
		}
	}
}

void comm_can_transmit_eid(uint32_t id, uint8_t *data, uint8_t len) {
	if (len > 8) {
		return;
	}

	CANTxFrame txmsg;
	txmsg.IDE = CAN_IDE_EXT;
	txmsg.EID = id;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = len;
	memcpy(txmsg.data8, data, len);

	chMtxLock(&can_mtx);
	canTransmit(&CANDx, CAN_ANY_MAILBOX, &txmsg, MS2ST(20));
	chMtxUnlock(&can_mtx);
}

void comm_can_transmit_sid(uint32_t id, uint8_t *data, uint8_t len) {
	if (len > 8) {
		return;
	}

	CANTxFrame txmsg;
	txmsg.IDE = CAN_IDE_STD;
	txmsg.SID = id;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = len;
	memcpy(txmsg.data8, data, len);

	chMtxLock(&can_mtx);
	canTransmit(&CANDx, CAN_ANY_MAILBOX, &txmsg, MS2ST(20));
	chMtxUnlock(&can_mtx);
}

// Private functions

static void range_func(float dist, uint8_t id) {
	uint8_t buffer[8];
	int32_t ind = 0;

	buffer[ind++] = CMD_DW_RANGE;
	buffer[ind++] = id;
	buffer_append_int32(buffer, (int32_t)(dist * 1000.0), &ind);

	comm_can_transmit_sid(main_id | CAN_MASK_DW, buffer, ind);
}

static void data_func(uint8_t sender, uint8_t *buffer, int len) {
	(void)sender;
	(void)buffer;
	(void)len;
	// TODO!
}

