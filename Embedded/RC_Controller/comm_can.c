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
#include "datatypes.h"
#include "buffer.h"
#include "crc.h"
#include "packet.h"
#include "bldc_interface.h"
#include "commands.h"
#include "motor_sim.h"

// Settings
#define CANDx						CAND1
#define RX_FRAMES_SIZE				100
#define RX_BUFFER_SIZE				PACKET_MAX_PL_LEN
#define CAN_STATUS_MSGS_TO_STORE	10

// Threads
static THD_WORKING_AREA(cancom_read_thread_wa, 512);
static THD_WORKING_AREA(cancom_process_thread_wa, 4096);
static THD_FUNCTION(cancom_read_thread, arg);
static THD_FUNCTION(cancom_process_thread, arg);

// Variables
static can_status_msg stat_msgs[CAN_STATUS_MSGS_TO_STORE];
static mutex_t can_mtx;
static mutex_t vesc_mtx_ext;
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static unsigned int rx_buffer_last_id;
static CANRxFrame rx_frames[RX_FRAMES_SIZE];
static int rx_frame_read;
static int rx_frame_write;
static thread_t *process_tp;
static int vesc_id = VESC_ID;

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

// Private functions
static void send_packet_wrapper(unsigned char *data, unsigned int len);
static void printf_wrapper(char *str);

// Function pointers
static void(*m_range_func)(uint8_t id, uint8_t dest, float range) = 0;

void comm_can_init(void) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		stat_msgs[i].id = -1;
	}

	rx_frame_read = 0;
	rx_frame_write = 0;
	vesc_id = VESC_ID;

	chMtxObjectInit(&can_mtx);
	chMtxObjectInit(&vesc_mtx_ext);

	palSetPadMode(GPIOD, 0, PAL_MODE_ALTERNATE(GPIO_AF_CAN1));
	palSetPadMode(GPIOD, 1, PAL_MODE_ALTERNATE(GPIO_AF_CAN1));

	canStart(&CANDx, &cancfg);

	bldc_interface_init(send_packet_wrapper);
	bldc_interface_set_rx_printf_func(printf_wrapper);

	chThdCreateStatic(cancom_read_thread_wa, sizeof(cancom_read_thread_wa), NORMALPRIO + 1,
			cancom_read_thread, NULL);
	chThdCreateStatic(cancom_process_thread_wa, sizeof(cancom_process_thread_wa), NORMALPRIO,
			cancom_process_thread, NULL);
}

void comm_can_set_vesc_id(int id) {
	vesc_id = id;

	if (vesc_id == DIFF_STEERING_VESC_LEFT) {
		motor_sim_set_motor(0);
	} else if (vesc_id == DIFF_STEERING_VESC_RIGHT) {
		motor_sim_set_motor(1);
	}
}

void comm_can_lock_vesc(void) {
	chMtxLock(&vesc_mtx_ext);
}

void comm_can_unlock_vesc(void) {
	chMtxUnlock(&vesc_mtx_ext);
}

static THD_FUNCTION(cancom_read_thread, arg) {
	(void)arg;
	chRegSetThreadName("CAN read");

	event_listener_t el;
	CANRxFrame rxmsg;

	chEvtRegister(&CANDx.rxfull_event, &el, 0);

	while(!chThdShouldTerminateX()) {
		if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(10)) == 0) {
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

	int32_t ind = 0;
	unsigned int rxbuf_len;
	unsigned int rxbuf_ind;
	uint8_t crc_low;
	uint8_t crc_high;
	bool commands_send;

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		while (rx_frame_read != rx_frame_write) {
			CANRxFrame rxmsg = rx_frames[rx_frame_read++];

			if (rxmsg.IDE == CAN_IDE_EXT) {
				// Process extended IDs (VESC Communication)

				uint8_t id = rxmsg.EID & 0xFF;
				CAN_PACKET_ID cmd = rxmsg.EID >> 8;
				can_status_msg *stat_tmp;

				switch (cmd) {
				case CAN_PACKET_FILL_RX_BUFFER:
					memcpy(rx_buffer + rxmsg.data8[0], rxmsg.data8 + 1, rxmsg.DLC - 1);
					break;

				case CAN_PACKET_FILL_RX_BUFFER_LONG:
					rxbuf_ind = (unsigned int)rxmsg.data8[0] << 8;
					rxbuf_ind |= rxmsg.data8[1];
					if (rxbuf_ind < RX_BUFFER_SIZE) {
						memcpy(rx_buffer + rxbuf_ind, rxmsg.data8 + 2, rxmsg.DLC - 2);
					}
					break;

				case CAN_PACKET_PROCESS_RX_BUFFER:
					ind = 0;
					rx_buffer_last_id = rxmsg.data8[ind++];
					commands_send = rxmsg.data8[ind++];
					rxbuf_len = (unsigned int)rxmsg.data8[ind++] << 8;
					rxbuf_len |= (unsigned int)rxmsg.data8[ind++];

					if (rxbuf_len > RX_BUFFER_SIZE) {
						break;
					}

					crc_high = rxmsg.data8[ind++];
					crc_low = rxmsg.data8[ind++];

					if (crc16(rx_buffer, rxbuf_len)
							== ((unsigned short) crc_high << 8
									| (unsigned short) crc_low)) {

						(void)commands_send;
						bldc_interface_process_packet(rx_buffer, rxbuf_len);
					}
					break;

				case CAN_PACKET_PROCESS_SHORT_BUFFER:
					ind = 0;
					rx_buffer_last_id = rxmsg.data8[ind++];
					commands_send = rxmsg.data8[ind++];
					(void)commands_send;
					bldc_interface_process_packet(rxmsg.data8 + ind, rxmsg.DLC - ind);
					break;

				case CAN_PACKET_STATUS:
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						stat_tmp = &stat_msgs[i];
						if (stat_tmp->id == id || stat_tmp->id == -1) {
							ind = 0;
							stat_tmp->id = id;
							stat_tmp->rx_time = chVTGetSystemTime();
							stat_tmp->rpm = (float)buffer_get_int32(rxmsg.data8, &ind);
							stat_tmp->current = (float)buffer_get_int16(rxmsg.data8, &ind) / 10.0;
							stat_tmp->duty = (float)buffer_get_int16(rxmsg.data8, &ind) / 1000.0;
							break;
						}
					}
					break;

				default:
					break;
				}
			} else if (rxmsg.IDE == CAN_IDE_STD) {
				// Process standard IDs
#if CAN_EN_DW
				if ((rxmsg.SID & 0x700) == CAN_MASK_DW) {
					switch (rxmsg.data8[0]) {
					case CMD_DW_RANGE: {
						int32_t ind = 1;
						uint8_t id = rxmsg.SID & 0xFF;
						uint8_t dest = rxmsg.data8[ind++];
						float range = (float)buffer_get_int32(rxmsg.data8, &ind) / 1000.0;

						if (m_range_func) {
							m_range_func(id, dest, range);
						}
					} break;
					default:
						break;
					}
				}
#endif
			}

			if (rx_frame_read == RX_FRAMES_SIZE) {
				rx_frame_read = 0;
			}
		}
	}
}

void comm_can_transmit_eid(uint32_t id, uint8_t *data, uint8_t len) {
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

/**
 * Send a buffer up to RX_BUFFER_SIZE bytes as fragments. If the buffer is 6 bytes or less
 * it will be sent in a single CAN frame, otherwise it will be split into
 * several frames.
 *
 * @param controller_id
 * The controller id to send to.
 *
 * @param data
 * The payload.
 *
 * @param len
 * The payload length.
 *
 * @param send
 * If true, this packet will be passed to the send function of commands.
 * Otherwise, it will be passed to the process function (DON'T CARE HERE, only for VESC).
 */
void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, bool send) {
	uint8_t send_buffer[8];

	if (len <= 6) {
		uint32_t ind = 0;
		send_buffer[ind++] = main_id + 128;
		send_buffer[ind++] = send;
		memcpy(send_buffer + ind, data, len);
		ind += len;
		comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8), send_buffer, ind);
	} else {
		unsigned int end_a = 0;
		for (unsigned int i = 0;i < len;i += 7) {
			if (i > 255) {
				break;
			}

			end_a = i + 7;

			uint8_t send_len = 7;
			send_buffer[0] = i;

			if ((i + 7) <= len) {
				memcpy(send_buffer + 1, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 1, data + i, send_len);
			}

			comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8), send_buffer, send_len + 1);
		}

		for (unsigned int i = end_a;i < len;i += 6) {
			uint8_t send_len = 6;
			send_buffer[0] = i >> 8;
			send_buffer[1] = i & 0xFF;

			if ((i + 6) <= len) {
				memcpy(send_buffer + 2, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 2, data + i, send_len);
			}

			comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8), send_buffer, send_len + 2);
		}

		uint32_t ind = 0;
		send_buffer[ind++] = main_id + 128;
		send_buffer[ind++] = send;
		send_buffer[ind++] = len >> 8;
		send_buffer[ind++] = len & 0xFF;
		unsigned short crc = crc16(data, len);
		send_buffer[ind++] = (uint8_t)(crc >> 8);
		send_buffer[ind++] = (uint8_t)(crc & 0xFF);

		comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8), send_buffer, ind++);
	}
}

/**
 * Start ranging with a DW node on the CAN bus.
 *
 * @param id
 * The ID of the DW node.
 *
 * @param dest
 * The ID of the node to range to.
 *
 * @param samples
 * How many samples to average over.
 */
void comm_can_dw_range(uint8_t id, uint8_t dest, int samples) {
	uint8_t buffer[8];
	int32_t ind = 0;

	buffer[ind++] = CMD_DW_RANGE;
	buffer[ind++] = dest;
	buffer[ind++] = samples;

	comm_can_transmit_sid(((uint32_t)id | CAN_MASK_DW), buffer, ind);
}

/**
 * Set the function to be called when ranging is done.
 *
 * @param func
 * A pointer to the function.
 */
void comm_can_set_range_func(void(*func)(uint8_t id, uint8_t dest, float range)) {
	m_range_func = func;
}

/**
 * Get status message by index.
 *
 * @param index
 * Index in the array
 *
 * @return
 * The message or 0 for an invalid index.
 */
can_status_msg *comm_can_get_status_msg_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs[index];
	} else {
		return 0;
	}
}

/**
 * Get status message by id.
 *
 * @param id
 * Id of the controller that sent the status message.
 *
 * @return
 * The message or 0 for an invalid id.
 */
can_status_msg *comm_can_get_status_msg_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs[i].id == id) {
			return &stat_msgs[i];
		}
	}

	return 0;
}

static void send_packet_wrapper(unsigned char *data, unsigned int len) {
	comm_can_send_buffer(vesc_id, data, len, false);
}

static void printf_wrapper(char *str) {
	commands_printf(str);
}
