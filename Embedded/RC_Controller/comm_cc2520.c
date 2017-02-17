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

#include "comm_cc2520.h"
#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "led.h"
#include "packet.h"
#include "crc.h"
#include "commands.h"
#include "comm_usb.h"

// CC2520
#include "hal_cc2520.h"
#include "hal_rf.h"
#include "hal_rf_util.h"
#include "basic_rf.h"

#include <string.h>

// Settings
#define TX_BUFFER_LENGTH	110
#define TX_BUFFER_SLOTS		30
#define TX_DELAY_US			5000
#define MAX_PL_LEN			100
#define RX_BUFFER_SIZE		PACKET_MAX_PL_LEN

// Private variables
static basicRfCfg_t basicRfConfig;
static THD_WORKING_AREA(rx_thread_wa, 2048);
static THD_WORKING_AREA(tx_thread_wa, 512);
static uint8_t tx_buffer[TX_BUFFER_SLOTS][TX_BUFFER_LENGTH];
static uint8_t tx_slot_len[TX_BUFFER_SLOTS];
static int tx_slot_read;
static int tx_slot_write;
static thread_t *tx_tp;
static virtual_timer_t vt;
static uint8_t rx_buffer[RX_BUFFER_SIZE];

#ifdef SECURITY_CCM
static uint8_t rf_security_key[] = {
		45, 22, 67, 1,
		56, 2, 32, 233,
		90, 97, 54, 44,
		59, 131, 37, 67
};
#endif

// Private functions
static THD_FUNCTION(rx_thread, arg);
static THD_FUNCTION(tx_thread, arg);
static void wakeup_tx(void *p);

#include "led.h"

void comm_cc2520_init(void) {
	tx_slot_read = 0;
	tx_slot_write = 0;

	// rf
	halAssyInit();
	basicRfConfig.panId = CC2520_PAN_ID;
	basicRfConfig.channel = CC2520_RF_CHANNEL;
	basicRfConfig.ackRequest = FALSE;
	basicRfConfig.myAddr = CC2520_NODE_ADDRESS;
#ifdef SECURITY_CCM
	basicRfConfig.securityKey = rf_security_key;
#endif

	if(basicRfInit(&basicRfConfig) == FAILED) {
		for(;;) {}
	}

	basicRfReceiveOn();

	chVTObjectInit(&vt);

	chThdCreateStatic(rx_thread_wa, sizeof(rx_thread_wa),
			NORMALPRIO, rx_thread, NULL);
	chThdCreateStatic(tx_thread_wa, sizeof(tx_thread_wa),
			NORMALPRIO, tx_thread, NULL);
}

void comm_cc2520_send_buffer(uint8_t *data, unsigned int len) {
	uint8_t send_buffer[MAX_PL_LEN];

	if (len <= (MAX_PL_LEN - 1)) {
		uint32_t ind = 0;
		send_buffer[ind++] = MOTE_PACKET_PROCESS_SHORT_BUFFER;
		memcpy(send_buffer + ind, data, len);
		ind += len;
		comm_cc2520_send_packet(send_buffer, ind);
	} else {
		unsigned int end_a = 0;
		unsigned int len2 = len - (MAX_PL_LEN - 5);

		for (unsigned int i = 0;i < len2;i += (MAX_PL_LEN - 2)) {
			if (i > 255) {
				break;
			}

			end_a = i + (MAX_PL_LEN - 2);

			uint8_t send_len = (MAX_PL_LEN - 2);
			send_buffer[0] = MOTE_PACKET_FILL_RX_BUFFER;
			send_buffer[1] = i;

			if ((i + (MAX_PL_LEN - 2)) <= len2) {
				memcpy(send_buffer + 2, data + i, send_len);
			} else {
				send_len = len2 - i;
				memcpy(send_buffer + 2, data + i, send_len);
			}

			comm_cc2520_send_packet(send_buffer, send_len + 2);
		}

		for (unsigned int i = end_a;i < len2;i += (MAX_PL_LEN - 3)) {
			uint8_t send_len = (MAX_PL_LEN - 3);
			send_buffer[0] = MOTE_PACKET_FILL_RX_BUFFER_LONG;
			send_buffer[1] = i >> 8;
			send_buffer[2] = i & 0xFF;

			if ((i + (MAX_PL_LEN - 3)) <= len2) {
				memcpy(send_buffer + 3, data + i, send_len);
			} else {
				send_len = len2 - i;
				memcpy(send_buffer + 3, data + i, send_len);
			}

			comm_cc2520_send_packet(send_buffer, send_len + 3);
		}

		uint32_t ind = 0;
		send_buffer[ind++] = MOTE_PACKET_PROCESS_RX_BUFFER;
		send_buffer[ind++] = len >> 8;
		send_buffer[ind++] = len & 0xFF;
		unsigned short crc = crc16(data, len);
		send_buffer[ind++] = (uint8_t)(crc >> 8);
		send_buffer[ind++] = (uint8_t)(crc & 0xFF);
		memcpy(send_buffer + 5, data + len2, len - len2);
		ind += len - len2;

		comm_cc2520_send_packet(send_buffer, ind);
	}
}

/*
 * Note that this packet is stored and sent in another thread after a delay. The delay is done
 * in case that this packet is a request from the (Qt) client, which means that more requests
 * can come within a short time. We want to give the additional requests a chance to arrive
 * before blocking the RF channel by sending the response.
 */
void comm_cc2520_send_packet(uint8_t *data, uint8_t len) {
	memcpy(tx_buffer[tx_slot_write], data, len);
	tx_slot_len[tx_slot_write] = len;

	tx_slot_write++;
	if (tx_slot_write >= TX_BUFFER_SLOTS) {
		tx_slot_write = 0;
	}

	chSysLock();
	if (!chVTIsArmedI(&vt)) {
		chVTSetI(&vt, US2ST(TX_DELAY_US), wakeup_tx, NULL);
	}
	chSysUnlock();
}

static THD_FUNCTION(rx_thread, arg) {
	(void)arg;

	chRegSetThreadName("CC2520 RX");

	for(;;) {
		if (basicRfPacketIsReady()) {
			static uint8_t buf[130];
			unsigned int len = 0;
			unsigned int ind = 0;

			len = basicRfReceive(buf, 130, NULL);
			MOTE_PACKET packet = buf[0];

			led_toggle(LED_RED);

			switch (packet) {
			case MOTE_PACKET_FILL_RX_BUFFER:
				memcpy(rx_buffer + buf[1], buf + 2, len - 2);
				break;

			case MOTE_PACKET_FILL_RX_BUFFER_LONG: {
				int rxbuf_ind = (unsigned int)buf[1] << 8;
				rxbuf_ind |= buf[2];
				if (rxbuf_ind < RX_BUFFER_SIZE) {
					memcpy(rx_buffer + rxbuf_ind, buf + 3, len - 3);
				}
			}
			break;

			case MOTE_PACKET_PROCESS_RX_BUFFER: {
				ind = 1;
				int rxbuf_len = (unsigned int)buf[ind++] << 8;
				rxbuf_len |= (unsigned int)buf[ind++];

				if (rxbuf_len > RX_BUFFER_SIZE) {
					break;
				}

				uint8_t crc_high = buf[ind++];
				uint8_t crc_low = buf[ind++];

				memcpy(rx_buffer + rxbuf_len - (len - ind), buf + ind, len - ind);

				if (crc16(rx_buffer, rxbuf_len)
						== ((unsigned short) crc_high << 8
								| (unsigned short) crc_low)) {
#if MAIN_MODE == MAIN_MODE_MOTE_2400
					comm_usb_send_packet(rx_buffer, rxbuf_len);
#else
					commands_process_packet(rx_buffer, rxbuf_len, comm_cc2520_send_buffer);
#endif
				}
			}
			break;

			case MOTE_PACKET_PROCESS_SHORT_BUFFER:
#if MAIN_MODE == MAIN_MODE_MOTE_2400
				comm_usb_send_packet(buf + 1, len - 1);
#else
				commands_process_packet(buf + 1, len - 1, comm_cc2520_send_buffer);
#endif
				break;

			default:
				break;
			}
		}

		chThdSleepMicroseconds(100);
	}
}

static THD_FUNCTION(tx_thread, arg) {
	(void)arg;

	chRegSetThreadName("CC2520 TX");

	tx_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		while(tx_slot_read != tx_slot_write) {
			basicRfSendPacket(CC2520_DEST_ADDRESS, tx_buffer[tx_slot_read], tx_slot_len[tx_slot_read]);
			led_toggle(LED_GREEN);

			tx_slot_read++;
			if (tx_slot_read >= TX_BUFFER_SLOTS) {
				tx_slot_read = 0;
			}
		}
	}
}

static void wakeup_tx(void *p) {
	(void)p;

	chSysLockFromISR();
	chEvtSignalI(tx_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}
