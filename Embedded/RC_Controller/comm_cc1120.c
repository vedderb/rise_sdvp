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

#include "comm_cc1120.h"
#include "ch.h"
#include "hal.h"
#include "commands.h"
#include "cc1120.h"
#include "utils.h"
#include "packet.h"
#include "led.h"
#include "comm_usb.h"

// Settings
#define DEBUG_MODE			0
#define TX_BUFFER_LENGTH	1100
#define TX_BUFFER_SLOTS		5
#define TX_DELAY_US			15000

// Threads
static THD_WORKING_AREA(tx_thread_wa, 1024);
static THD_FUNCTION(tx_thread, arg);

// Private variables
static bool init_done = false;
static uint8_t tx_buffer[TX_BUFFER_SLOTS][TX_BUFFER_LENGTH];
static volatile int tx_slot_len[TX_BUFFER_SLOTS];
static volatile int tx_slot_read;
static volatile int tx_slot_write;
static thread_t *tx_tp;
static virtual_timer_t vt;

// Private functions
static void rx_func(uint8_t *data, int len, int rssi, int lqi, bool crc_ok);
static void wakeup_tx(void *p);

void comm_cc1120_init(void) {
	if (!cc1120_init()) {
		return;
	}

	init_done = true;
	tx_slot_read = 0;
	tx_slot_write = 0;

	cc1120_set_rx_callback(rx_func);

	chVTObjectInit(&vt);

	chThdCreateStatic(tx_thread_wa, sizeof(tx_thread_wa),
			NORMALPRIO, tx_thread, NULL);
}

bool comm_cc1120_init_done(void) {
	return init_done;
}

void comm_cc1120_send_buffer(uint8_t *data, unsigned int len) {
	if (!init_done) {
		return;
	}

	// Wait while fifo is full
	int to = 1000;
	while (((tx_slot_write + 1) % TX_BUFFER_SLOTS) == tx_slot_read && to) {
		chThdSleepMilliseconds(1);
		to--;
	}

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

static void rx_func(uint8_t *data, int len, int rssi, int lqi, bool crc_ok) {
#if DEBUG_MODE
	commands_printf("CC1120 Packet RX");
	commands_printf("Length: %d", len);
	commands_printf("RSSI: %d", rssi);
	commands_printf("LQI: %d", lqi);
	commands_printf("CRC OK: %s", crc_ok ? "Yes" : "No");
	commands_printf("Freqoff est: %.2f Hz", (double)cc1120_get_last_freqoff_est());
	commands_printf("Freqoff: %.2f Hz", (double)cc1120_read_freqoff());

	// Print the bytes
//	for (int i = 0;i < len;i++) {
//		commands_printf("CC1120 RX Byte: %d", data[i]);
//	}

	(void)data;

	commands_printf(" ");
#else
	(void)rssi;
	(void)lqi;

	if (crc_ok) {
		led_toggle(LED_RED);
#if MAIN_MODE_IS_MOTE
		comm_usb_send_packet(data, len);
#else
		commands_process_packet(data, len, comm_cc1120_send_buffer);
#endif
	}
#endif
}

static void wakeup_tx(void *p) {
	(void)p;

	chSysLockFromISR();
	chEvtSignalI(tx_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

static THD_FUNCTION(tx_thread, arg) {
	(void)arg;

	chRegSetThreadName("CC1120 TX");

	tx_tp = chThdGetSelfX();

#if DEBUG_MODE
	for(;;) {
		if (main_id == 2) {
			const int to_tx = 512;
			static uint8_t d[512];
			d[0] = 12;
			for (int i = 1;i < to_tx;i++) {
				d[i] = i;
			}
			commands_printf("Start TX");
			int res = cc1120_transmit(d, to_tx);
			commands_printf("Res: %d", res);
			commands_printf("State After: %s txfifo: %d\n", cc1120_state_name(),
					cc1120_single_read(CC1120_NUM_TXBYTES));
		}

//		if (cc1120_carrier_sense()) {
//			commands_printf("Carrier");
//		}
//		chThdSleepMilliseconds(1);

		chThdSleepMilliseconds(1000);
	}
#else
	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		while(tx_slot_read != tx_slot_write) {
			cc1120_transmit(tx_buffer[tx_slot_read], tx_slot_len[tx_slot_read]);
			led_toggle(LED_GREEN);

			tx_slot_read++;
			if (tx_slot_read >= TX_BUFFER_SLOTS) {
				tx_slot_read = 0;
			}
		}
	}
#endif
}
