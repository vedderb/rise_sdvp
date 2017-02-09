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
#include "conf_general.h"
#include "commands.h"
#include "cc1120.h"
#include "utils.h"
#include "packet.h"

// Threads
static THD_WORKING_AREA(tx_thread_wa, 512);
static THD_FUNCTION(tx_thread, arg);

// Private functions
static void rx_func(uint8_t *data, int len, int rssi, int lqi, bool crc_ok);

void comm_cc1120_init(void) {
	cc1120_init();
	cc1120_set_rx_callback(rx_func);

	chThdCreateStatic(tx_thread_wa, sizeof(tx_thread_wa),
			NORMALPRIO, tx_thread, NULL);
}

static void rx_func(uint8_t *data, int len, int rssi, int lqi, bool crc_ok) {
	commands_printf("CC1120 Packet RX");
	commands_printf("Length: %d", len);
	commands_printf("RSSI: %d", rssi);
	commands_printf("LQI: %d", lqi);
	commands_printf("CRC OK: %s", crc_ok ? "Yes" : "No");

	// Print the bytes for now
//	for (int i = 0;i < len;i++) {
//		commands_printf("CC1120 RX Byte: %d", data[i]);
//	}

	(void)data;

	commands_printf(" ");
}

static THD_FUNCTION(tx_thread, arg) {
	(void)arg;

	chRegSetThreadName("CC1120 TX");

	for(;;) {
//		const int to_tx = 1024;
//		uint8_t d[to_tx];
//		d[0] = 12;
//		for (int i = 1;i < to_tx;i++) {
//			d[i] = i;
//		}
//		commands_printf("Start TX");
//		int res = cc1120_transmit(d, to_tx);
//		commands_printf("Res: %d", res);
//		commands_printf("State After: %s txfifo: %d\n", cc1120_state_name(),
//				cc1120_single_read(CC1120_NUM_TXBYTES));
//
//		if (cc1120_carrier_sense()) {
//			commands_printf("Carrier");
//		}
//		chThdSleepMilliseconds(1);

		chThdSleepMilliseconds(1000);
	}
}
