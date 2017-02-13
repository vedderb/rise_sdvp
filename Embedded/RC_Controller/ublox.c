/*
	Copyright 2017 Benjamin Vedder	benjamin@vedder.se

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

#include "ublox.h"
#include "commands.h"
#include "utils.h"
#include "pos.h"
#include "rtcm3_simple.h"
#include <string.h>

// Settings
#define HW_UART_DEV					UARTD6
#define HW_UART_TX_PORT				GPIOC
#define HW_UART_TX_PIN				6
#define HW_UART_RX_PORT				GPIOC
#define HW_UART_RX_PIN				7
#define BAUDRATE					115200
#define SERIAL_RX_BUFFER_SIZE		1024
#define LINE_BUFFER_SIZE			256
#define UBX_BUFFER_SIZE				2048
#define PRINT_UBX_MSGS				1

// Threads
static THD_FUNCTION(process_thread, arg);
static THD_WORKING_AREA(process_thread_wa, 4096);
static thread_t *process_tp = 0;

// Variables
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;
static rtcm3_state rtcm_state;

// Private functions
static void ubx_decode(uint8_t class, uint8_t id, uint8_t *msg, int len);
static void ubx_decode_relposned(uint8_t *msg, int len);
static void ubx_decode_rawx(uint8_t *msg, int len);

static uint8_t ubx_get_U1(uint8_t *msg, int *ind);
static int8_t ubx_get_I1(uint8_t *msg, int *ind);
static uint8_t ubx_get_X1(uint8_t *msg, int *ind);
static uint16_t ubx_get_U2(uint8_t *msg, int *ind);
static int16_t ubx_get_I2(uint8_t *msg, int *ind);
static uint16_t ubx_get_X2(uint8_t *msg, int *ind);
static uint32_t ubx_get_U4(uint8_t *msg, int *ind);
static int32_t ubx_get_I4(uint8_t *msg, int *ind);
static uint32_t ubx_get_X4(uint8_t *msg, int *ind);
static float ubx_get_R4(uint8_t *msg, int *ind);
static double ubx_get_R8(uint8_t *msg, int *ind);

// Callbacks
static void(*rx_relposned)(ubx_nav_relposned *pos) = 0;
static void(*rx_rawx)(ubx_rxm_rawx *pos) = 0;

/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {
	(void)uartp;
	(void)e;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
	(void)uartp;
	serial_rx_buffer[serial_rx_write_pos++] = c;

	if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
		serial_rx_write_pos = 0;
	}

	chSysLockFromISR();
	chEvtSignalI(process_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
		txend1,
		txend2,
		rxend,
		rxchar,
		rxerr,
		BAUDRATE,
		0,
		USART_CR2_LINEN,
		0
};

void ublox_init(void) {
	uartStart(&HW_UART_DEV, &uart_cfg);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(8));
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(8));

	rtcm3_init_state(&rtcm_state);

	chThdCreateStatic(process_thread_wa, sizeof(process_thread_wa), NORMALPRIO, process_thread, NULL);

	// Prevent unused warnings
	(void)ubx_get_U1;
	(void)ubx_get_I1;
	(void)ubx_get_X1;
	(void)ubx_get_U2;
	(void)ubx_get_I2;
	(void)ubx_get_X2;
	(void)ubx_get_U4;
	(void)ubx_get_I4;
	(void)ubx_get_X4;
	(void)ubx_get_R4;
	(void)ubx_get_R8;
}

void ublox_send(unsigned char *data, unsigned int len) {
	if (!process_tp) {
		return;
	}

	// Wait for the previous transmission to finish.
	while (HW_UART_DEV.txstate == UART_TX_ACTIVE) {
		chThdSleep(1);
	}

	// Copy this data to a new buffer in case the provided one is re-used
	// after this function returns.
	static uint8_t buffer[1024];
	memcpy(buffer, data, len);

	uartStartSend(&HW_UART_DEV, len, buffer);
}

void ublox_set_rx_callback_relposned(void(*func)(ubx_nav_relposned *pos)) {
	rx_relposned = func;
}

void ublox_set_rx_callback_rawx(void(*func)(ubx_rxm_rawx *pos)) {
	rx_rawx = func;
}

static THD_FUNCTION(process_thread, arg) {
	(void)arg;

	chRegSetThreadName("ublox process");

	process_tp = chThdGetSelfX();

	static uint8_t line[LINE_BUFFER_SIZE];
	static uint8_t ubx[UBX_BUFFER_SIZE];
	static int line_pos = 0;
	static int ubx_pos = 0;
	static uint8_t ubx_class;
	static uint8_t ubx_id;
	static uint8_t ubx_ck_a;
	static uint8_t ubx_ck_b;
	static int ubx_len;

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		while (serial_rx_read_pos != serial_rx_write_pos) {
			uint8_t ch = serial_rx_buffer[serial_rx_read_pos++];
			bool ch_used = false;

			if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_read_pos = 0;
			}

			// RTCM
			if (!ch_used && line_pos == 0 && ubx_pos == 0) {
				ch_used = rtcm3_input_data(ch, &rtcm_state) >= 0;
			}

			// Ubx
			if (!ch_used && line_pos == 0) {
				int ubx_pos_last = ubx_pos;

				if (ubx_pos == 0) {
					if (ch == 0xB5) {
						ubx_pos++;
					}
				} else if (ubx_pos == 1) {
					if (ch == 0x62) {
						ubx_pos++;
						ubx_ck_a = 0;
						ubx_ck_b = 0;
					}
				} else if (ubx_pos == 2) {
					ubx_class = ch;
					ubx_ck_a += ch;
					ubx_ck_b += ubx_ck_a;
					ubx_pos++;
				} else if (ubx_pos == 3) {
					ubx_id = ch;
					ubx_ck_a += ch;
					ubx_ck_b += ubx_ck_a;
					ubx_pos++;
				} else if (ubx_pos == 4) {
					ubx_len = ch;
					ubx_ck_a += ch;
					ubx_ck_b += ubx_ck_a;
					ubx_pos++;
				} else if (ubx_pos == 5) {
					ubx_len |= ch << 8;
					ubx_ck_a += ch;
					ubx_ck_b += ubx_ck_a;
					ubx_pos++;
				} else if ((ubx_pos - 6) < ubx_len) {
					ubx[ubx_pos - 6] = ch;
					ubx_ck_a += ch;
					ubx_ck_b += ubx_ck_a;
					ubx_pos++;
				} else if ((ubx_pos - 6) == ubx_len) {
					if (ch == ubx_ck_a) {
						ubx_pos++;
					} else {
						commands_printf("UBX Checksum A Error");
					}
				} else if ((ubx_pos - 6) == (ubx_len + 1)) {
					if (ch == ubx_ck_b) {
						ubx_decode(ubx_class, ubx_id, ubx, ubx_len);
						ubx_pos = 0;
					} else {
						commands_printf("UBX Checksum B Error");
					}
				}

				if (ubx_pos_last != ubx_pos) {
					ch_used = true;
				} else {
					ubx_pos = 0;
				}
			}

			// NMEA
			if (!ch_used) {
				line[line_pos++] = ch;
				if (line_pos == LINE_BUFFER_SIZE) {
					line_pos = 0;
				}

				if (line_pos > 0 && line[line_pos - 1] == '\n') {
					commands_send_nmea(line, line_pos);
					line[line_pos] = '\0';
					pos_input_nmea((const char*)line);
					line_pos = 0;
				}
			}
		}
	}
}

static void ubx_decode(uint8_t class, uint8_t id, uint8_t *msg, int len) {
	switch (class) {
	case UBX_CLASS_NAV: {
		switch (id) {
		case UBX_NAV_RELPOSNED:
			ubx_decode_relposned(msg, len);
			break;
		default:
			break;
		}
	} break;

	case UBX_CLASS_RXM: {
		switch (id) {
		case UBX_RXM_RAWX:
			ubx_decode_rawx(msg, len);
			break;
		default:
			break;
		}
	} break;

	default:
		break;
	}
}

static void ubx_decode_relposned(uint8_t *msg, int len) {
	(void)len;

	static ubx_nav_relposned pos;
	int ind = 2;
	uint32_t flags;

	pos.ref_station_id = ubx_get_U2(msg, &ind);
	pos.i_tow = ubx_get_U4(msg, &ind);
	pos.pos_n = (float)ubx_get_I4(msg, &ind) / 100.0;
	pos.pos_e = (float)ubx_get_I4(msg, &ind) / 100.0;
	pos.pos_d = (float)ubx_get_I4(msg, &ind) / 100.0;
	pos.pos_n += (float)ubx_get_I1(msg, &ind) / 10000.0;
	pos.pos_e += (float)ubx_get_I1(msg, &ind) / 10000.0;
	pos.pos_d += (float)ubx_get_I1(msg, &ind) / 10000.0;
	ind += 1;
	pos.acc_n = (float)ubx_get_U4(msg, &ind) / 10000.0;
	pos.acc_e = (float)ubx_get_U4(msg, &ind) / 10000.0;
	pos.acc_d = (float)ubx_get_U4(msg, &ind) / 10000.0;
	flags = ubx_get_X4(msg, &ind);
	pos.fix_ok = flags & 0x01;
	pos.diff_soln = flags & 0x02;
	pos.rel_pos_valid = flags & 0x04;
	pos.carr_soln = (flags >> 3) & 0x03;

	if (rx_relposned) {
		rx_relposned(&pos);
	}

#if PRINT_UBX_MSGS
	commands_printf(
			"NED RX\n"
			"i_tow: %d ms\n"
			"N: %.3f m\n"
			"E: %.3f m\n"
			"D: %.3f m\n"
			"N_ACC: %.3f m\n"
			"E_ACC: %.3f m\n"
			"D_ACC: %.3f m\n"
			"Fix OK: %d\n"
			"Diff Soln: %d\n"
			"Rel Pos Valid: %d\n"
			"Carr Soln: %d\n",
			pos.i_tow,
			(double)pos.pos_n, (double)pos.pos_e, (double)pos.pos_d,
			(double)pos.acc_n, (double)pos.acc_e, (double)pos.acc_d,
			pos.fix_ok, pos.diff_soln, pos.rel_pos_valid, pos.carr_soln);
#endif
}

// Note: Message version 0x01
static void ubx_decode_rawx(uint8_t *msg, int len) {
	(void)len;

	static ubx_rxm_rawx raw;
	int ind = 0;
	uint32_t flags;

	raw.rcv_tow = ubx_get_R8(msg, &ind);
	raw.week = ubx_get_U2(msg, &ind);
	raw.leap_sec = ubx_get_I1(msg, &ind);
	raw.num_meas = ubx_get_U1(msg, &ind);
	flags = ubx_get_X1(msg, &ind);
	raw.leap_sec = flags & 0x01;
	raw.clk_reset = flags & 0x02;

	ind = 16;

	if (raw.num_meas > 40) {
		commands_printf("Too many raw measurements to store in buffer: %d\n", raw.num_meas);
		return;
	}

	for (int i = 0;i < raw.num_meas;i++) {
		raw.obs[i].pr_mes = ubx_get_R8(msg, &ind);
		raw.obs[i].cp_mes = ubx_get_R8(msg, &ind);
		raw.obs[i].do_mes = ubx_get_R4(msg, &ind);
		raw.obs[i].gnss_id = ubx_get_U1(msg, &ind);
		raw.obs[i].sv_id = ubx_get_U1(msg, &ind);
		ind += 1;
		raw.obs[i].freq_id = ubx_get_U1(msg, &ind);
		raw.obs[i].locktime = ubx_get_U2(msg, &ind);
		raw.obs[i].cno = ubx_get_U1(msg, &ind);
		raw.obs[i].pr_stdev = ubx_get_X1(msg, &ind) & 0x0F;
		raw.obs[i].cp_stdev = ubx_get_X1(msg, &ind) & 0x0F;
		raw.obs[i].do_stdev = ubx_get_X1(msg, &ind) & 0x0F;
		flags = ubx_get_X1(msg, &ind);
		raw.obs[i].pr_valid = flags & 0x01;
		raw.obs[i].cp_valid = flags & 0x02;
		raw.obs[i].half_cyc_valid = flags & 0x04;
		raw.obs[i].half_cyc_sub = flags & 0x08;
		ind += 1;
	}

	if (rx_rawx) {
		rx_rawx(&raw);
	}

#if PRINT_UBX_MSGS
	// TODO: Print table of observations?
	commands_printf(
			"RAWX_RX\n"
			"tow: %.3f\n"
			"week: %d\n"
			"leap_sec: %d\n"
			"num_meas: %d\n"
			"pr_0: %.3f\n"
			"pr_1: %.3f\n",
			raw.rcv_tow, raw.week, raw.leap_sec, raw.num_meas,
			raw.obs[0].pr_mes, raw.obs[1].pr_mes);
#endif
}

static uint8_t ubx_get_U1(uint8_t *msg, int *ind) {
	return msg[(*ind)++];
}

static int8_t ubx_get_I1(uint8_t *msg, int *ind) {
	return (int8_t)msg[(*ind)++];
}

static uint8_t ubx_get_X1(uint8_t *msg, int *ind) {
	return msg[(*ind)++];
}

static uint16_t ubx_get_U2(uint8_t *msg, int *ind) {
	uint16_t res =	((uint16_t) msg[*ind + 1]) << 8 |
					((uint16_t) msg[*ind]);
	*ind += 2;
	return res;
}

static int16_t ubx_get_I2(uint8_t *msg, int *ind) {
	int16_t res =	((uint16_t) msg[*ind + 1]) << 8 |
					((uint16_t) msg[*ind]);
	*ind += 2;
	return res;
}

static uint16_t ubx_get_X2(uint8_t *msg, int *ind) {
	uint16_t res =	((uint16_t) msg[*ind + 1]) << 8 |
					((uint16_t) msg[*ind]);
	*ind += 2;
	return res;
}

static uint32_t ubx_get_U4(uint8_t *msg, int *ind) {
	uint32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
					((uint32_t) msg[*ind + 2]) << 16 |
					((uint32_t) msg[*ind + 1]) << 8 |
					((uint32_t) msg[*ind]);
	*ind += 4;
	return res;
}

static int32_t ubx_get_I4(uint8_t *msg, int *ind) {
	int32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
					((uint32_t) msg[*ind + 2]) << 16 |
					((uint32_t) msg[*ind + 1]) << 8 |
					((uint32_t) msg[*ind]);
	*ind += 4;
	return res;
}

static uint32_t ubx_get_X4(uint8_t *msg, int *ind) {
	uint32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
					((uint32_t) msg[*ind + 2]) << 16 |
					((uint32_t) msg[*ind + 1]) << 8 |
					((uint32_t) msg[*ind]);
	*ind += 4;
	return res;
}

static float ubx_get_R4(uint8_t *msg, int *ind) {
	uint32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
					((uint32_t) msg[*ind + 2]) << 16 |
					((uint32_t) msg[*ind + 1]) << 8 |
					((uint32_t) msg[*ind]);
	*ind += 4;

	union asd {
	     float f;
	     uint32_t i;
	} x;

	x.i = res;

	return x.f;
}

static double ubx_get_R8(uint8_t *msg, int *ind) {
	uint64_t res =	((uint64_t) msg[*ind + 7]) << 56 |
					((uint64_t) msg[*ind + 6]) << 48 |
					((uint64_t) msg[*ind + 5]) << 40 |
					((uint64_t) msg[*ind + 4]) << 32 |
					((uint64_t) msg[*ind + 3]) << 24 |
					((uint64_t) msg[*ind + 2]) << 16 |
					((uint64_t) msg[*ind + 1]) << 8 |
					((uint64_t) msg[*ind]);
	*ind += 8;

	union asd {
	     double f;
	     uint64_t i;
	} x;

	x.i = res;

	return x.f;
}
