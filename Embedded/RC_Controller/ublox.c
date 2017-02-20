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
#include "terminal.h"
#include "comm_cc1120.h"
#include "comm_cc2520.h"

#include <string.h>
#include <math.h>

// Settings
#define HW_UART_DEV					UARTD6
#define HW_UBX_TX_PORT				GPIOC
#define HW_UBX_TX_PIN				6
#define HW_UBX_RX_PORT				GPIOC
#define HW_UBX_RX_PIN				7
#define HW_UBX_RESET_PORT			GPIOC
#define HW_UBX_RESET_PIN			9
#define BAUDRATE					115200
#define SERIAL_RX_BUFFER_SIZE		1024
#define LINE_BUFFER_SIZE			256
#define UBX_BUFFER_SIZE				2048
#define PRINT_UBX_MSGS				1
#define CFG_ACK_WAIT_MS				100

// Threads
static THD_FUNCTION(process_thread, arg);
static THD_WORKING_AREA(process_thread_wa, 4096);
static thread_t *process_tp = 0;
static thread_t *ack_wait_tp = 0;

// Variables
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;
static rtcm3_state rtcm_state;

// Private functions
static void ubx_terminal_cmd_poll(int argc, const char **argv);
static void ubx_encode_send(uint8_t class, uint8_t id, uint8_t *msg, int len);
static int wait_ack_nak(int timeout_ms);
static void rtcm_rx(uint8_t *data, int len, int type);

// Decode functions
static void ubx_decode(uint8_t class, uint8_t id, uint8_t *msg, int len);
static void ubx_decode_relposned(uint8_t *msg, int len);
static void ubx_decode_svin(uint8_t *msg, int len);
static void ubx_decode_ack(uint8_t *msg, int len);
static void ubx_decode_nak(uint8_t *msg, int len);
static void ubx_decode_rawx(uint8_t *msg, int len);

// Ublox type getters
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

// Ublox type setters
static void ubx_put_U1(uint8_t *msg, int *ind, uint8_t data);
static void ubx_put_I1(uint8_t *msg, int *ind, int8_t data);
static void ubx_put_X1(uint8_t *msg, int *ind, uint8_t data);
static void ubx_put_U2(uint8_t *msg, int *ind, uint16_t data);
static void ubx_put_I2(uint8_t *msg, int *ind, int16_t data);
static void ubx_put_X2(uint8_t *msg, int *ind, uint16_t data);
static void ubx_put_U4(uint8_t *msg, int *ind, uint32_t data);
static void ubx_put_I4(uint8_t *msg, int *ind, int32_t data);
static void ubx_put_X4(uint8_t *msg, int *ind, uint32_t data);
static void ubx_put_R4(uint8_t *msg, int *ind, float data);
static void ubx_put_R8(uint8_t *msg, int *ind, double data);

// Callbacks
static void(*rx_relposned)(ubx_nav_relposned *pos) = 0;
static void(*rx_rawx)(ubx_rxm_rawx *pos) = 0;
static void(*rx_svin)(ubx_nav_svin *svin) = 0;

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
	palSetPadMode(HW_UBX_TX_PORT, HW_UBX_TX_PIN, PAL_MODE_ALTERNATE(8));
	palSetPadMode(HW_UBX_RX_PORT, HW_UBX_RX_PIN, PAL_MODE_ALTERNATE(8));
	palSetPadMode(HW_UBX_RESET_PORT, HW_UBX_RESET_PIN, PAL_MODE_OUTPUT_PUSHPULL);
	
	palClearPad(HW_UBX_RESET_PORT, HW_UBX_RESET_PIN);
	chThdSleepMilliseconds(10);
	palSetPad(HW_UBX_RESET_PORT, HW_UBX_RESET_PIN);
	chThdSleepMilliseconds(1000);

	uartStart(&HW_UART_DEV, &uart_cfg);

	rtcm3_init_state(&rtcm_state);
	rtcm3_set_rx_callback(rtcm_rx, &rtcm_state);

	chThdCreateStatic(process_thread_wa, sizeof(process_thread_wa), NORMALPRIO, process_thread, NULL);

	terminal_register_command_callback(
			"ubx_poll",
			"Poll one of the ubx protocol messages. Supported messages:\n"
			"  UBX_NAV_RELPOSNED - Relative position to base in NED frame\n"
			"  UBX_NAV_SVIN - survey-in data\n"
			"  UBX_RXM_RAWX - raw data",
			"[msg]",
			ubx_terminal_cmd_poll);

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

	(void)ubx_put_U1;
	(void)ubx_put_I1;
	(void)ubx_put_X1;
	(void)ubx_put_U2;
	(void)ubx_put_I2;
	(void)ubx_put_X2;
	(void)ubx_put_U4;
	(void)ubx_put_I4;
	(void)ubx_put_X4;
	(void)ubx_put_R4;
	(void)ubx_put_R8;

	chThdSleepMilliseconds(100);

	// Disable survey in
	ubx_cfg_tmode3 tmode3;
	memset(&tmode3, 0, sizeof(ubx_cfg_tmode3));
	ublox_cfg_tmode3(&tmode3);

	// Switch off RTCM message output
	ublox_cfg_msg(UBX_CLASS_RTCM3, UBX_RTCM3_1005, 0);
	ublox_cfg_msg(UBX_CLASS_RTCM3, UBX_RTCM3_1077, 0);
	ublox_cfg_msg(UBX_CLASS_RTCM3, UBX_RTCM3_1087, 0);

	// Set rate to 5 Hz and time reference to UTC
	ublox_cfg_rate(200, 1, 0);

	// Dynamic model
	ubx_cfg_nav5 nav5;
	memset(&nav5, 0, sizeof(ubx_cfg_nav5));
	nav5.apply_dyn = true;
	nav5.dyn_model = 4;
	ublox_cfg_nav5(&nav5);
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

void ublox_set_rx_callback_svin(void(*func)(ubx_nav_svin *pos)) {
	rx_svin = func;
}

void ublox_poll(uint8_t msg_class, uint8_t id) {
	ubx_encode_send(msg_class, id, 0, 0);
}

/**
 * Set the tmode3 configuration.
 *
 * @param cfg
 * The configuration.
 *
 * @return
 * 0: Ack received
 * 1: Nak received (rejected)
 * -1: Timeout when waiting for ack/nak
 */
int ublox_cfg_tmode3(ubx_cfg_tmode3 *cfg) {
	uint8_t buffer[40];
	int ind = 0;

	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	uint16_t flags = ((cfg->lla ? 1 : 0) << 8) | cfg->mode;
	ubx_put_X2(buffer, &ind, flags);

	int32_t x_lat = 0;
	int32_t y_lon = 0;
	int32_t z_alt = 0;
	int8_t x_lat_hp = 0;
	int8_t y_lon_hp = 0;
	int8_t z_alt_hp = 0;
	if (cfg->lla) {
		x_lat = round(cfg->ecefx_lat * D(1e7));
		y_lon = round(cfg->ecefy_lon * D(1e7));
		z_alt = round(cfg->ecefz_alt * D(1e2));
		x_lat_hp = ((cfg->ecefx_lat - ((double)x_lat * D(1e-7))) * D(1e9));
		y_lon_hp = ((cfg->ecefy_lon - ((double)y_lon * D(1e-7))) * D(1e9));
		z_alt_hp = ((cfg->ecefz_alt - ((double)z_alt * D(1e-2))) * D(1e4));
	} else {
		x_lat = cfg->ecefx_lat * D(1e2);
		y_lon = cfg->ecefy_lon * D(1e2);
		z_alt = cfg->ecefz_alt * D(1e2);
		x_lat_hp = ((cfg->ecefx_lat - ((double)x_lat * D(1e-2))) * D(1e4));
		y_lon_hp = ((cfg->ecefy_lon - ((double)y_lon * D(1e-2))) * D(1e4));
		z_alt_hp = ((cfg->ecefz_alt - ((double)z_alt * D(1e-2))) * D(1e4));
	}

	ubx_put_I4(buffer, &ind, x_lat);
	ubx_put_I4(buffer, &ind, y_lon);
	ubx_put_I4(buffer, &ind, z_alt);
	ubx_put_I1(buffer, &ind, x_lat_hp);
	ubx_put_I1(buffer, &ind, y_lon_hp);
	ubx_put_I1(buffer, &ind, z_alt_hp);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U4(buffer, &ind, cfg->fixed_pos_acc * 1e4);
	ubx_put_U4(buffer, &ind, cfg->svin_min_dur);
	ubx_put_U4(buffer, &ind, cfg->svin_acc_limit * 1e4);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);

	ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_TMODE3, buffer, ind);
	return wait_ack_nak(CFG_ACK_WAIT_MS);
}

/**
 * Set the msg output configuration.
 *
 * @param msg_class
 * The the message class.
 *
 * @param id
 * The message id
 *
 * @param
 * The message rate. 0 = disbaled
 *
 * @return
 * 0: Ack received
 * 1: Nak received (rejected)
 * -1: Timeout when waiting for ack/nak
 */
int ublox_cfg_msg(uint8_t msg_class, uint8_t id, uint8_t rate) {
	uint8_t buffer[8];
	int ind = 0;

	ubx_put_U1(buffer, &ind, msg_class);
	ubx_put_U1(buffer, &ind, id);
	ubx_put_U1(buffer, &ind, rate);
	ubx_put_U1(buffer, &ind, rate);
	ubx_put_U1(buffer, &ind, rate);
	ubx_put_U1(buffer, &ind, rate);
	ubx_put_U1(buffer, &ind, rate);
	ubx_put_U1(buffer, &ind, rate);

	ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_MSG, buffer, ind);
	return wait_ack_nak(CFG_ACK_WAIT_MS);
}

/**
 * Set the measurement rate, navigation rate and time reference.
 *
 * @param meas_rate_ms
 * The elapsed time between GNSS measurements, which defines the rate,
 * e.g. 100ms => 10Hz, 1000ms => 1Hz, 10000ms => 0.1Hz
 *
 * @param nav_rate_ms
 * The ratio between the number of measurements and the number of navigation
 * solutions, e.g. 5 means five measurements for every navigation solution.
 * Max. value is 127. (This parameter is ignored and the navRate is fixed to 1
 * in protocol versions less than 18)
 *
 * @param time_ref
 * The time system to which measurements are aligned:
 * 0: UTC time
 * 1: GPS time
 * 2: GLONASS time (not supported in protocol versions less than 18)
 * 3: BeiDou time (not supported in protocol versions less than 18)
 * 4: Galileo time (not supported in protocol versions less than 18)
 *
 * @return
 * 0: Ack received
 * 1: Nak received (rejected)
 * -1: Timeout when waiting for ack/nak
 */
int ublox_cfg_rate(uint16_t meas_rate_ms, uint16_t nav_rate_ms, uint16_t time_ref) {
	uint8_t buffer[6];
	int ind = 0;

	ubx_put_U2(buffer, &ind, meas_rate_ms);
	ubx_put_U2(buffer, &ind, nav_rate_ms);
	ubx_put_U2(buffer, &ind, time_ref);

	ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_RATE, buffer, ind);
	return wait_ack_nak(CFG_ACK_WAIT_MS);
}

/**
 * Set the nav5 configuration.
 *
 * @param cfg
 * The configuration.
 *
 * @return
 * 0: Ack received
 * 1: Nak received (rejected)
 * -1: Timeout when waiting for ack/nak
 */
int ublox_cfg_nav5(ubx_cfg_nav5 *cfg) {
	uint8_t buffer[36];
	int ind = 0;

	uint16_t mask = 0;
	mask |= (cfg->apply_dyn ? 1 : 0) << 0;
	mask |= (cfg->apply_min_el ? 1 : 0) << 1;
	mask |= (cfg->apply_pos_fix_mode ? 1 : 0) << 2;
	mask |= (cfg->apply_pos_mask ? 1 : 0) << 4;
	mask |= (cfg->apply_time_mask ? 1 : 0) << 5;
	mask |= (cfg->apply_static_hold_mask ? 1 : 0) << 6;
	mask |= (cfg->apply_dgps ? 1 : 0) << 7;
	mask |= (cfg->apply_cno ? 1 : 0) << 8;
	mask |= (cfg->apply_utc ? 1 : 0) << 10;

	ubx_put_X2(buffer, &ind, mask);
	ubx_put_U1(buffer, &ind, cfg->dyn_model);
	ubx_put_U1(buffer, &ind, cfg->fix_mode);
	ubx_put_I4(buffer, &ind, cfg->fixed_alt * D(100.0));
	ubx_put_U4(buffer, &ind, cfg->fixed_alt_var * D(10000.0));
	ubx_put_I1(buffer, &ind, cfg->min_elev);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U2(buffer, &ind, cfg->p_dop * 10.0);
	ubx_put_U2(buffer, &ind, cfg->t_dop * 10.0);
	ubx_put_U2(buffer, &ind, cfg->p_acc);
	ubx_put_U2(buffer, &ind, cfg->t_acc);
	ubx_put_U1(buffer, &ind, cfg->static_hold_thres);
	ubx_put_U1(buffer, &ind, cfg->dgnss_timeout);
	ubx_put_U1(buffer, &ind, cfg->cno_tres_num_sat);
	ubx_put_U1(buffer, &ind, cfg->cno_tres);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U2(buffer, &ind, cfg->static_hold_max_dist);
	ubx_put_U1(buffer, &ind, cfg->utc_standard);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);
	ubx_put_U1(buffer, &ind, 0);

	ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_NAV5, buffer, ind);
	return wait_ack_nak(CFG_ACK_WAIT_MS);
}

static THD_FUNCTION(process_thread, arg) {
	(void)arg;

	chRegSetThreadName("ublox process");

	process_tp = chThdGetSelfX();

	static uint8_t line[LINE_BUFFER_SIZE];
	static uint8_t ubx[UBX_BUFFER_SIZE];
	int line_pos = 0;
	int ubx_pos = 0;
	uint8_t ubx_class = 0;
	uint8_t ubx_id = 0;
	uint8_t ubx_ck_a = 0;
	uint8_t ubx_ck_b = 0;
	int ubx_len = 0;

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
					line[line_pos] = '\0';
					line_pos = 0;

#if MAIN_MODE == MAIN_MODE_CAR
					bool found = pos_input_nmea((const char*)line);

					// Only send the lines that pos decoded
					if (found) {
						commands_send_nmea(line, strlen((char*)line));
					}
#endif
				}
			}
		}
	}
}

static void ubx_terminal_cmd_poll(int argc, const char **argv) {
	if (argc == 2) {
		if (strcmp(argv[1], "UBX_NAV_RELPOSNED") == 0) {
			ublox_poll(UBX_CLASS_NAV, UBX_NAV_RELPOSNED);
			commands_printf("OK\n");
		} else if (strcmp(argv[1], "UBX_NAV_SVIN") == 0) {
			ublox_poll(UBX_CLASS_NAV, UBX_NAV_SVIN);
			commands_printf("OK\n");
		} else if (strcmp(argv[1], "UBX_RXM_RAWX") == 0) {
			ublox_poll(UBX_CLASS_RXM, UBX_RXM_RAWX);
			commands_printf("OK\n");
		} else {
			commands_printf("Wrong argument %s\n", argv[1]);
		}
	} else {
		commands_printf("Wrong number of arguments\n");
	}
}

static void ubx_encode_send(uint8_t class, uint8_t id, uint8_t *msg, int len) {
	static uint8_t ubx[UBX_BUFFER_SIZE];
	int ind = 0;
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;

	ubx[ind++] = 0xB5;
	ubx[ind++] = 0x62;

	ubx[ind] = class;
	ck_a += ubx[ind];
	ck_b += ck_a;
	ind++;

	ubx[ind] = id;
	ck_a += ubx[ind];
	ck_b += ck_a;
	ind++;

	ubx[ind] = len & 0xFF;
	ck_a += ubx[ind];
	ck_b += ck_a;
	ind++;

	ubx[ind] = (len >> 8) & 0xFF;
	ck_a += ubx[ind];
	ck_b += ck_a;
	ind++;

	for (int i = 0;i < len;i++) {
		ubx[ind] = msg[i];
		ck_a += ubx[ind];
		ck_b += ck_a;
		ind++;
	}

	ubx[ind++] = ck_a;
	ubx[ind++] = ck_b;

	ublox_send(ubx, ind);
}

/**
 * Wait for ack or nak.
 *
 * @param timeout_ms
 * The timeout for the wait, -1 = infinite.
 *
 * @return
 * 0: ack
 * 1: nak
 * -1: timeout
 *
 */
static int wait_ack_nak(int timeout_ms) {
	systime_t to;
	if (timeout_ms >= 0) {
		to = MS2ST(timeout_ms);
	} else {
		to = TIME_INFINITE;
	}

	ack_wait_tp = chThdGetSelfX();
	int res = chEvtWaitAnyTimeout((eventmask_t)3, to);
	ack_wait_tp = 0;

	if (res == 1) {
		return 0;
	} else if (res == 2) {
		return 1;
	} else {
		return -1;
	}
}

static void rtcm_rx(uint8_t *data, int len, int type) {
	(void)type;
#if MAIN_MODE == MAIN_MODE_MOTE_2400
#if MAIN_MODE_MOTE_RTCM_400 == 1
	comm_cc1120_send_buffer(data, len);
#else
	comm_cc2520_send_buffer(data, len);
#endif
#else
	comm_cc1120_send_buffer(data, len);
#endif
}

static void ubx_decode(uint8_t class, uint8_t id, uint8_t *msg, int len) {
	switch (class) {
	case UBX_CLASS_NAV: {
		switch (id) {
		case UBX_NAV_RELPOSNED:
			ubx_decode_relposned(msg, len);
			break;
		case UBX_NAV_SVIN:
			ubx_decode_svin(msg, len);
			break;
		default:
			break;
		}
	} break;

	case UBX_CLASS_ACK: {
		switch (id) {
		case UBX_ACK_ACK:
			ubx_decode_ack(msg, len);
			break;

		case UBX_ACK_NAK:
			ubx_decode_nak(msg, len);
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
	} else {
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
}

static void ubx_decode_svin(uint8_t *msg, int len) {
	(void)len;

	static ubx_nav_svin svin;
	int ind = 4;

	svin.i_tow = ubx_get_U4(msg, &ind);
	svin.dur = ubx_get_U4(msg, &ind);
	svin.meanX = (double)ubx_get_I4(msg, &ind) / D(100.0);
	svin.meanY = (double)ubx_get_I4(msg, &ind) / D(100.0);
	svin.meanZ = (double)ubx_get_I4(msg, &ind) / D(100.0);
	svin.meanX += (double)ubx_get_I1(msg, &ind) / D(10000.0);
	svin.meanY += (double)ubx_get_I1(msg, &ind) / D(10000.0);
	svin.meanZ += (double)ubx_get_I1(msg, &ind) / D(10000.0);
	ind += 1;
	svin.meanAcc = (float)ubx_get_U4(msg, &ind) / 10000.0;
	svin.obs = ubx_get_U4(msg, &ind);
	svin.valid = ubx_get_U1(msg, &ind);
	svin.active = ubx_get_U1(msg, &ind);

	if (rx_svin) {
		rx_svin(&svin);
	} else {
#if PRINT_UBX_MSGS
		commands_printf(
				"SVIN RX\n"
				"i_tow: %d ms\n"
				"dur: %d s\n"
				"Mean X: %.3f m\n"
				"Mean Y: %.3f m\n"
				"Mean Z: %.3f m\n"
				"Mean ACC: %.3f m\n"
				"Valid: %d\n"
				"Active: %d\n",
				svin.i_tow, svin.dur,
				svin.meanX, svin.meanY, svin.meanZ, (double)svin.meanAcc,
				svin.valid, svin.active);
#endif
	}
}

static void ubx_decode_ack(uint8_t *msg, int len) {
	(void)len;

	int ind = 0;

	uint8_t cls_id = ubx_get_I1(msg, &ind);
	uint8_t msg_id = ubx_get_I1(msg, &ind);

	// TODO: Use these
	(void)cls_id;
	(void)msg_id;

	if (ack_wait_tp) {
		chEvtSignalI(ack_wait_tp, (eventmask_t)1);
	}
}

static void ubx_decode_nak(uint8_t *msg, int len) {
	(void)len;

	int ind = 0;

	uint8_t cls_id = ubx_get_I1(msg, &ind);
	uint8_t msg_id = ubx_get_I1(msg, &ind);

	// TODO: Use these
	(void)cls_id;
	(void)msg_id;

	if (ack_wait_tp) {
		chEvtSignalI(ack_wait_tp, (eventmask_t)2);
	}
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
	} else {
#if PRINT_UBX_MSGS
		// TODO: Print table of observations?
		commands_printf(
				"RAWX RX\n"
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

static void ubx_put_U1(uint8_t *msg, int *ind, uint8_t data) {
	msg[(*ind)++] = data;
}

static void ubx_put_I1(uint8_t *msg, int *ind, int8_t data) {
	msg[(*ind)++] = data;
}

static void ubx_put_X1(uint8_t *msg, int *ind, uint8_t data) {
	msg[(*ind)++] = data;
}

static void ubx_put_U2(uint8_t *msg, int *ind, uint16_t data) {
	msg[(*ind)++] = data;
	msg[(*ind)++] = data >> 8;
}

static void ubx_put_I2(uint8_t *msg, int *ind, int16_t data) {
	msg[(*ind)++] = data;
	msg[(*ind)++] = data >> 8;
}

static void ubx_put_X2(uint8_t *msg, int *ind, uint16_t data) {
	msg[(*ind)++] = data;
	msg[(*ind)++] = data >> 8;
}

static void ubx_put_U4(uint8_t *msg, int *ind, uint32_t data) {
	msg[(*ind)++] = data;
	msg[(*ind)++] = data >> 8;
	msg[(*ind)++] = data >> 16;
	msg[(*ind)++] = data >> 24;
}

static void ubx_put_I4(uint8_t *msg, int *ind, int32_t data) {
	msg[(*ind)++] = data;
	msg[(*ind)++] = data >> 8;
	msg[(*ind)++] = data >> 16;
	msg[(*ind)++] = data >> 24;
}

static void ubx_put_X4(uint8_t *msg, int *ind, uint32_t data) {
	msg[(*ind)++] = data;
	msg[(*ind)++] = data >> 8;
	msg[(*ind)++] = data >> 16;
	msg[(*ind)++] = data >> 24;
}

static void ubx_put_R4(uint8_t *msg, int *ind, float data) {
	union asd {
		float f;
		uint64_t i;
	} x;

	x.f = data;

	msg[(*ind)++] = x.i;
	msg[(*ind)++] = x.i >> 8;
	msg[(*ind)++] = x.i >> 16;
	msg[(*ind)++] = x.i >> 24;
}

static void ubx_put_R8(uint8_t *msg, int *ind, double data) {
	union asd {
		double f;
		uint64_t i;
	} x;

	x.f = data;

	msg[(*ind)++] = x.i;
	msg[(*ind)++] = x.i >> 8;
	msg[(*ind)++] = x.i >> 16;
	msg[(*ind)++] = x.i >> 24;
	msg[(*ind)++] = x.i >> 32;
	msg[(*ind)++] = x.i >> 40;
	msg[(*ind)++] = x.i >> 48;
	msg[(*ind)++] = x.i >> 56;
}
