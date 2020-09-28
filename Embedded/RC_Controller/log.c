/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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

#include "log.h"
#include "pos.h"
#include "commands.h"
#include "servo_simple.h"
#include "comm_can.h"
#include "ch.h"
#include "hal.h"

#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>

// Settings
#define UART_DEV					UARTD1
#define UART_TX_PORT				GPIOB
#define UART_TX_PIN					6
#define UART_RX_PORT				GPIOB
#define UART_RX_PIN					7
#define UART_ALT_PIN_FUNCTION		7
#define UART_LOG_RX_BUFFER_SIZE		256

// Defines
#define LOG_MODE_IS_UART(mode)		(mode == LOG_EXT_UART || mode == LOG_EXT_UART_POLLED)

// private variables
static int m_log_rate_hz;
static bool m_log_en;
static bool m_write_split;
static char m_log_name[LOG_NAME_MAX_LEN + 1];
static LOG_EXT_MODE m_log_ext_mode;
static int m_log_uart_baud;
static char m_log_uart_rx_buffer[UART_LOG_RX_BUFFER_SIZE];
static int m_log_uart_rx_ptr;

// Threads
static THD_WORKING_AREA(log_thread_wa, 2048);
static THD_FUNCTION(log_thread, arg);
static THD_WORKING_AREA(log_uart_thread_wa, 2048);
static THD_FUNCTION(log_uart_thread, arg);
static thread_t *log_uart_tp;

// Private functions
static void print_log_ext(void);
static void txend1(UARTDriver *uartp);
static void txend2(UARTDriver *uartp);
static void rxerr(UARTDriver *uartp, uartflags_t e);
static void rxchar(UARTDriver *uartp, uint16_t c);
static void rxend(UARTDriver *uartp);
static void set_baudrate(uint32_t baud);
static void printf_blocking(bool ethernet, char* format, ...);
static void write_blocking(unsigned char *data, unsigned int len);

// Configuration structures
static UARTConfig uart_cfg = {
		txend1,
		txend2,
		rxend,
		rxchar,
		rxerr,
		115200,
		0,
		USART_CR2_LINEN,
		0
};

void log_init(void) {
	m_log_rate_hz = 20;
	m_log_en = false;
	m_write_split = true;
	strcpy(m_log_name, "Undefined");
	m_log_ext_mode = LOG_EXT_OFF;
	m_log_uart_baud = 115200;
	m_log_uart_rx_ptr = 0;
	memset(m_log_uart_rx_buffer, 0, sizeof(m_log_uart_rx_buffer));

	chThdCreateStatic(log_thread_wa, sizeof(log_thread_wa),
			NORMALPRIO, log_thread, NULL);
	chThdCreateStatic(log_uart_thread_wa, sizeof(log_uart_thread_wa),
			NORMALPRIO, log_uart_thread, NULL);
}

void log_set_rate(int rate_hz) {
	m_log_rate_hz = rate_hz;
}

void log_set_enabled(bool enabled) {
	if (enabled && !m_log_en) {
		m_write_split = true;
	}

	m_log_en = enabled;
}

void log_set_name(char *name) {
	strcpy(m_log_name, name);
}

void log_set_ext(LOG_EXT_MODE mode, int baud) {
	if (LOG_MODE_IS_UART(mode) && !LOG_MODE_IS_UART(m_log_ext_mode)) {
		palSetPadMode(UART_TX_PORT, UART_TX_PIN, PAL_MODE_ALTERNATE(UART_ALT_PIN_FUNCTION));
		palSetPadMode(UART_RX_PORT, UART_RX_PIN, PAL_MODE_ALTERNATE(UART_ALT_PIN_FUNCTION));
		uartStart(&UART_DEV, &uart_cfg);
	} else if (!LOG_MODE_IS_UART(mode) && LOG_MODE_IS_UART(m_log_ext_mode)) {
		uartStop(&UART_DEV);
		palSetPadMode(UART_TX_PORT, UART_TX_PIN, PAL_MODE_RESET);
		palSetPadMode(UART_RX_PORT, UART_RX_PIN, PAL_MODE_RESET);
	}

	m_log_ext_mode = mode;

	if (LOG_MODE_IS_UART(m_log_ext_mode) > 0) {
		set_baudrate(baud);
	}
}

static THD_FUNCTION(log_thread, arg) {
	(void)arg;

	chRegSetThreadName("Log");

	systime_t time_p = chVTGetSystemTimeX(); // T0

	for(;;) {
		if (m_log_ext_mode == LOG_EXT_UART || m_log_ext_mode == LOG_EXT_ETHERNET) {
			print_log_ext();
		}

		if (m_log_en) {
			if (m_write_split) {
				commands_printf_log_usb("//%s\n", m_log_name);
				m_write_split = false;
			}

#ifdef LOG_EN_CARREL
			mc_values val;
			POS_STATE pos;
			GPS_STATE gps;
			float accel[3];
			float gyro[3];
			float mag[3];

			float steering_angle = (servo_simple_get_pos_now()
					- main_config.car.steering_center)
									* ((2.0 * main_config.car.steering_max_angle_rad)
											/ main_config.car.steering_range);

			pos_get_mc_val(&val);
			pos_get_pos(&pos);
			pos_get_gps(&gps);
			pos_get_imu(accel, gyro, mag);
			uint32_t time = chVTGetSystemTimeX();

			commands_printf_log_usb(
					"%u "     // timestamp
					"%.2f "   // temp_mos
					"%.2f "   // current_in
					"%.2f "   // current_motor
					"%.2f "   // v_in
					"%.3f "   // car x
					"%.3f "   // car y
					"%.3f "   // gps x
					"%.3f "   // gps y
					"%.3f "   // gps z
					"%.3f "   // gps ENU initial x
					"%.3f "   // gps ENU initial y
					"%.3f "   // gps ENU initial z
					"%.3f "   // speed
					"%.2f "   // roll
					"%.2f "   // pitch
					"%.2f "   // yaw
					"%.3f "   // accel[0]
					"%.3f "   // accel[1]
					"%.3f "   // accel[2]
					"%.1f "   // gyro[0]
					"%.1f "   // gyro[1]
					"%.1f "   // gyro[2]
					"%.1f "   // mag[0]
					"%.1f "   // mag[1]
					"%.1f "   // mag[2]
					"%d "     // tachometer
					"%.3f\n", // commanded steering angle

					time,
					(double)val.temp_mos,
					(double)val.current_in,
					(double)val.current_motor,
					(double)val.v_in,
					(double)pos.px,
					(double)pos.py,
					(double)gps.lx,
					(double)gps.ly,
					(double)gps.lz,
					gps.ix,
					gps.iy,
					gps.iz,
					(double)pos.speed,
					(double)pos.roll,
					(double)pos.pitch,
					(double)pos.yaw,
					(double)accel[0],
					(double)accel[1],
					(double)accel[2],
					(double)gyro[0],
					(double)gyro[1],
					(double)gyro[2],
					(double)mag[0],
					(double)mag[1],
					(double)mag[2],
					val.tachometer,
					(double)steering_angle);
#elif defined(LOG_EN_ITRANSIT)
			mc_values val;
			GPS_STATE gps;
			float accel[3];
			float gyro[3];
			float mag[3];

			float steering_angle = (servo_simple_get_pos_now()
					- main_config.car.steering_center)
												* ((2.0 * main_config.car.steering_max_angle_rad)
														/ main_config.car.steering_range);

			pos_get_mc_val(&val);
			pos_get_gps(&gps);
			pos_get_imu(accel, gyro, mag);
			uint32_t time = chVTGetSystemTimeX();

			commands_printf_log_usb(
					"%u "     // timestamp
					"%u "     // gps ms
					"%.8f "   // Lat
					"%.8f "   // Lon
					"%.3f "   // height
					"%u "     // fix type
					"%.3f "   // x
					"%.3f "   // y
					"%.3f "   // z
					"%.3f "   // ix
					"%.3f "   // iy
					"%.3f "   // iz
					"%.3f "   // accel[0]
					"%.3f "   // accel[1]
					"%.3f "   // accel[2]
					"%.1f "   // gyro[0]
					"%.1f "   // gyro[1]
					"%.1f "   // gyro[2]
					"%.1f "   // mag[0]
					"%.1f "   // mag[1]
					"%.1f "   // mag[2]
					"%d "     // tachometer
					"%.3f\n", // commanded steering angle

					time,
					gps.ms,
					gps.lat,
					gps.lon,
					gps.height,
					gps.fix_type,
					(double)gps.lx,
					(double)gps.ly,
					(double)gps.lz,
					gps.ix,
					gps.iy,
					gps.iz,
					(double)accel[0],
					(double)accel[1],
					(double)accel[2],
					(double)gyro[0],
					(double)gyro[1],
					(double)gyro[2],
					(double)mag[0],
					(double)mag[1],
					(double)mag[2],
					val.tachometer,
					(double)steering_angle);
#endif
		}

		time_p += CH_CFG_ST_FREQUENCY / m_log_rate_hz;
		systime_t time = chVTGetSystemTimeX();

		if (time_p >= time + 5) {
			chThdSleepUntil(time_p);
		} else {
			chThdSleepMilliseconds(1);
		}
	}
}

static THD_FUNCTION(log_uart_thread, arg) {
	(void)arg;

	chRegSetThreadName("Log UART");
	log_uart_tp = chThdGetSelfX();

	for (;;) {
		chEvtWaitAny((eventmask_t) 1);

		if (strcmp(m_log_uart_rx_buffer, "READ_POS") == 0) {
			print_log_ext();
		}
	}
}

static void print_log_ext(void) {
	static mc_values val;
	static POS_STATE pos;
	static GPS_STATE gps;
	float accel[3];
	float mag[3];

	pos_get_mc_val(&val);
	pos_get_pos(&pos);
	pos_get_gps(&gps);
	pos_get_imu(accel, 0, mag);
	uint32_t time = ST2MS(chVTGetSystemTimeX());
	uint32_t ms_today = pos_get_ms_today();

	printf_blocking(m_log_ext_mode == LOG_EXT_ETHERNET,
			"%u,"     // timestamp (ms)
			"%u,"     // timestamp pos today (ms)
			"%.3f,"   // car x
			"%.3f,"   // car y
			"%.2f,"   // roll
			"%.2f,"   // pitch
			"%.2f,"   // yaw
			"%.2f,"   // roll rate
			"%.2f,"   // pitch rate
			"%.2f,"   // yaw rate
			"%.2f,"   // accel_x
			"%.2f,"   // accel_y
			"%.2f,"   // accel_z
			"%.2f,"   // mag_x
			"%.2f,"   // mag_y
			"%.2f,"   // mag_z
			"%.3f,"   // speed
			"%d,"     // tachometer
			"%u,"     // timestamp gps sample today (ms)
			"%.7f,"   // lat
			"%.7f,"   // lon
			"%.3f,"  // height
			"%.3f,"  // Travel distance
			"%.2f\r\n",  // Yaw IMU

			time,
			ms_today,
			(double)pos.px,
			(double)pos.py,
			(double)pos.roll,
			(double)pos.pitch,
			(double)pos.yaw,
			(double)pos.roll_rate,
			(double)pos.pitch_rate,
			(double)pos.yaw_rate,
			(double)accel[0],
			(double)accel[1],
			(double)accel[2],
			(double)mag[0],
			(double)mag[1],
			(double)mag[2],
			(double)pos.speed,
			val.tachometer,
			gps.ms,
			gps.lat,
			gps.lon,
			gps.height,
			(double)(val.tachometer * main_config.car.gear_ratio
			* (2.0 / main_config.car.motor_poles) * (1.0 / 6.0)
			* main_config.car.wheel_diam * M_PI),
			(double)pos.yaw_imu);
}

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

	if (m_log_ext_mode == LOG_EXT_UART_POLLED) {
		if (c == '\n' || c == '\r') {
			if (m_log_uart_rx_ptr > 0) {
				m_log_uart_rx_buffer[m_log_uart_rx_ptr] = '\0';
				m_log_uart_rx_ptr = 0;

				chSysLockFromISR();
				chEvtSignalI(log_uart_tp, (eventmask_t) 1);
				chSysUnlockFromISR();
			}
		} else {
			m_log_uart_rx_buffer[m_log_uart_rx_ptr++] = c;

			if (m_log_uart_rx_ptr >= (UART_LOG_RX_BUFFER_SIZE - 1)) {
				m_log_uart_rx_ptr = 0;
			}
		}
	}
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
	(void)uartp;
}

static void set_baudrate(uint32_t baud) {
	if (UART_DEV.usart == USART1) {
		UART_DEV.usart->BRR = STM32_PCLK2 / baud;
	} else {
		UART_DEV.usart->BRR = STM32_PCLK1 / baud;
	}
}

static void printf_blocking(bool ethernet, char* format, ...) {
	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[512];

	len = vsnprintf(print_buffer, 511, format, arg);
	va_end (arg);

	print_buffer[len] = '\0';

	if(len > 0) {
		if (ethernet) {
			commands_send_log_ethernet((unsigned char*)print_buffer, (len < 511) ? len : 511);
		} else {
			write_blocking((unsigned char*)print_buffer, (len < 511) ? len : 511);
		}
	}
}

static void write_blocking(unsigned char *data, unsigned int len) {
	// Wait for the previous transmission to finish.
	while (UART_DEV.txstate == UART_TX_ACTIVE) {
		chThdSleep(1);
	}

	// Copy this data to a new buffer in case the provided one is re-used
	// after this function returns.
	static uint8_t buffer[512];
	memcpy(buffer, data, len);

	uartStartSend(&UART_DEV, len, buffer);
}
