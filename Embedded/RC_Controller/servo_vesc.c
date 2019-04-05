/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

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

#include "servo_vesc.h"
#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "bldc_interface.h"
#include "terminal.h"
#include "utils.h"
#include "comm_can.h"
#include "commands.h"
#include <math.h>

// Settings
#define SPI_SW_MISO_GPIO			GPIOC
#define SPI_SW_MISO_PIN				11
#define SPI_SW_MOSI_GPIO			GPIOC
#define SPI_SW_MOSI_PIN				12
#define SPI_SW_SCK_GPIO				GPIOC
#define SPI_SW_SCK_PIN				10
#define SPI_SW_CS_GPIO				GPIOD
#define SPI_SW_CS_PIN				2

// Private functions
static void terminal_state(int argc, const char **argv);
static float as5047_read(bool *ok);
static bool spi_check_parity(uint16_t x);
static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length);
static void spi_begin(void);
static void spi_end(void);
static void spi_delay(void);

// Private variables
static float m_pos_set = 0.5;
static float m_pos_now = 0.0;
static float m_pos_now_raw = 0.0;

// Threads
static THD_WORKING_AREA(servo_thread_wa, 1024);
static THD_FUNCTION(servo_thread, arg);

void servo_vesc_init(void) {
	palSetPadMode(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN, PAL_MODE_INPUT);
	palSetPadMode(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(SPI_SW_CS_GPIO, SPI_SW_CS_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

	// Set MOSI to 1
	palSetPadMode(SPI_SW_MOSI_GPIO, SPI_SW_MOSI_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPad(SPI_SW_MOSI_GPIO, SPI_SW_MOSI_PIN);

	chThdCreateStatic(servo_thread_wa, sizeof(servo_thread_wa), NORMALPRIO, servo_thread, NULL);

	terminal_register_command_callback(
			"servo_vesc_state",
			"Print the state of the VESC servo for 30 seconds",
			"",
			terminal_state);
}

void servo_vesc_set_pos(float pos) {
	utils_truncate_number(&pos, 0.0, 1.0);
	m_pos_set = pos;
}

float servo_vesc_get_pos(void) {
	return m_pos_now;
}

float servo_vesc_get_pos_set(void) {
	return m_pos_set;
}

static THD_FUNCTION(servo_thread, arg) {
	(void)arg;

	chRegSetThreadName("Servo VESC");

	// Control loop state
	float i_term = 0.0;
	float prev_error = 0.0;
	float dt_int = 0.0;
	float d_filter = 0.0;
	int not_ok_cnt = 0;

	for(;;) {
		// Map s1 to 0.0 and s2 to 1.0
		bool ok = false;
		m_pos_now_raw = as5047_read(&ok);
		float pos = m_pos_now_raw;
		pos -= SERVO_VESC_S1;

		// Allow some margin after limit without wrapping around
		// TODO: check if this is the correct way
		if (pos < -20) {
			utils_norm_angle_360(&pos);
		}

		float end = SERVO_VESC_S2 - SERVO_VESC_S1;
		utils_norm_angle_360(&end);

		m_pos_now = utils_map(pos, 0.0, end, 0.0, 1.0);

		// Run PID-controller on the output

		float error = m_pos_set - m_pos_now;

		float dt = 0.01;
		float p_term = error * SERVO_VESC_P_GAIN;
		i_term += error * (SERVO_VESC_I_GAIN * dt);

		// Average DT for the D term when the error does not change. This likely
		// happens at low speed when the position resolution is low and several
		// control iterations run without position updates.
		// TODO: Are there problems with this approach?
		float d_term = 0.0;
		dt_int += dt;
		if (error == prev_error) {
			d_term = 0.0;
		} else {
			d_term = (error - prev_error) * (SERVO_VESC_D_GAIN / dt_int);
			dt_int = 0.0;
		}

		// Filter D
		UTILS_LP_FAST(d_filter, d_term, SERVO_VESC_D_FILTER);
		d_term = d_filter;

		// I-term wind-up protection
		utils_truncate_number_abs(&p_term, 1.0);
		utils_truncate_number_abs(&i_term, 1.0 - fabsf(p_term));

		// Store previous error
		prev_error = error;

		// Calculate output
		float output = p_term + i_term + d_term;
		utils_truncate_number(&output, -1.0, 1.0);

		if (ok) {
			not_ok_cnt = 0;
		} else {
			not_ok_cnt++;
		}

		if (not_ok_cnt < 100) {
			comm_can_set_vesc_id(SERVO_VESC_ID);
			bldc_interface_set_duty_cycle(SERVO_VESC_INVERTED ? output : -output);
		} else {
			bldc_interface_set_current(0.0);
		}

		chThdSleepMilliseconds(10);
	}
}

static void terminal_state(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	for (int i = 0;i < 300;i++) {
		commands_printf("Raw: %.1f Processed: %.3f",
				(double)m_pos_now_raw, (double)m_pos_now);
		chThdSleepMilliseconds(100);
	}

	commands_printf(" ");
}

// AS5047
static float as5047_read(bool *ok) {
	uint16_t pos;
	static float last_enc_angle = 0.0;
	float res = last_enc_angle;

	spi_begin();
	spi_transfer(&pos, 0, 1);
	spi_end();

	if(spi_check_parity(pos) && pos != 0xffff) {  // all ones = disconnect
		pos &= 0x3FFF;
		last_enc_angle = ((float)pos * 360.0) / 16384.0;
		res = last_enc_angle;
		if (ok) {
			*ok = true;
		}
	} else {
		if (ok) {
			*ok = false;
		}
	}

	return res;
}

static bool spi_check_parity(uint16_t x) {
	x ^= x >> 8;
	x ^= x >> 4;
	x ^= x >> 2;
	x ^= x >> 1;
	return (~x) & 1;
}

static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length) {
	for (int i = 0;i < length;i++) {
		uint16_t send = out_buf ? out_buf[i] : 0xFFFF;
		uint16_t recieve = 0;

		for (int bit = 0;bit < 16;bit++) {
			//palWritePad(HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, send >> 15);
			send <<= 1;

			spi_delay();
			palSetPad(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN);
			spi_delay();

			int r1, r2, r3;
			r1 = palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);
			__NOP();
			r2 = palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);
			__NOP();
			r3 = palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);

			recieve <<= 1;
			if (utils_middle_of_3_int(r1, r2, r3)) {
				recieve |= 1;
			}

			palClearPad(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN);
			spi_delay();
		}

		if (in_buf) {
			in_buf[i] = recieve;
		}
	}
}

static void spi_begin(void) {
	palClearPad(SPI_SW_CS_GPIO, SPI_SW_CS_PIN);
}

static void spi_end(void) {
	palSetPad(SPI_SW_CS_GPIO, SPI_SW_CS_PIN);
}

static void spi_delay(void) {
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}
