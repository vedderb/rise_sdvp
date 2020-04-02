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

#include "as5047.h"

// Private functions
static float as5047_read(bool *ok);
static bool spi_check_parity(uint16_t x);
static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length);
static void spi_begin(void);
static void spi_end(void);
static void spi_delay(void);

// Threads
static THD_WORKING_AREA(read_thread_wa, 512);
static THD_FUNCTION(read_thread, arg);

// Private variables
static float m_angle_last = 0.0;

void as5047_init(void) {
	palSetPadMode(AS5047_MISO_GPIO, AS5047_MISO_PIN, PAL_MODE_INPUT);
	palSetPadMode(AS5047_SCK_GPIO, AS5047_SCK_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(AS5047_CS_GPIO, AS5047_CS_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

	// Set MOSI to 1
	palSetPadMode(AS5047_MOSI_GPIO, AS5047_MOSI_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPad(AS5047_MOSI_GPIO, AS5047_MOSI_PIN);

	chThdCreateStatic(read_thread_wa, sizeof(read_thread_wa), NORMALPRIO, read_thread, NULL);
}

float as5047_angle(void) {
	return m_angle_last;
}

static THD_FUNCTION(read_thread, arg) {
	(void)arg;

	chRegSetThreadName("AS5047");

	for(;;) {
		bool ok;
		float res = as5047_read(&ok);

		if (ok) {
			m_angle_last = res;
		}

		chThdSleepMilliseconds(1);
	}

}

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
		uint16_t receive = 0;

		for (int bit = 0;bit < 16;bit++) {
			//palWritePad(HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, send >> 15);
			send <<= 1;

			spi_delay();
			palSetPad(AS5047_SCK_GPIO, AS5047_SCK_PIN);
			spi_delay();

			int samples = 0;
			samples += palReadPad(AS5047_MISO_GPIO, AS5047_MISO_PIN);
			samples += palReadPad(AS5047_MISO_GPIO, AS5047_MISO_PIN);
			samples += palReadPad(AS5047_MISO_GPIO, AS5047_MISO_PIN);

			receive <<= 1;
			if (samples > 1) {
				receive |= 1;
			}

			palClearPad(AS5047_SCK_GPIO, AS5047_SCK_PIN);
			spi_delay();
		}

		if (in_buf) {
			in_buf[i] = receive;
		}
	}
}

static void spi_begin(void) {
	palClearPad(AS5047_CS_GPIO, AS5047_CS_PIN);
}

static void spi_end(void) {
	palSetPad(AS5047_CS_GPIO, AS5047_CS_PIN);
}

static void spi_delay(void) {
	chThdSleep(1);
}
