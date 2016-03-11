/*
	Copyright 2015 Benjamin Vedder	benjamin@vedder.se

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

#include "spi_sw.h"
#include "ch.h"
#include "hal.h"
#include <stdbool.h>
#include "stm32_hw.h"

// Private variables
static bool init_done = false;

// Private functions
static void spi_sw_delay(void);

void spi_sw_init(void) {
	if (!init_done) {
		palSetPadMode(CC2520_SPI_MISO_PORT, CC2520_SPI_MISO_PIN, PAL_MODE_INPUT);
		palSetPadMode(CC2520_SPI_CSN_PORT, CC2520_SPI_CSN_PIN, PAL_MODE_OUTPUT_PUSHPULL);
		palSetPadMode(CC2520_SPI_SCK_PORT, CC2520_SPI_SCK_PIN, PAL_MODE_OUTPUT_PUSHPULL);
		palSetPadMode(CC2520_SPI_MOSI_PORT, CC2520_SPI_MOSI_PIN, PAL_MODE_OUTPUT_PUSHPULL);

		palSetPad(CC2520_SPI_CSN_PORT, CC2520_SPI_CSN_PIN);
		palClearPad(CC2520_SPI_SCK_PORT, CC2520_SPI_SCK_PIN);
		init_done = true;
	}
}

void spi_sw_transfer(char *in_buf, const char *out_buf, int length) {
	palClearPad(CC2520_SPI_SCK_PORT, CC2520_SPI_SCK_PIN);
	spi_sw_delay();

	for (int i = 0;i < length;i++) {
		unsigned char send = out_buf ? out_buf[i] : 0;
		unsigned char recieve = 0;

		for (int bit=0;bit < 8;bit++) {
			palWritePad(CC2520_SPI_MOSI_PORT, CC2520_SPI_MOSI_PIN, send >> 7);
			send <<= 1;

			spi_sw_delay();

			recieve <<= 1;
			if (palReadPad(CC2520_SPI_MISO_PORT, CC2520_SPI_MISO_PIN)) {
				recieve |= 0x1;
			}

			palSetPad(CC2520_SPI_SCK_PORT, CC2520_SPI_SCK_PIN);
			spi_sw_delay();
			palClearPad(CC2520_SPI_SCK_PORT, CC2520_SPI_SCK_PIN);
		}

		if (in_buf) {
			in_buf[i] = recieve;
		}
	}
}

void spi_sw_begin(void) {
	palClearPad(CC2520_SPI_CSN_PORT, CC2520_SPI_CSN_PIN);
	spi_sw_delay();
}

void spi_sw_end(void) {
	spi_sw_delay();
	palSetPad(CC2520_SPI_CSN_PORT, CC2520_SPI_CSN_PIN);
}

static void spi_sw_delay(void) {
//	for (volatile int i = 0;i < 5;i++) {
//		__NOP();
//	}
	__NOP();
}
