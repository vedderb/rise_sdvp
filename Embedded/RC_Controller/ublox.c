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

// Settings
#define HW_UART_DEV					UARTD6
#define HW_UART_TX_PORT				GPIOC
#define HW_UART_TX_PIN				6
#define HW_UART_RX_PORT				GPIOC
#define HW_UART_RX_PIN				7
#define BAUDRATE					115200
#define SERIAL_RX_BUFFER_SIZE		1024

// Threads
static THD_FUNCTION(process_thread, arg);
static THD_WORKING_AREA(process_thread_wa, 4096);
static thread_t *process_tp;

// Variables
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;

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

	chThdCreateStatic(process_thread_wa, sizeof(process_thread_wa), NORMALPRIO, process_thread, NULL);
}

void ublox_send(unsigned char *data, unsigned int len) {
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

static THD_FUNCTION(process_thread, arg) {
	(void)arg;

	chRegSetThreadName("ublox process");

	process_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		static uint8_t m_send_buffer[SERIAL_RX_BUFFER_SIZE];
		static uint8_t line[SERIAL_RX_BUFFER_SIZE];
		static int line_pos = 0;

		while (serial_rx_read_pos != serial_rx_write_pos) {
			line[line_pos] = serial_rx_buffer[serial_rx_read_pos];

			if (line[line_pos] == '\n') {
				int32_t send_index = 0;
				m_send_buffer[send_index++] = main_id;
				m_send_buffer[send_index++] = CMD_SEND_NMEA_RADIO;
				memcpy(m_send_buffer + send_index, line, line_pos);
				send_index += line_pos;
				line[line_pos] = '\0';
				commands_send_packet(m_send_buffer, send_index);
				pos_input_nmea((const char*)line);
				line_pos = 0;
			}

			line_pos++;
			serial_rx_read_pos++;

			if (line_pos == SERIAL_RX_BUFFER_SIZE) {
				line_pos = 0;
			}

			if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_read_pos = 0;
			}
		}
	}
}
