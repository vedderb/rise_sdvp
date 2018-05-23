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

#include "comm_uart.h"
#include "packet.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "buffer.h"
#include "deca_range.h"

#include <string.h>

// Settings
#define HW_UART_DEV					UARTD2
#define HW_UART_TX_PORT				GPIOA
#define HW_UART_TX_PIN				2
#define HW_UART_RX_PORT				GPIOA
#define HW_UART_RX_PIN				3

#define BAUDRATE					115200
#define PACKET_HANDLER				0
#define SERIAL_RX_BUFFER_SIZE		1024

// Threads
static THD_FUNCTION(packet_process_thread, arg);
static THD_WORKING_AREA(packet_process_thread_wa, 4096);
static thread_t *process_tp = 0;

// Variables
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;

// Private functions
static void process_packet(unsigned char *data, unsigned int len);
static void send_packet(unsigned char *data, unsigned int len);
static void range_func(float dist, uint8_t id);
static void data_func(uint8_t sender, uint8_t *buffer, int len);

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

static void process_packet(unsigned char *data, unsigned int len) {
	(void)len;

	deca_range_set_range_func(range_func);
	deca_range_set_data_func(data_func);

	switch (data[0]) {
	case CMD_DW_RANGE:
		deca_range_measure(data[1], data[2]);
		break;

	default:
		break;
	}
}

static void send_packet(unsigned char *data, unsigned int len) {
	// Wait for the previous transmission to finish.
	while (HW_UART_DEV.txstate == UART_TX_ACTIVE) {
		chThdSleep(1);
	}

	// Copy this data to a new buffer in case the provided one is re-used
	// after this function returns.
	static uint8_t buffer[PACKET_MAX_PL_LEN + 5];
	memcpy(buffer, data, len);

	uartStartSend(&HW_UART_DEV, len, buffer);
}

static void range_func(float dist, uint8_t id) {
	uint8_t buffer[8];
	int32_t ind = 0;

	buffer[ind++] = CMD_DW_RANGE;
	buffer[ind++] = id;
	buffer_append_int32(buffer, (int32_t)(dist * 1000.0), &ind);

	packet_send_packet(buffer, ind, PACKET_HANDLER);
}

static void data_func(uint8_t sender, uint8_t *buffer, int len) {
	(void)sender;
	(void)buffer;
	(void)len;
	// TODO!
}

void comm_uart_init(void) {
	packet_init(send_packet, process_packet, PACKET_HANDLER);
	serial_rx_read_pos = 0;
	serial_rx_write_pos = 0;

	uartStart(&HW_UART_DEV, &uart_cfg);

	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(GPIO_AF_USART2) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(GPIO_AF_USART2) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);

	chThdCreateStatic(packet_process_thread_wa, sizeof(packet_process_thread_wa),
			NORMALPRIO, packet_process_thread, NULL);
}

static THD_FUNCTION(packet_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("UART proc");

	process_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		while (serial_rx_read_pos != serial_rx_write_pos) {
			packet_process_byte(serial_rx_buffer[serial_rx_read_pos++], PACKET_HANDLER);

			if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_read_pos = 0;
			}
		}
	}
}
