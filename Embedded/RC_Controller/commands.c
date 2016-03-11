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

/*
 * commands.c
 *
 *  Created on: 11 mars 2016
 *      Author: benjamin
 */

#include "commands.h"
#include "packet.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// Private variables
static uint8_t send_buffer[PACKET_MAX_PL_LEN];
static void(*send_func)(unsigned char *data, unsigned int len) = 0;

/**
 * Provide a function to use the next time there are packets to be sent.
 *
 * @param func
 * A pointer to the packet sending function.
 */
void commands_set_send_func(void(*func)(unsigned char *data, unsigned int len)) {
	send_func = func;
}

/**
 * Send a packet using the set send function.
 *
 * @param data
 * The packet data.
 *
 * @param len
 * The data length.
 */
void commands_send_packet(unsigned char *data, unsigned int len) {
	if (send_func) {
		send_func(data, len);
	}
}

/**
 * Process a received buffer with commands and data.
 *
 * @param data
 * The buffer to process.
 *
 * @param len
 * The length of the buffer.
 */
void commands_process_packet(unsigned char *data, unsigned int len) {
	if (!len) {
		return;
	}

	COMM_PACKET_ID packet_id;
	int32_t ind = 0;
	uint8_t id = 0;

	id = data[0];
	data++;
	len--;

	packet_id = data[0];
	data++;
	len--;

	if (id == main_config.id || id == ID_ALL) {
		switch (packet_id) {
		case COMM_GET_ATTITUDE:
			// TODO!
			break;

		default:
			break;
		}
	}
}

void commands_printf(char* format, ...) {
	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[255];

	print_buffer[0] = main_config.id;
	print_buffer[1] = COMM_PRINTF;
	len = vsnprintf(print_buffer + 2, 253, format, arg);
	va_end (arg);

	if(len > 0) {
		commands_send_packet((unsigned char*)print_buffer, (len<253)? len+2: 255);
	}
}
