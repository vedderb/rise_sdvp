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
#include "ch.h"
#include "hal.h"
#include "packet.h"
#include "pos.h"
#include "buffer.h"
#include "terminal.h"
#include "bldc_interface.h"
#include "servo_simple.h"
#include "utils.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// Defines
#define FWD_TIME		5000

// Private variables
static uint8_t send_buffer[PACKET_MAX_PL_LEN];
static void(*send_func)(unsigned char *data, unsigned int len) = 0;
static virtual_timer_t vt;

// Private functions
static void stop_forward(void *p);

void commands_init(void) {
	send_func = 0;
	chVTObjectInit(&vt);
}

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

	CMD_PACKET packet_id;
	uint8_t id = 0;

	id = data[0];
	data++;
	len--;

	packet_id = data[0];
	data++;
	len--;

	if (id == main_config.id || id == ID_ALL) {
		switch (packet_id) {
		case CMD_GET_SENSORS: {
			POS_STATE pos;
			float accel[3];
			float gyro[3];
			float mag[3];

			pos_get_imu(accel, gyro, mag);
			pos_get_pos(&pos);
			int32_t send_index = 0;
			send_buffer[send_index++] = main_config.id;
			send_buffer[send_index++] = CMD_GET_SENSORS;
			buffer_append_float32(send_buffer, pos.roll, 1e6, &send_index);
			buffer_append_float32(send_buffer, pos.pitch, 1e6, &send_index);
			buffer_append_float32(send_buffer, pos.yaw, 1e6, &send_index);
			buffer_append_float32(send_buffer, accel[0], 1e6, &send_index);
			buffer_append_float32(send_buffer, accel[1], 1e6, &send_index);
			buffer_append_float32(send_buffer, accel[2], 1e6, &send_index);
			buffer_append_float32(send_buffer, gyro[0], 1e6, &send_index);
			buffer_append_float32(send_buffer, gyro[1], 1e6, &send_index);
			buffer_append_float32(send_buffer, gyro[2], 1e6, &send_index);
			buffer_append_float32(send_buffer, mag[0], 1e6, &send_index);
			buffer_append_float32(send_buffer, mag[1], 1e6, &send_index);
			buffer_append_float32(send_buffer, mag[2], 1e6, &send_index);
			buffer_append_float32(send_buffer, pos.q0, 1e8, &send_index);
			buffer_append_float32(send_buffer, pos.q1, 1e8, &send_index);
			buffer_append_float32(send_buffer, pos.q2, 1e8, &send_index);
			buffer_append_float32(send_buffer, pos.q3, 1e8, &send_index);
			buffer_append_float32(send_buffer, pos.px, 1e4, &send_index);
			buffer_append_float32(send_buffer, pos.py, 1e4, &send_index);
			buffer_append_float32(send_buffer, pos.speed, 1e6, &send_index);
			commands_send_packet(send_buffer, send_index);
		} break;

		case CMD_TERMINAL_CMD:
			data[len] = '\0';
			terminal_process_string((char*)data);
			break;

		case CMD_VESC_FWD:
			bldc_interface_set_forward_func(commands_forward_vesc_packet);
			bldc_interface_send_packet(data, len);
			chVTSet(&vt, MS2ST(FWD_TIME), stop_forward, NULL);
			break;

		case CMD_RC_CONTROL: {
			RC_MODE mode;
			float throttle, steering;
			int32_t ind = 0;
			mode = data[ind++];
			throttle = buffer_get_float32(data, 1e4, &ind);
			steering = buffer_get_float32(data, 1e6, &ind);

			switch (mode) {
				case RC_MODE_CURRENT:
					bldc_interface_set_current(throttle);
					break;

				case RC_MODE_DUTY:
					utils_truncate_number(&throttle, -1.0, 1.0);
					bldc_interface_set_duty_cycle(throttle);
					break;

				case RC_MODE_PID: // In m/s
					throttle = throttle / (main_config.gear_ratio
							* (2.0 / main_config.motor_poles) * (1.0 / 60.0)
							* main_config.wheel_diam * M_PI);
					bldc_interface_set_rpm(throttle);
					break;

				default:
					break;
			}

			utils_truncate_number(&steering, -1.0, 1.0);
			steering = utils_map(steering, -1.0, 1.0, main_config.steering_left, main_config.steering_right);
			servo_simple_set_output(steering);
		} break;

		case CMD_SET_POS: {
			float x, y, angle;
			int32_t ind = 0;
			x = buffer_get_float32(data, 1e4, &ind);
			y = buffer_get_float32(data, 1e4, &ind);
			angle = buffer_get_float32(data, 1e6, &ind);
			pos_set_xya(x, y, angle);
		} break;

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
	print_buffer[1] = CMD_PRINTF;
	len = vsnprintf(print_buffer + 2, 253, format, arg);
	va_end (arg);

	if(len > 0) {
		commands_send_packet((unsigned char*)print_buffer, (len<253) ? len + 2: 255);
	}
}

void commands_forward_vesc_packet(unsigned char *data, unsigned int len) {
	send_buffer[0] = main_config.id;
	send_buffer[1] = CMD_VESC_FWD;
	memcpy(send_buffer + 2, data, len);
	commands_send_packet((unsigned char*)send_buffer, len + 2);
}

static void stop_forward(void *p) {
	(void)p;
	bldc_interface_set_forward_func(0);
}
