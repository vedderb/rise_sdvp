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
#include "autopilot.h"
#include "comm_cc2520.h"
#include "comm_usb.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// Defines
#define FWD_TIME		5000

// Private variables
static uint8_t m_send_buffer[PACKET_MAX_PL_LEN];
static void(*m_send_func)(unsigned char *data, unsigned int len) = 0;
static virtual_timer_t vt;

// Private functions
static void stop_forward(void *p);

void commands_init(void) {
	m_send_func = 0;
	chVTObjectInit(&vt);
}

/**
 * Provide a function to use the next time there are packets to be sent.
 *
 * @param func
 * A pointer to the packet sending function.
 */
void commands_set_send_func(void(*func)(unsigned char *data, unsigned int len)) {
	m_send_func = func;
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
	if (m_send_func) {
		m_send_func(data, len);
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
 *
 * @param func
 * A pointer to the packet sending function.
 */
void commands_process_packet(unsigned char *data, unsigned int len,
		void (*func)(unsigned char *data, unsigned int len)) {
	if (!len) {
		return;
	}

	CMD_PACKET packet_id;
	uint8_t id = 0;
	MAIN_CONFIG main_cfg_tmp;

	id = data[0];
	data++;
	len--;

	packet_id = data[0];
	data++;
	len--;

	if (id == main_id || id == ID_ALL) {
		switch (packet_id) {
		case CMD_GET_STATE: {
			POS_STATE pos;
			mc_values mcval;
			float accel[3];
			float gyro[3];
			float mag[3];
			
			m_send_func = func;

			pos_get_imu(accel, gyro, mag);
			pos_get_pos(&pos);
			pos_get_mc_val(&mcval);
			int32_t send_index = 0;
			m_send_buffer[send_index++] = main_id;
			m_send_buffer[send_index++] = CMD_GET_STATE;
			m_send_buffer[send_index++] = FW_VERSION_MAJOR;
			m_send_buffer[send_index++] = FW_VERSION_MINOR;
			buffer_append_float32(m_send_buffer, pos.roll, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, pos.pitch, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, pos.yaw, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, accel[0], 1e6, &send_index);
			buffer_append_float32(m_send_buffer, accel[1], 1e6, &send_index);
			buffer_append_float32(m_send_buffer, accel[2], 1e6, &send_index);
			buffer_append_float32(m_send_buffer, gyro[0], 1e6, &send_index);
			buffer_append_float32(m_send_buffer, gyro[1], 1e6, &send_index);
			buffer_append_float32(m_send_buffer, gyro[2], 1e6, &send_index);
			buffer_append_float32(m_send_buffer, mag[0], 1e6, &send_index);
			buffer_append_float32(m_send_buffer, mag[1], 1e6, &send_index);
			buffer_append_float32(m_send_buffer, mag[2], 1e6, &send_index);
			buffer_append_float32(m_send_buffer, pos.q0, 1e8, &send_index);
			buffer_append_float32(m_send_buffer, pos.q1, 1e8, &send_index);
			buffer_append_float32(m_send_buffer, pos.q2, 1e8, &send_index);
			buffer_append_float32(m_send_buffer, pos.q3, 1e8, &send_index);
			buffer_append_float32(m_send_buffer, pos.px, 1e4, &send_index);
			buffer_append_float32(m_send_buffer, pos.py, 1e4, &send_index);
			buffer_append_float32(m_send_buffer, pos.speed, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, mcval.v_in, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, mcval.temp_mos1, 1e6, &send_index);
			m_send_buffer[send_index++] = mcval.fault_code;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_TERMINAL_CMD:
			m_send_func = func;
			data[len] = '\0';
			terminal_process_string((char*)data);
			break;

		case CMD_VESC_FWD:
			m_send_func = func;
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
					autopilot_set_motor_speed(throttle);
					break;

				default:
					break;
			}

			utils_truncate_number(&steering, -1.0, 1.0);
			steering = utils_map(steering, -1.0, 1.0, main_config.steering_left, main_config.steering_right);
			servo_simple_set_pos_ramp(steering);
		} break;

		case CMD_SET_POS: {
			float x, y, angle;
			int32_t ind = 0;
			x = buffer_get_float32(data, 1e4, &ind);
			y = buffer_get_float32(data, 1e4, &ind);
			angle = buffer_get_float32(data, 1e6, &ind);
			pos_set_xya(x, y, angle);
		} break;

		case CMD_AP_ADD_POINTS: {
			m_send_func = func;
			int32_t ind = 0;

			while (ind < (int32_t)len) {
				ROUTE_POINT p;
				p.px = buffer_get_float32(data, 1e4, &ind);
				p.py = buffer_get_float32(data, 1e4, &ind);
				p.speed = buffer_get_float32(data, 1e6, &ind);
				autopilot_add_point(&p);
			}

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = main_id;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_AP_CLEAR_POINTS: {
			autopilot_clear_route();

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = main_id;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_AP_SET_ACTIVE: {
			autopilot_set_active(data[0]);

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = main_id;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_SET_SERVO_DIRECT: {
			int32_t ind = 0;
			float steering = buffer_get_float32(data, 1e6, &ind);
			utils_truncate_number(&steering, 0.0, 1.0);
			servo_simple_set_pos_ramp(steering);
		} break;

		case CMD_SEND_RTCM_USB: {
			int32_t send_index = 0;
			m_send_buffer[send_index++] = main_id;
			m_send_buffer[send_index++] = packet_id;
			memcpy(m_send_buffer + send_index, data, len);
			send_index += len;
			comm_usb_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_SEND_NMEA_RADIO: {
			int32_t send_index = 0;
			m_send_buffer[send_index++] = main_id;
			m_send_buffer[send_index++] = packet_id;
			memcpy(m_send_buffer + send_index, data, len);
			send_index += len;
			comm_cc2520_send_buffer(m_send_buffer, send_index);
		} break;

		case CMD_SET_MAIN_CONFIG: {
			int32_t ind = 0;
			main_config.mag_comp = data[ind++];
			main_config.yaw_imu_gain = buffer_get_float32(data, 1e6, &ind);

			main_config.mag_cal_cx = buffer_get_float32(data, 1e6, &ind);
			main_config.mag_cal_cy = buffer_get_float32(data, 1e6, &ind);
			main_config.mag_cal_cz = buffer_get_float32(data, 1e6, &ind);
			main_config.mag_cal_xx = buffer_get_float32(data, 1e6, &ind);
			main_config.mag_cal_xy = buffer_get_float32(data, 1e6, &ind);
			main_config.mag_cal_xz = buffer_get_float32(data, 1e6, &ind);
			main_config.mag_cal_yx = buffer_get_float32(data, 1e6, &ind);
			main_config.mag_cal_yy = buffer_get_float32(data, 1e6, &ind);
			main_config.mag_cal_yz = buffer_get_float32(data, 1e6, &ind);
			main_config.mag_cal_zx = buffer_get_float32(data, 1e6, &ind);
			main_config.mag_cal_zy = buffer_get_float32(data, 1e6, &ind);
			main_config.mag_cal_zz = buffer_get_float32(data, 1e6, &ind);

			main_config.gear_ratio = buffer_get_float32(data, 1e6, &ind);
			main_config.wheel_diam = buffer_get_float32(data, 1e6, &ind);
			main_config.motor_poles = buffer_get_float32(data, 1e6, &ind);
			main_config.steering_max_angle_rad = buffer_get_float32(data, 1e6, &ind);
			main_config.steering_center = buffer_get_float32(data, 1e6, &ind);
			main_config.steering_left = buffer_get_float32(data, 1e6, &ind);
			main_config.steering_right = buffer_get_float32(data, 1e6, &ind);
			main_config.steering_ramp_time = buffer_get_float32(data, 1e6, &ind);
			main_config.axis_distance = buffer_get_float32(data, 1e6, &ind);

			conf_general_store_main_config(&main_config);

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = main_id;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_GET_MAIN_CONFIG:
		case CMD_GET_MAIN_CONFIG_DEFAULT: {
			if (packet_id == CMD_GET_MAIN_CONFIG) {
				main_cfg_tmp = main_config;
			} else {
				conf_general_get_default_main_config(&main_cfg_tmp);
			}

			int32_t send_index = 0;
			m_send_buffer[send_index++] = main_id;
			m_send_buffer[send_index++] = packet_id;

			m_send_buffer[send_index++] = main_cfg_tmp.mag_comp;
			buffer_append_float32(m_send_buffer, main_cfg_tmp.yaw_imu_gain, 1e6, &send_index);

			buffer_append_float32(m_send_buffer, main_cfg_tmp.mag_cal_cx, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.mag_cal_cy, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.mag_cal_cz, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.mag_cal_xx, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.mag_cal_xy, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.mag_cal_xz, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.mag_cal_yx, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.mag_cal_yy, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.mag_cal_yz, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.mag_cal_zx, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.mag_cal_zy, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.mag_cal_zz, 1e6, &send_index);

			buffer_append_float32(m_send_buffer, main_cfg_tmp.gear_ratio, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.wheel_diam, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.motor_poles, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.steering_max_angle_rad, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.steering_center, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.steering_left, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.steering_right, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.steering_ramp_time, 1e6, &send_index);
			buffer_append_float32(m_send_buffer, main_cfg_tmp.axis_distance, 1e6, &send_index);

			commands_send_packet(m_send_buffer, send_index);
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

	print_buffer[0] = main_id;
	print_buffer[1] = CMD_PRINTF;
	len = vsnprintf(print_buffer + 2, 253, format, arg);
	va_end (arg);

	if(len > 0) {
		commands_send_packet((unsigned char*)print_buffer, (len<253) ? len + 2: 255);
	}
}

void commands_forward_vesc_packet(unsigned char *data, unsigned int len) {
	m_send_buffer[0] = main_id;
	m_send_buffer[1] = CMD_VESC_FWD;
	memcpy(m_send_buffer + 2, data, len);
	commands_send_packet((unsigned char*)m_send_buffer, len + 2);
}

static void stop_forward(void *p) {
	(void)p;
	bldc_interface_set_forward_func(0);
}
