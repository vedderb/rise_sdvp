/*
	Copyright 2016 - 2018 Benjamin Vedder	benjamin@vedder.se

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
#include "timeout.h"
#include "log.h"
#include "radar.h"
#include "ublox.h"
#include "comm_cc1120.h"
#include "mr_control.h"
#include "adconv.h"
#include "motor_sim.h"
#include "m8t_base.h"
#include "pos_uwb.h"
#include "fi.h"
#include "comm_can.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// Defines
#define FWD_TIME		20000

// Private variables
static uint8_t m_send_buffer[PACKET_MAX_PL_LEN];
static void(*m_send_func)(unsigned char *data, unsigned int len) = 0;
static virtual_timer_t vt;
static mutex_t m_print_gps;
static bool m_init_done = false;

// Private functions
static void stop_forward(void *p);
static void rtcm_rx(uint8_t *data, int len, int type);
static void rtcm_base_rx(rtcm_ref_sta_pos_t *pos);

// Private variables
static rtcm3_state m_rtcm_state;

void commands_init(void) {
	m_send_func = 0;
	chMtxObjectInit(&m_print_gps);
	chVTObjectInit(&vt);

	rtcm3_init_state(&m_rtcm_state);
	rtcm3_set_rx_callback(rtcm_rx, &m_rtcm_state);
	rtcm3_set_rx_callback_1005_1006(rtcm_base_rx, &m_rtcm_state);

#if MAIN_MODE != MAIN_MODE_CAR
	(void)stop_forward;
#endif

	m_init_done = true;
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

	if (data[0] == RTCM3PREAMB) {
		for (unsigned int i = 0;i < len;i++) {
			rtcm3_input_data(data[i], &m_rtcm_state);
		}
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

	if (id == main_id || id == ID_ALL || id == ID_CAR_CLIENT) {
		int id_ret = main_id;

		if (id == ID_CAR_CLIENT) {
			id_ret = ID_CAR_CLIENT;
		}

		switch (packet_id) {
		// ==================== General commands ==================== //
		case CMD_TERMINAL_CMD: {
			timeout_reset();
			commands_set_send_func(func);

			data[len] = '\0';
			terminal_process_string((char*)data);
		} break;

		// ==================== Vehicle commands ==================== //
#if MAIN_MODE_IS_VEHICLE
		case CMD_SET_POS:
		case CMD_SET_POS_ACK: {
			timeout_reset();

			float x, y, angle;
			int32_t ind = 0;
			x = buffer_get_float32(data, 1e4, &ind);
			y = buffer_get_float32(data, 1e4, &ind);
			angle = buffer_get_float32(data, 1e6, &ind);
			pos_set_xya(x, y, angle);
			pos_uwb_set_xya(x, y, angle);

			if (packet_id == CMD_SET_POS_ACK) {
				commands_set_send_func(func);
				// Send ack
				int32_t send_index = 0;
				m_send_buffer[send_index++] = id_ret;
				m_send_buffer[send_index++] = packet_id;
				commands_send_packet(m_send_buffer, send_index);
			}
		} break;

		case CMD_SET_ENU_REF: {
			timeout_reset();
			commands_set_send_func(func);

			int32_t ind = 0;
			double lat, lon, height;
			lat = buffer_get_double64(data, D(1e16), &ind);
			lon = buffer_get_double64(data, D(1e16), &ind);
			height = buffer_get_float32(data, 1e3, &ind);
			pos_set_enu_ref(lat, lon, height);

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_GET_ENU_REF: {
			timeout_reset();
			commands_set_send_func(func);

			double llh[3];
			pos_get_enu_ref(llh);

			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = CMD_GET_ENU_REF;
			buffer_append_double64(m_send_buffer, llh[0], D(1e16), &send_index);
			buffer_append_double64(m_send_buffer, llh[1], D(1e16), &send_index);
			buffer_append_float32(m_send_buffer, llh[2], 1e3, &send_index);
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_AP_ADD_POINTS: {
			timeout_reset();
			commands_set_send_func(func);

			int32_t ind = 0;
			bool first = true;

			while (ind < (int32_t)len) {
				ROUTE_POINT p;
				p.px = buffer_get_float32(data, 1e4, &ind);
				p.py = buffer_get_float32(data, 1e4, &ind);
				p.speed = buffer_get_float32(data, 1e6, &ind);
				p.time = buffer_get_int32(data, &ind);
				bool res = autopilot_add_point(&p, first);
				first = false;

				if (!res) {
					break;
				}
			}

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_AP_REMOVE_LAST_POINT: {
			timeout_reset();
			commands_set_send_func(func);

			autopilot_remove_last_point();

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_AP_CLEAR_POINTS: {
			timeout_reset();
			commands_set_send_func(func);

			autopilot_clear_route();

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_AP_GET_ROUTE_PART: {
			int32_t ind = 0;
			int first = buffer_get_int32(data, &ind);
			int num = data[ind++];

			if (num > 20) {
				break;
			}

			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = CMD_AP_GET_ROUTE_PART;

			int route_len = autopilot_get_route_len();
			buffer_append_int32(m_send_buffer, route_len, &send_index);

			for (int i = first;i < (first + num);i++) {
				ROUTE_POINT rp = autopilot_get_route_point(i);
				buffer_append_float32_auto(m_send_buffer, rp.px, &send_index);
				buffer_append_float32_auto(m_send_buffer, rp.py, &send_index);
				buffer_append_float32_auto(m_send_buffer, rp.speed, &send_index);
				buffer_append_int32(m_send_buffer, rp.time, &send_index);
			}

			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_AP_SET_ACTIVE: {
			timeout_reset();
			commands_set_send_func(func);

			autopilot_set_active(data[0]);

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_AP_REPLACE_ROUTE: {
			timeout_reset();
			commands_set_send_func(func);

			int32_t ind = 0;
			int first = true;

			while (ind < (int32_t)len) {
				ROUTE_POINT p;
				p.px = buffer_get_float32(data, 1e4, &ind);
				p.py = buffer_get_float32(data, 1e4, &ind);
				p.speed = buffer_get_float32(data, 1e6, &ind);
				p.time = buffer_get_int32(data, &ind);

				if (first) {
					first = !autopilot_replace_route(&p);
				} else {
					autopilot_add_point(&p, false);
				}
			}

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_AP_SYNC_POINT: {
			timeout_reset();
			commands_set_send_func(func);

			int32_t ind = 0;
			int32_t point = buffer_get_int32(data, &ind);
			int32_t time = buffer_get_int32(data, &ind);
			int32_t min_diff = buffer_get_int32(data, &ind);

			autopilot_sync_point(point, time, min_diff);

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_SEND_RTCM_USB: {
			for (unsigned int i = 0;i < len;i++) {
				rtcm3_input_data(data[i], &m_rtcm_state);
			}
		} break;

		case CMD_SEND_NMEA_RADIO: {
#if !UBLOX_EN
			// Only enable this command if the board is configured without the ublox
			char *curLine = (char*)data;
			while(curLine) {
				char *nextLine = strchr(curLine, '\n');
				if (nextLine) {
					*nextLine = '\0';
				}

				bool found = pos_input_nmea(curLine);

				// Only send the lines that pos decoded
				if (found && main_config.gps_send_nmea) {
					int32_t send_index = 0;
					m_send_buffer[send_index++] = id_ret;
					m_send_buffer[send_index++] = packet_id;
					int len_line = strlen(curLine);
					memcpy(m_send_buffer + send_index, curLine, len_line);
					send_index += len_line;

					commands_send_packet(m_send_buffer, send_index);
				}

				if (nextLine) {
					*nextLine = '\n';
				}

				curLine = nextLine ? (nextLine + 1) : NULL;
			}
#endif
		} break;

		case CMD_SET_YAW_OFFSET:
		case CMD_SET_YAW_OFFSET_ACK: {
			timeout_reset();

			float angle;
			int32_t ind = 0;
			angle = buffer_get_float32(data, 1e6, &ind);
			pos_set_yaw_offset(angle);

			if (packet_id == CMD_SET_YAW_OFFSET_ACK) {
				commands_set_send_func(func);
				// Send ack
				int32_t send_index = 0;
				m_send_buffer[send_index++] = id_ret;
				m_send_buffer[send_index++] = packet_id;
				commands_send_packet(m_send_buffer, send_index);
			}
		} break;

		case CMD_RADAR_SETUP_SET: {
#if RADAR_EN
			timeout_reset();
			commands_set_send_func(func);

			radar_settings_t s;
			int32_t ind = 0;

			s.f_center = buffer_get_float32_auto(data, &ind);
			s.f_span = buffer_get_float32_auto(data, &ind);
			s.points = buffer_get_int16(data, &ind);
			s.t_sweep = buffer_get_float32_auto(data, &ind);
			s.cc_x = buffer_get_float32_auto(data, &ind);
			s.cc_y = buffer_get_float32_auto(data, &ind);
			s.cc_rad = buffer_get_float32_auto(data, &ind);
			s.log_rate_ms = buffer_get_int32(data, &ind);
			s.log_en = data[ind++];
			s.map_plot_avg_factor = buffer_get_float32_auto(data, &ind);
			s.map_plot_max_div = buffer_get_float32_auto(data, &ind);
			s.plot_mode = data[ind++];
			s.map_plot_start = buffer_get_uint16(data, &ind);
			s.map_plot_end = buffer_get_uint16(data, &ind);

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);

			timeout_reset();

			radar_setup_measurement(&s);
#endif
		} break;

		case CMD_RADAR_SETUP_GET: {
#if RADAR_EN
			timeout_reset();
			commands_set_send_func(func);

			const radar_settings_t *s = radar_get_settings();
			int32_t send_index = 0;

			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			buffer_append_float32_auto(m_send_buffer, s->f_center, &send_index);
			buffer_append_float32_auto(m_send_buffer, s->f_span, &send_index);
			buffer_append_int16(m_send_buffer, s->points, &send_index);
			buffer_append_float32_auto(m_send_buffer, s->t_sweep, &send_index);
			buffer_append_float32_auto(m_send_buffer, s->cc_x, &send_index);
			buffer_append_float32_auto(m_send_buffer, s->cc_y, &send_index);
			buffer_append_float32_auto(m_send_buffer, s->cc_rad, &send_index);
			buffer_append_int32(m_send_buffer, s->log_rate_ms, &send_index);
			m_send_buffer[send_index++] = s->log_en;
			buffer_append_float32_auto(m_send_buffer, s->map_plot_avg_factor, &send_index);
			buffer_append_float32_auto(m_send_buffer, s->map_plot_max_div, &send_index);
			m_send_buffer[send_index++] = s->plot_mode;
			buffer_append_uint16(m_send_buffer, s->map_plot_start, &send_index);
			buffer_append_uint16(m_send_buffer, s->map_plot_end, &send_index);

			commands_send_packet(m_send_buffer, send_index);
#endif
		} break;

		case CMD_SET_MS_TODAY: {
			timeout_reset();

			int32_t time;
			int32_t ind = 0;
			time = buffer_get_int32(data, &ind);
			pos_set_ms_today(time);
		} break;

		case CMD_SET_SYSTEM_TIME: {
			commands_set_send_func(func);

			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			memcpy(m_send_buffer + send_index, data, len);
			send_index += len;
			comm_usb_send_packet(m_send_buffer, send_index);

			// Send ack
			send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = CMD_SET_SYSTEM_TIME_ACK;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_REBOOT_SYSTEM: {
			commands_set_send_func(func);

			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			memcpy(m_send_buffer + send_index, data, len);
			send_index += len;
			comm_usb_send_packet(m_send_buffer, send_index);

			// Send ack
			send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = CMD_REBOOT_SYSTEM_ACK;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_SET_MAIN_CONFIG: {
			timeout_reset();
			commands_set_send_func(func);

			int32_t ind = 0;
			main_config.mag_use = data[ind++];
			main_config.mag_comp = data[ind++];
			main_config.yaw_mag_gain = buffer_get_float32_auto(data, &ind);

			main_config.mag_cal_cx = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_cy = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_cz = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_xx = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_xy = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_xz = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_yx = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_yy = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_yz = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_zx = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_zy = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_zz = buffer_get_float32_auto(data, &ind);

			main_config.gps_ant_x = buffer_get_float32_auto(data, &ind);
			main_config.gps_ant_y = buffer_get_float32_auto(data, &ind);
			main_config.gps_comp = data[ind++];
			main_config.gps_req_rtk = data[ind++];
			main_config.gps_use_rtcm_base_as_enu_ref = data[ind++];
			main_config.gps_corr_gain_stat = buffer_get_float32_auto(data, &ind);
			main_config.gps_corr_gain_dyn = buffer_get_float32_auto(data, &ind);
			main_config.gps_corr_gain_yaw = buffer_get_float32_auto(data, &ind);
			main_config.gps_send_nmea = data[ind++];
			main_config.gps_use_ubx_info = data[ind++];
			main_config.gps_ubx_max_acc = buffer_get_float32_auto(data, &ind);

			main_config.ap_repeat_routes = data[ind++];
			main_config.ap_base_rad = buffer_get_float32_auto(data, &ind);
			main_config.ap_mode_time = data[ind++];
			main_config.ap_max_speed = buffer_get_float32_auto(data, &ind);
			main_config.ap_time_add_repeat_ms = buffer_get_int32(data, &ind);

			main_config.log_rate_hz = buffer_get_int16(data, &ind);
			main_config.log_en = data[ind++];
			strcpy(main_config.log_name, (const char*)(data + ind));
			ind += strlen(main_config.log_name) + 1;
			main_config.log_mode_ext = data[ind++];
			main_config.log_uart_baud = buffer_get_uint32(data, &ind);

			log_set_rate(main_config.log_rate_hz);
			log_set_enabled(main_config.log_en);
			log_set_name(main_config.log_name);
			log_set_ext(main_config.log_mode_ext, main_config.log_uart_baud);

			// Car settings
			main_config.car.yaw_use_odometry = data[ind++];
			main_config.car.yaw_imu_gain = buffer_get_float32_auto(data, &ind);
			main_config.car.disable_motor = data[ind++];
			main_config.car.simulate_motor = data[ind++];
			main_config.car.clamp_imu_yaw_stationary = data[ind++];

			main_config.car.gear_ratio = buffer_get_float32_auto(data, &ind);
			main_config.car.wheel_diam = buffer_get_float32_auto(data, &ind);
			main_config.car.motor_poles = buffer_get_float32_auto(data, &ind);
			main_config.car.steering_max_angle_rad = buffer_get_float32_auto(data, &ind);
			main_config.car.steering_center = buffer_get_float32_auto(data, &ind);
			main_config.car.steering_range = buffer_get_float32_auto(data, &ind);
			main_config.car.steering_ramp_time = buffer_get_float32_auto(data, &ind);
			main_config.car.axis_distance = buffer_get_float32_auto(data, &ind);

#if MAIN_MODE == MAIN_MODE_CAR
			motor_sim_set_running(main_config.car.simulate_motor);
#endif

			// Multirotor settings
			main_config.mr.vel_decay_e = buffer_get_float32_auto(data, &ind);
			main_config.mr.vel_decay_l = buffer_get_float32_auto(data, &ind);
			main_config.mr.vel_max = buffer_get_float32_auto(data, &ind);
			main_config.mr.map_min_x = buffer_get_float32_auto(data, &ind);
			main_config.mr.map_max_x = buffer_get_float32_auto(data, &ind);
			main_config.mr.map_min_y = buffer_get_float32_auto(data, &ind);
			main_config.mr.map_max_y = buffer_get_float32_auto(data, &ind);

			main_config.mr.vel_gain_p = buffer_get_float32_auto(data, &ind);
			main_config.mr.vel_gain_i = buffer_get_float32_auto(data, &ind);
			main_config.mr.vel_gain_d = buffer_get_float32_auto(data, &ind);

			main_config.mr.tilt_gain_p = buffer_get_float32_auto(data, &ind);
			main_config.mr.tilt_gain_i = buffer_get_float32_auto(data, &ind);
			main_config.mr.tilt_gain_d = buffer_get_float32_auto(data, &ind);

			main_config.mr.max_corr_error = buffer_get_float32_auto(data, &ind);
			main_config.mr.max_tilt_error = buffer_get_float32_auto(data, &ind);

			main_config.mr.ctrl_gain_roll_p = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_roll_i = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_roll_dp = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_roll_de = buffer_get_float32_auto(data, &ind);

			main_config.mr.ctrl_gain_pitch_p = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_pitch_i = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_pitch_dp = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_pitch_de = buffer_get_float32_auto(data, &ind);

			main_config.mr.ctrl_gain_yaw_p = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_yaw_i = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_yaw_dp = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_yaw_de = buffer_get_float32_auto(data, &ind);

			main_config.mr.ctrl_gain_pos_p = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_pos_i = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_pos_d = buffer_get_float32_auto(data, &ind);

			main_config.mr.ctrl_gain_alt_p = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_alt_i = buffer_get_float32_auto(data, &ind);
			main_config.mr.ctrl_gain_alt_d = buffer_get_float32_auto(data, &ind);

			main_config.mr.js_gain_tilt = buffer_get_float32_auto(data, &ind);
			main_config.mr.js_gain_yaw = buffer_get_float32_auto(data, &ind);
			main_config.mr.js_mode_rate = data[ind++];

			main_config.mr.motor_fl_f = data[ind++];
			main_config.mr.motor_bl_l = data[ind++];
			main_config.mr.motor_fr_r = data[ind++];
			main_config.mr.motor_br_b = data[ind++];
			main_config.mr.motors_x = data[ind++];
			main_config.mr.motors_cw = data[ind++];
			main_config.mr.motor_pwm_min_us = buffer_get_uint16(data, &ind);
			main_config.mr.motor_pwm_max_us = buffer_get_uint16(data, &ind);

			conf_general_store_main_config(&main_config);

			// Doing this while driving will get wrong as there is so much accelerometer noise then.
			//pos_reset_attitude();

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_GET_MAIN_CONFIG:
		case CMD_GET_MAIN_CONFIG_DEFAULT: {
			timeout_reset();
			commands_set_send_func(func);

			MAIN_CONFIG main_cfg_tmp;

			if (packet_id == CMD_GET_MAIN_CONFIG) {
				main_cfg_tmp = main_config;
			} else {
				conf_general_get_default_main_config(&main_cfg_tmp);
			}

			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;

			m_send_buffer[send_index++] = main_cfg_tmp.mag_use;
			m_send_buffer[send_index++] = main_cfg_tmp.mag_comp;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.yaw_mag_gain, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_cx, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_cy, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_cz, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_xx, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_xy, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_xz, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_yx, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_yy, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_yz, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_zx, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_zy, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_zz, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_ant_x, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_ant_y, &send_index);
			m_send_buffer[send_index++] = main_cfg_tmp.gps_comp;
			m_send_buffer[send_index++] = main_cfg_tmp.gps_req_rtk;
			m_send_buffer[send_index++] = main_cfg_tmp.gps_use_rtcm_base_as_enu_ref;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_corr_gain_stat, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_corr_gain_dyn, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_corr_gain_yaw, &send_index);
			m_send_buffer[send_index++] = main_cfg_tmp.gps_send_nmea;
			m_send_buffer[send_index++] = main_cfg_tmp.gps_use_ubx_info;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_ubx_max_acc, &send_index);

			m_send_buffer[send_index++] = main_cfg_tmp.ap_repeat_routes;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.ap_base_rad, &send_index);
			m_send_buffer[send_index++] = main_cfg_tmp.ap_mode_time;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.ap_max_speed, &send_index);
			buffer_append_int32(m_send_buffer, main_cfg_tmp.ap_time_add_repeat_ms, &send_index);

			buffer_append_int16(m_send_buffer, main_cfg_tmp.log_rate_hz, &send_index);
			m_send_buffer[send_index++] = main_cfg_tmp.log_en;
			strcpy((char*)(m_send_buffer + send_index), main_cfg_tmp.log_name);
			send_index += strlen(main_config.log_name) + 1;
			m_send_buffer[send_index++] = main_cfg_tmp.log_mode_ext;
			buffer_append_uint32(m_send_buffer, main_cfg_tmp.log_uart_baud, &send_index);

			// Car settings
			m_send_buffer[send_index++] = main_cfg_tmp.car.yaw_use_odometry;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.yaw_imu_gain, &send_index);
			m_send_buffer[send_index++] = main_cfg_tmp.car.disable_motor;
			m_send_buffer[send_index++] = main_cfg_tmp.car.simulate_motor;
			m_send_buffer[send_index++] = main_cfg_tmp.car.clamp_imu_yaw_stationary;

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.gear_ratio, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.wheel_diam, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.motor_poles, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.steering_max_angle_rad, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.steering_center, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.steering_range, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.steering_ramp_time, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.car.axis_distance, &send_index);

			// Multirotor settings
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.vel_decay_e, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.vel_decay_l, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.vel_max, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.map_min_x, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.map_max_x, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.map_min_y, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.map_max_y, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.vel_gain_p, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.vel_gain_i, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.vel_gain_d, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.tilt_gain_p, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.tilt_gain_i, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.tilt_gain_d, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.max_corr_error, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.max_tilt_error, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_roll_p, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_roll_i, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_roll_dp, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_roll_de, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_pitch_p, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_pitch_i, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_pitch_dp, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_pitch_de, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_yaw_p, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_yaw_i, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_yaw_dp, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_yaw_de, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_pos_p, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_pos_i, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_pos_d, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_alt_p, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_alt_i, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.ctrl_gain_alt_d, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.js_gain_tilt, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mr.js_gain_yaw, &send_index);
			m_send_buffer[send_index++] = main_cfg_tmp.mr.js_mode_rate;

			m_send_buffer[send_index++] = main_cfg_tmp.mr.motor_fl_f;
			m_send_buffer[send_index++] = main_cfg_tmp.mr.motor_bl_l;
			m_send_buffer[send_index++] = main_cfg_tmp.mr.motor_fr_r;
			m_send_buffer[send_index++] = main_cfg_tmp.mr.motor_br_b;
			m_send_buffer[send_index++] = main_cfg_tmp.mr.motors_x;
			m_send_buffer[send_index++] = main_cfg_tmp.mr.motors_cw;
			buffer_append_uint16(m_send_buffer, main_cfg_tmp.mr.motor_pwm_min_us, &send_index);
			buffer_append_uint16(m_send_buffer, main_cfg_tmp.mr.motor_pwm_max_us, &send_index);

			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_ADD_UWB_ANCHOR: {
			int32_t ind = 0;
			UWB_ANCHOR a;

			a.id = buffer_get_int16(data, &ind);
			a.px = buffer_get_float32_auto(data, &ind);
			a.py = buffer_get_float32_auto(data, &ind);
			a.height = buffer_get_float32_auto(data, &ind);
			a.dist_last = 0.0;
			pos_uwb_add_anchor(a);

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_CLEAR_UWB_ANCHORS: {
			pos_uwb_clear_anchors();

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		// ==================== Car commands ==================== //
#if MAIN_MODE == MAIN_MODE_CAR
		case CMD_GET_STATE: {
			timeout_reset();

			POS_STATE pos, pos_uwb;
			mc_values mcval;
			float accel[3];
			float gyro[3];
			float mag[3];
			ROUTE_POINT rp_goal;

			commands_set_send_func(func);

			pos_get_imu(accel, gyro, mag);
			pos_get_pos(&pos);
			pos_get_mc_val(&mcval);
			autopilot_get_goal_now(&rp_goal);
			pos_uwb_get_pos(&pos_uwb);

			fi_inject_fault_float("px", &pos.px);

			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret; // 1
			m_send_buffer[send_index++] = CMD_GET_STATE; // 2
			m_send_buffer[send_index++] = FW_VERSION_MAJOR; // 3
			m_send_buffer[send_index++] = FW_VERSION_MINOR; // 4
			buffer_append_float32(m_send_buffer, pos.roll, 1e6, &send_index); // 8
			buffer_append_float32(m_send_buffer, pos.pitch, 1e6, &send_index); // 12
			buffer_append_float32(m_send_buffer, pos.yaw, 1e6, &send_index); // 16
			buffer_append_float32(m_send_buffer, accel[0], 1e6, &send_index); // 20
			buffer_append_float32(m_send_buffer, accel[1], 1e6, &send_index); // 24
			buffer_append_float32(m_send_buffer, accel[2], 1e6, &send_index); // 28
			buffer_append_float32(m_send_buffer, gyro[0], 1e6, &send_index); // 32
			buffer_append_float32(m_send_buffer, gyro[1], 1e6, &send_index); // 36
			buffer_append_float32(m_send_buffer, gyro[2], 1e6, &send_index); // 40
			buffer_append_float32(m_send_buffer, mag[0], 1e6, &send_index); // 44
			buffer_append_float32(m_send_buffer, mag[1], 1e6, &send_index); // 48
			buffer_append_float32(m_send_buffer, mag[2], 1e6, &send_index); // 52
			buffer_append_float32(m_send_buffer, pos.px, 1e4, &send_index); // 56
			buffer_append_float32(m_send_buffer, pos.py, 1e4, &send_index); // 60
			buffer_append_float32(m_send_buffer, pos.speed, 1e6, &send_index); // 64
			buffer_append_float32(m_send_buffer, mcval.v_in, 1e6, &send_index); // 68
			buffer_append_float32(m_send_buffer, mcval.temp_mos, 1e6, &send_index); // 72
			m_send_buffer[send_index++] = mcval.fault_code; // 73
			buffer_append_float32(m_send_buffer, pos.px_gps, 1e4, &send_index); // 77
			buffer_append_float32(m_send_buffer, pos.py_gps, 1e4, &send_index); // 81
			buffer_append_float32(m_send_buffer, rp_goal.px, 1e4, &send_index); // 85
			buffer_append_float32(m_send_buffer, rp_goal.py, 1e4, &send_index); // 89
			buffer_append_float32(m_send_buffer, autopilot_get_rad_now(), 1e6, &send_index); // 93
			buffer_append_int32(m_send_buffer, pos_get_ms_today(), &send_index); // 97
			buffer_append_int16(m_send_buffer, autopilot_get_route_left(), &send_index); // 99
			buffer_append_float32(m_send_buffer, pos_uwb.px, 1e4, &send_index); // 103
			buffer_append_float32(m_send_buffer, pos_uwb.py, 1e4, &send_index); // 107
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_VESC_FWD:
			timeout_reset();
			commands_set_send_func(func);

			bldc_interface_set_forward_func(commands_forward_vesc_packet);
			bldc_interface_send_packet(data, len);
			chVTSet(&vt, MS2ST(FWD_TIME), stop_forward, NULL);
			break;

		case CMD_RC_CONTROL: {
			timeout_reset();

			RC_MODE mode;
			float throttle, steering;
			int32_t ind = 0;
			mode = data[ind++];
			throttle = buffer_get_float32(data, 1e4, &ind);
			steering = buffer_get_float32(data, 1e6, &ind);

			utils_truncate_number(&steering, -1.0, 1.0);
			steering *= autopilot_get_steering_scale();

			autopilot_set_active(false);

			switch (mode) {
			case RC_MODE_CURRENT:
				if (!main_config.car.disable_motor) {
#if HAS_DIFF_STEERING
					comm_can_lock_vesc();
					comm_can_set_vesc_id(DIFF_STEERING_VESC_LEFT);
					bldc_interface_set_current(throttle + throttle * steering);
					comm_can_set_vesc_id(DIFF_STEERING_VESC_RIGHT);
					bldc_interface_set_current(throttle - throttle * steering);
					comm_can_unlock_vesc();
#else
					bldc_interface_set_current(throttle);
#endif
				}
				break;

			case RC_MODE_DUTY:
				utils_truncate_number(&throttle, -1.0, 1.0);
				if (!main_config.car.disable_motor) {
#if HAS_DIFF_STEERING
					comm_can_lock_vesc();
					comm_can_set_vesc_id(DIFF_STEERING_VESC_LEFT);
					bldc_interface_set_duty_cycle(throttle + throttle * steering);
					comm_can_set_vesc_id(DIFF_STEERING_VESC_RIGHT);
					bldc_interface_set_duty_cycle(throttle - throttle * steering);
					comm_can_unlock_vesc();
#else
					bldc_interface_set_duty_cycle(throttle);
#endif
				}
				break;

			case RC_MODE_PID: // In m/s
#if HAS_DIFF_STEERING
				if (steering < 0.001) {
					autopilot_set_turn_rad(1e6);
				} else {
					autopilot_set_turn_rad(1.0 / steering);
				}
#endif
				autopilot_set_motor_speed(throttle);
				break;

			case RC_MODE_CURRENT_BRAKE:
				if (!main_config.car.disable_motor) {
#if HAS_DIFF_STEERING
					comm_can_lock_vesc();
					comm_can_set_vesc_id(ID_ALL);
					bldc_interface_set_current_brake(throttle);
					comm_can_unlock_vesc();
#else
					bldc_interface_set_current_brake(throttle);
#endif
				}
				break;

			default:
				break;
			}

#if !HAS_DIFF_STEERING
			steering = utils_map(steering, -1.0, 1.0,
					main_config.car.steering_center + (main_config.car.steering_range / 2.0),
					main_config.car.steering_center - (main_config.car.steering_range / 2.0));
			servo_simple_set_pos_ramp(steering);
#endif
		} break;

		case CMD_SET_SERVO_DIRECT: {
			timeout_reset();

			int32_t ind = 0;
			float steering = buffer_get_float32(data, 1e6, &ind);
			utils_truncate_number(&steering, 0.0, 1.0);
			servo_simple_set_pos_ramp(steering);
		} break;
#endif
#if MAIN_MODE == MAIN_MODE_MULTIROTOR
		case CMD_MR_GET_STATE: {
			timeout_reset();

			POS_STATE pos;
			float accel[3];
			float gyro[3];
			float mag[3];
			ROUTE_POINT rp_goal;

			commands_set_send_func(func);

			pos_get_imu(accel, gyro, mag);
			pos_get_pos(&pos);
			autopilot_get_goal_now(&rp_goal);

			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret; // 1
			m_send_buffer[send_index++] = CMD_MR_GET_STATE; // 2
			m_send_buffer[send_index++] = FW_VERSION_MAJOR; // 3
			m_send_buffer[send_index++] = FW_VERSION_MINOR; // 4
			buffer_append_float32_auto(m_send_buffer, pos.roll, &send_index); // 8
			buffer_append_float32_auto(m_send_buffer, pos.pitch, &send_index); // 12
			buffer_append_float32_auto(m_send_buffer, pos.yaw, &send_index); // 16
			buffer_append_float32_auto(m_send_buffer, accel[0], &send_index); // 20
			buffer_append_float32_auto(m_send_buffer, accel[1], &send_index); // 24
			buffer_append_float32_auto(m_send_buffer, accel[2], &send_index); // 28
			buffer_append_float32_auto(m_send_buffer, gyro[0], &send_index); // 32
			buffer_append_float32_auto(m_send_buffer, gyro[1], &send_index); // 36
			buffer_append_float32_auto(m_send_buffer, gyro[2], &send_index); // 40
			buffer_append_float32_auto(m_send_buffer, mag[0], &send_index); // 44
			buffer_append_float32_auto(m_send_buffer, mag[1], &send_index); // 48
			buffer_append_float32_auto(m_send_buffer, mag[2], &send_index); // 52
			buffer_append_float32_auto(m_send_buffer, pos.px, &send_index); // 56
			buffer_append_float32_auto(m_send_buffer, pos.py, &send_index); // 60
			buffer_append_float32_auto(m_send_buffer, pos.pz, &send_index); // 64
			buffer_append_float32_auto(m_send_buffer, pos.speed, &send_index); // 68
			buffer_append_float32_auto(m_send_buffer, adconv_get_vin(), &send_index); // 72
			buffer_append_float32_auto(m_send_buffer, pos.px_gps, &send_index); // 76
			buffer_append_float32_auto(m_send_buffer, pos.py_gps, &send_index); // 80
			buffer_append_float32_auto(m_send_buffer, rp_goal.px, &send_index); // 84
			buffer_append_float32_auto(m_send_buffer, rp_goal.py, &send_index); // 88
			buffer_append_int32(m_send_buffer, pos_get_ms_today(), &send_index); // 92
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_MR_RC_CONTROL: {
			int32_t ind = 0;
			float throttle = buffer_get_float32_auto(data, &ind);
			float roll = buffer_get_float32_auto(data, &ind);
			float pitch = buffer_get_float32_auto(data, &ind);
			float yaw = buffer_get_float32_auto(data, &ind);
			mr_control_set_input(throttle, roll, pitch, yaw);
		} break;

		case CMD_MR_OVERRIDE_POWER: {
			int32_t ind = 0;
			mr_control_set_motor_override(0, buffer_get_float32_auto(data, &ind));
			mr_control_set_motor_override(1, buffer_get_float32_auto(data, &ind));
			mr_control_set_motor_override(2, buffer_get_float32_auto(data, &ind));
			mr_control_set_motor_override(3, buffer_get_float32_auto(data, &ind));
		} break;
#endif
#endif

		// ==================== Mote commands ==================== //
#if MAIN_MODE_IS_MOTE
		case CMD_MOTE_UBX_START_BASE: {
			commands_set_send_func(func);

			ubx_cfg_tmode3 cfg;
			memset(&cfg, 0, sizeof(ubx_cfg_tmode3));
			int32_t ind = 0;

			cfg.mode = data[ind++];
			cfg.lla = true;
			cfg.ecefx_lat = buffer_get_double64(data, D(1e16), &ind);
			cfg.ecefy_lon = buffer_get_double64(data, D(1e16), &ind);
			cfg.ecefz_alt = buffer_get_float32_auto(data, &ind);
			cfg.fixed_pos_acc = buffer_get_float32_auto(data, &ind);
			cfg.svin_min_dur = buffer_get_uint32(data, &ind);
			cfg.svin_acc_limit = buffer_get_float32_auto(data, &ind);

			if (cfg.mode == 0) {
				m8t_base_stop();
				ublox_cfg_tmode3(&cfg);

				// Switch off RTCM messages, set rate to 5 Hz and time reference to UTC
				ublox_cfg_msg(UBX_CLASS_RTCM3, UBX_RTCM3_1005, 0);
				ublox_cfg_msg(UBX_CLASS_RTCM3, UBX_RTCM3_1077, 0);
				ublox_cfg_msg(UBX_CLASS_RTCM3, UBX_RTCM3_1087, 0);
				ublox_cfg_rate(200, 1, 0);

				// Automotive dynamic model
				ubx_cfg_nav5 nav5;
				memset(&nav5, 0, sizeof(ubx_cfg_nav5));
				nav5.apply_dyn = true;
				nav5.dyn_model = 4;
				ublox_cfg_nav5(&nav5);
			} else if (cfg.mode == 1 || cfg.mode == 2) {
				ublox_cfg_tmode3(&cfg);

				// Switch on RTCM messages, set rate to 1 Hz and time reference to UTC
				ublox_cfg_rate(1000, 1, 0);
				ublox_cfg_msg(UBX_CLASS_RTCM3, UBX_RTCM3_1005, 1); // Every second
				ublox_cfg_msg(UBX_CLASS_RTCM3, UBX_RTCM3_1077, 1); // Every second
				ublox_cfg_msg(UBX_CLASS_RTCM3, UBX_RTCM3_1087, 1); // Every second

				// Stationary dynamic model
				ubx_cfg_nav5 nav5;
				memset(&nav5, 0, sizeof(ubx_cfg_nav5));
				nav5.apply_dyn = true;
				nav5.dyn_model = 2;
				ublox_cfg_nav5(&nav5);
			} else if (cfg.mode == 3) {
				m8t_base_set_min_acc_samples(cfg.svin_acc_limit, cfg.svin_min_dur);
				m8t_base_start();
			} else if (cfg.mode == 4) {
				m8t_base_set_pos(cfg.ecefx_lat, cfg.ecefy_lon, cfg.ecefz_alt);
				m8t_base_start();
			}

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = CMD_MOTE_UBX_START_BASE_ACK;
			commands_send_packet(m_send_buffer, send_index);
		} break;
#endif

		default:
			break;
		}
	}
}

void commands_printf(const char* format, ...) {
	if (!m_init_done) {
		return;
	}

	chMtxLock(&m_print_gps);
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
	chMtxUnlock(&m_print_gps);
}

void commands_printf_log_usb(char* format, ...) {
	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[255];

	print_buffer[0] = ID_CAR_CLIENT;
	print_buffer[1] = CMD_LOG_LINE_USB;
	len = vsnprintf(print_buffer + 2, 253, format, arg);
	va_end (arg);

	if(len > 0) {
		comm_usb_send_packet((unsigned char*)print_buffer, (len<253) ? len + 2: 255);
	}
}

void commands_forward_vesc_packet(unsigned char *data, unsigned int len) {
	m_send_buffer[0] = main_id;
	m_send_buffer[1] = CMD_VESC_FWD;
	memcpy(m_send_buffer + 2, data, len);
	commands_send_packet((unsigned char*)m_send_buffer, len + 2);
}

void commands_send_nmea(unsigned char *data, unsigned int len) {
	if (main_config.gps_send_nmea) {
		int32_t send_index = 0;
		m_send_buffer[send_index++] = main_id;
		m_send_buffer[send_index++] = CMD_SEND_NMEA_RADIO;
		memcpy(m_send_buffer + send_index, data, len);
		send_index += len;
		commands_send_packet(m_send_buffer, send_index);
	}
}

void commands_init_plot(char *namex, char *namey) {
	int ind = 0;
	m_send_buffer[ind++] = main_id;
	m_send_buffer[ind++] = CMD_PLOT_INIT;
	memcpy(m_send_buffer + ind, namex, strlen(namex));
	ind += strlen(namex);
	m_send_buffer[ind++] = '\0';
	memcpy(m_send_buffer + ind, namey, strlen(namey));
	ind += strlen(namey);
	m_send_buffer[ind++] = '\0';
	commands_send_packet((unsigned char*)m_send_buffer, ind);
}

void commands_plot_add_graph(char *name) {
	int ind = 0;
	m_send_buffer[ind++] = main_id;
	m_send_buffer[ind++] = CMD_PLOT_ADD_GRAPH;
	memcpy(m_send_buffer + ind, name, strlen(name));
	ind += strlen(name);
	m_send_buffer[ind++] = '\0';
	commands_send_packet((unsigned char*)m_send_buffer, ind);
}

void commands_plot_set_graph(int graph) {
	int ind = 0;
	m_send_buffer[ind++] = main_id;
	m_send_buffer[ind++] = CMD_PLOT_SET_GRAPH;
	m_send_buffer[ind++] = graph;
	commands_send_packet((unsigned char*)m_send_buffer, ind);
}

void commands_send_plot_points(float x, float y) {
	int32_t ind = 0;
	m_send_buffer[ind++] = main_id;
	m_send_buffer[ind++] = CMD_PLOT_DATA;
	buffer_append_float32_auto(m_send_buffer, x, &ind);
	buffer_append_float32_auto(m_send_buffer, y, &ind);
	commands_send_packet((unsigned char*)m_send_buffer, ind);
}

void commands_send_radar_samples(float *dists, int num) {
	if (num > 24) {
		num = 24;
	}

	int32_t ind = 0;
	m_send_buffer[ind++] = main_id;
	m_send_buffer[ind++] = CMD_RADAR_SAMPLES;
	for (int i = 0;i < num;i++) {
		buffer_append_float32_auto(m_send_buffer, dists[i], &ind);
	}
	commands_send_packet((unsigned char*)m_send_buffer, ind);
}

void commands_send_dw_sample(DW_LOG_INFO *dw) {
	int32_t ind = 0;
	m_send_buffer[ind++] = main_id;
	m_send_buffer[ind++] = CMD_DW_SAMPLE;
	m_send_buffer[ind++] = dw->valid;
	m_send_buffer[ind++] = dw->dw_anchor;
	buffer_append_int32(m_send_buffer, dw->time_today_ms, &ind);
	buffer_append_float32_auto(m_send_buffer, dw->dw_dist, &ind);
	buffer_append_float32_auto(m_send_buffer, dw->px, &ind);
	buffer_append_float32_auto(m_send_buffer, dw->py, &ind);
	buffer_append_float32_auto(m_send_buffer, dw->px_gps, &ind);
	buffer_append_float32_auto(m_send_buffer, dw->py_gps, &ind);
	buffer_append_float32_auto(m_send_buffer, dw->pz_gps, &ind);

#if LOG_DW_FORCE_CC1120
	if (comm_cc1120_init_done()) {
		comm_cc1120_send_buffer(m_send_buffer, ind);
	} else {
		commands_send_packet(m_send_buffer, ind);
	}
#else
	commands_send_packet(m_send_buffer, ind);
#endif
}

void commands_send_log_ethernet(unsigned char *data, int len) {
	int32_t ind = 0;
	m_send_buffer[ind++] = ID_CAR_CLIENT;
	m_send_buffer[ind++] = CMD_LOG_ETHERNET;
	memcpy(m_send_buffer + ind, data, len);
	ind += len;
	comm_usb_send_packet(m_send_buffer, ind);
}

static void stop_forward(void *p) {
	(void)p;
	bldc_interface_set_forward_func(0);
}

static void rtcm_rx(uint8_t *data, int len, int type) {
	(void)type;

#if UBLOX_EN
	ublox_send(data, len);
	(void)m_send_buffer;
#else
	int32_t send_index = 0;
	m_send_buffer[send_index++] = main_id;
	m_send_buffer[send_index++] = CMD_SEND_RTCM_USB;
	memcpy(m_send_buffer + send_index, data, len);
	send_index += len;
	comm_usb_send_packet(m_send_buffer, send_index);
#endif
}

static void rtcm_base_rx(rtcm_ref_sta_pos_t *pos) {
	if (main_config.gps_use_rtcm_base_as_enu_ref) {
		pos_set_enu_ref(pos->lat, pos->lon, pos->height);
	}
}

rtcm3_state* commands_get_rtcm3_state(void) {
	return &m_rtcm_state;
}
