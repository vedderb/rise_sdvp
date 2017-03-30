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

#include "log.h"
#include "pos.h"
#include "commands.h"
#include "servo_simple.h"
#include "radar.h"
#include "comm_can.h"

#include <string.h>

// private variables
static bool m_log_en;
static bool m_write_split;
static char m_log_name[LOG_NAME_MAX_LEN + 1];
#ifdef LOG_EN_DW
static int m_dw_anchor_now = 0;
static DW_LOG_INFO m_dw_anchor_info[3];
#endif

// Threads
static THD_WORKING_AREA(log_thread_wa, 2048);
static THD_FUNCTION(log_thread, arg);

// Private functions
#ifdef LOG_EN_DW
static void range_callback(uint8_t id, uint8_t dest, float range);
#endif

void log_init(void) {
	m_log_en = false;
	m_write_split = true;
	strcpy(m_log_name, "Undefined");

	chThdCreateStatic(log_thread_wa, sizeof(log_thread_wa), NORMALPRIO, log_thread, NULL);
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

static THD_FUNCTION(log_thread, arg) {
	(void)arg;

	chRegSetThreadName("Log");

	systime_t time_p = chVTGetSystemTimeX(); // T0

	for(;;) {
		if (m_log_en) {
#ifndef LOG_EN_DW
			if (m_write_split) {
				commands_printf_log_usb("//%s\n", m_log_name);
				m_write_split = false;
			}
#endif

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
						- main_config.steering_center)
								* ((2.0 * main_config.steering_max_angle_rad)
										/ main_config.steering_range);

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
#elif defined(LOG_EN_DW)
			comm_can_set_range_func(range_callback);

			if (m_dw_anchor_now == 0 && LOG_DW_ANCHOR0 >= 0) {
				memset(m_dw_anchor_info, 0, sizeof(m_dw_anchor_info));
				comm_can_dw_range(CAN_DW_ID_ANY, LOG_DW_ANCHOR0, 5);
			} else if (m_dw_anchor_now == 1 && LOG_DW_ANCHOR1 >= 0) {
				comm_can_dw_range(CAN_DW_ID_ANY, LOG_DW_ANCHOR1, 5);
			} else if (m_dw_anchor_now == 2 && LOG_DW_ANCHOR2 >= 0) {
				comm_can_dw_range(CAN_DW_ID_ANY, LOG_DW_ANCHOR2, 5);
			} else if (m_dw_anchor_now >= 3) {
				int closest = -1;
				float min_dist = 1e20;

				for (int i = 0;i < 3;i++) {
					if (m_dw_anchor_info[i].valid &&
							m_dw_anchor_info[i].dw_dist < min_dist) {
						closest = i;
						min_dist = m_dw_anchor_info[i].dw_dist;
					}
				}

				if (closest >= 0) {
					commands_send_dw_sample(&m_dw_anchor_info[closest]);
				}

				m_dw_anchor_now = -1;
			}

			m_dw_anchor_now++;
#endif
		}

		time_p += MS2ST(LOG_INTERVAL_MS);
		systime_t time = chVTGetSystemTimeX();

		if (time_p >= time + 5) {
			chThdSleepUntil(time_p);
		} else {
			chThdSleepMilliseconds(1);
		}
	}
}

#ifdef LOG_EN_DW
static void range_callback(uint8_t id, uint8_t dest, float range) {
	(void)id;

	int now = m_dw_anchor_now - 1;

	if (now >= 0 && now < 3) {
		POS_STATE pos;
		pos_get_pos(&pos);

		m_dw_anchor_info[now].valid = true;
		m_dw_anchor_info[now].dw_anchor = dest;
		m_dw_anchor_info[now].time_today_ms = pos_get_ms_today();
		m_dw_anchor_info[now].dw_dist = range;
		m_dw_anchor_info[now].px = pos.px;
		m_dw_anchor_info[now].py = pos.py;
		m_dw_anchor_info[now].px_gps = pos.px_gps;
		m_dw_anchor_info[now].py_gps = pos.py_gps;
		m_dw_anchor_info[now].pz_gps = pos.pz_gps;
	}
}
#endif
