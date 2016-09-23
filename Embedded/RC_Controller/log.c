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

#include <string.h>

// Variables
bool m_log_en;
bool m_write_split;
char m_log_name[LOG_NAME_MAX_LEN + 1];

// Threads
static THD_WORKING_AREA(log_thread_wa, 2048);
static THD_FUNCTION(log_thread, arg);

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
			if (m_write_split) {
				commands_printf_log_usb("//%s\n", m_log_name);
				m_write_split = false;
			}

#ifdef LOG_EN_CARREL
			mc_values val;
			POS_STATE pos;
			float accel[3];
			float gyro[3];
			float mag[3];

			float steering_angle = (servo_simple_get_pos_now()
									- main_config.steering_center)
											* ((2.0 * main_config.steering_max_angle_rad)
													/ main_config.steering_range);

			pos_get_mc_val(&val);
			pos_get_pos(&pos);
			pos_get_imu(accel, gyro, mag);
			uint32_t time = chVTGetSystemTimeX();

			commands_printf_log_usb(
					"%u "     // timestamp
					"%.2f "   // temp_mos
					"%.2f "   // current_in
					"%.2f "   // current_motor
					"%.2f "   // v_in
					"%.3f "   // px
					"%.3f "   // py
					"%.3f "   // px_gps
					"%.3f "   // py_gps
					"%.3f "   // pz_gps
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
					"%.3f\n", // servo

					time,
					(double)val.temp_mos,
					(double)val.current_in,
					(double)val.current_motor,
					(double)val.v_in,
					(double)pos.px,
					(double)pos.py,
					(double)pos.px_gps,
					(double)pos.py_gps,
					(double)pos.pz_gps,
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
					"%.3f\n", // steering angle

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
