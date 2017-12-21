/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ch.h"
#include "hal.h"
#include "ahrs.h"
#include "stm32f4xx_conf.h"
#include "led.h"
#include "mpu9150.h"
#include "bldc_interface.h"
#include "utils.h"
#include "servo_simple.h"
#include "commands.h"
#include "ublox.h"
#include "mr_control.h"
#include "srf10.h"
#include "terminal.h"
#include "log.h"

// Defines
#define ITERATION_TIMER_FREQ			50000
#define POS_HISTORY_LEN					100

// Private variables
static ATTITUDE_INFO m_att;
static POS_STATE m_pos;
static GPS_STATE m_gps;
static bool m_attitude_init_done;
static float m_accel[3];
static float m_gyro[3];
static float m_mag[3];
static float m_mag_raw[3];
static mc_values m_mc_val;
static float m_imu_yaw;
static float m_yaw_offset_gps;
static mutex_t m_mutex_pos;
static mutex_t m_mutex_gps;
static int32_t m_ms_today;
static bool m_ubx_pos_valid;
static int32_t m_nma_last_time;
static POS_POINT m_pos_history[POS_HISTORY_LEN];
static int m_pos_history_ptr;
static bool m_pos_history_print;
static int32_t m_pps_cnt;

// Private functions
static void cmd_terminal_delay_info(int argc, const char **argv);
static void mpu9150_read(void);
static void update_orientation_angles(float *accel, float *gyro, float *mag, float dt);
static void init_gps_local(GPS_STATE *gps);
static double nmea_parse_val(char *str);
static void ublox_relposned_rx(ubx_nav_relposned *pos);
static void save_pos_history(void);
static POS_POINT get_closest_point_to_time(int32_t time);
static void correct_pos_gps(POS_STATE *pos);

#if MAIN_MODE == MAIN_MODE_CAR
static void mc_values_received(mc_values *val);
#elif MAIN_MODE == MAIN_MODE_MULTIROTOR
static void srf_distance_received(float distance);
#endif

#if MAIN_MODE == MAIN_MODE_MULTIROTOR
static void mr_update_pos(POS_STATE *pos, float dt);
#endif

void pos_init(void) {
	ahrs_init_attitude_info(&m_att);
	m_attitude_init_done = false;
	memset(&m_pos, 0, sizeof(m_pos));
	memset(&m_gps, 0, sizeof(m_gps));
	memset(&m_mc_val, 0, sizeof(m_mc_val));
	m_imu_yaw = 0.0;
	m_ubx_pos_valid = true;
	m_nma_last_time = 0;
	memset(&m_pos_history, 0, sizeof(m_pos_history));
	m_pos_history_ptr = 0;
	m_pos_history_print = false;
	m_pps_cnt = 0;
	m_yaw_offset_gps = 0.0;

	m_ms_today = -1;
	chMtxObjectInit(&m_mutex_pos);
	chMtxObjectInit(&m_mutex_gps);

	mpu9150_init();
	chThdSleepMilliseconds(1000);
	led_write(LED_RED, 1);
	mpu9150_sample_gyro_offsets(100);
	led_write(LED_RED, 0);

	// Iteration timer (ITERATION_TIMER_FREQ Hz)
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	uint16_t PrescalerValue = (uint16_t)((168e6 / 2) / ITERATION_TIMER_FREQ) - 1;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM6, ENABLE);

	mpu9150_set_read_callback(mpu9150_read);
	ublox_set_rx_callback_relposned(ublox_relposned_rx);

#if MAIN_MODE == MAIN_MODE_CAR
	bldc_interface_set_rx_value_func(mc_values_received);
#elif MAIN_MODE == MAIN_MODE_MULTIROTOR
	srf10_set_sample_callback(srf_distance_received);
#endif

	// PPS interrupt
#if UBLOX_EN && UBLOX_USE_PPS
	extChannelEnable(&EXTD1, 8);
#elif GPS_EXT_PPS
	palSetPadMode(GPIOD, 4, PAL_MODE_INPUT_PULLDOWN);
	extChannelEnable(&EXTD1, 4);
#endif

	terminal_register_command_callback(
			"delay_info",
			"Print delay information when doing GNSS position correction.\n"
			"  0 - Disabled\n"
			"  1 - Enabled",
			"[print_en]",
			cmd_terminal_delay_info);

#ifdef LOG_EN_SW
	palSetPadMode(GPIOD, 3, PAL_MODE_INPUT_PULLUP);
#endif

	(void)save_pos_history();
}

void pos_pps_cb(EXTDriver *extp, expchannel_t channel) {
	(void)extp;
	(void)channel;

	m_pps_cnt++;

	// Assume that the last NMEA time stamp is less than one second
	// old and round to the closest second after it.
	if (m_nma_last_time != 0) {
		int32_t s_today = m_nma_last_time / 1000;
		s_today++;
		m_ms_today = s_today * 1000;
	}
}

void pos_get_imu(float *accel, float *gyro, float *mag) {
	if (accel) {
		accel[0] = m_accel[0];
		accel[1] = m_accel[1];
		accel[2] = m_accel[2];
	}

	if (gyro) {
		gyro[0] = m_gyro[0];
		gyro[1] = m_gyro[1];
		gyro[2] = m_gyro[2];
	}

	if (mag) {
		mag[0] = m_mag_raw[0];
		mag[1] = m_mag_raw[1];
		mag[2] = m_mag_raw[2];
	}
}

void pos_get_quaternions(float *q) {
	chMtxLock(&m_mutex_pos);
	q[0] = m_pos.q0;
	q[1] = m_pos.q1;
	q[2] = m_pos.q2;
	q[3] = m_pos.q3;
	chMtxUnlock(&m_mutex_pos);
}

void pos_get_pos(POS_STATE *p) {
	chMtxLock(&m_mutex_pos);
	*p = m_pos;
	chMtxUnlock(&m_mutex_pos);
}

void pos_get_gps(GPS_STATE *p) {
	chMtxLock(&m_mutex_gps);
	*p = m_gps;
	chMtxUnlock(&m_mutex_gps);
}

float pos_get_speed(void) {
	return m_pos.speed;
}

void pos_set_xya(float x, float y, float angle) {
	chMtxLock(&m_mutex_pos);
	chMtxLock(&m_mutex_gps);

	m_pos.px = x;
	m_pos.py = y;
	m_pos.yaw = angle;
	m_yaw_offset_gps = m_imu_yaw - angle;

	chMtxUnlock(&m_mutex_gps);
	chMtxUnlock(&m_mutex_pos);
}

void pos_set_yaw_offset(float angle) {
	chMtxLock(&m_mutex_pos);

	m_yaw_offset_gps = angle;
	utils_norm_angle(&m_yaw_offset_gps);
	m_pos.yaw = m_imu_yaw - m_yaw_offset_gps;
	utils_norm_angle(&m_pos.yaw);

	chMtxUnlock(&m_mutex_pos);
}

void pos_set_enu_ref(double lat, double lon, double height) {
	double x, y, z;
	utils_llh_to_xyz(lat, lon, height, &x, &y, &z);

	chMtxLock(&m_mutex_gps);

	m_gps.ix = x;
	m_gps.iy = y;
	m_gps.iz = z;

	float so = sinf((float)lon * M_PI / 180.0);
	float co = cosf((float)lon * M_PI / 180.0);
	float sa = sinf((float)lat * M_PI / 180.0);
	float ca = cosf((float)lat * M_PI / 180.0);

	// ENU
	m_gps.r1c1 = -so;
	m_gps.r1c2 = co;
	m_gps.r1c3 = 0.0;

	m_gps.r2c1 = -sa * co;
	m_gps.r2c2 = -sa * so;
	m_gps.r2c3 = ca;

	m_gps.r3c1 = ca * co;
	m_gps.r3c2 = ca * so;
	m_gps.r3c3 = sa;

	m_gps.lx = 0.0;
	m_gps.ly = 0.0;
	m_gps.lz = 0.0;

	m_gps.local_init_done = true;

	chMtxUnlock(&m_mutex_gps);
}

void pos_get_enu_ref(double *llh) {
	chMtxLock(&m_mutex_gps);
	utils_xyz_to_llh(m_gps.ix, m_gps.iy, m_gps.iz, &llh[0], &llh[1], &llh[2]);
	chMtxUnlock(&m_mutex_gps);
}

void pos_reset_enu_ref(void) {
	m_gps.local_init_done = false;
}

void pos_get_mc_val(mc_values *v) {
	*v = m_mc_val;
}

int32_t pos_get_ms_today(void) {
	return m_ms_today;
}

void pos_set_ms_today(int32_t ms) {
	m_ms_today = ms;
}

bool pos_input_nmea(const char *data) {
	static char nmea_str[1024];
	int32_t ms = -1;
	double lat = 0.0;
	double lon = 0.0;
	double height = 0.0;
	int fix_type = 0;
	int sats = 0;
	int ind = 0;

	bool found = false;
	int len = strlen(data);

	for (int i = 0;i < 10;i++) {
		if ((i + 5) >= len) {
			break;
		}

		if (    data[i] == 'G' &&
				data[i + 1] == 'G' &&
				data[i + 2] == 'A' &&
				data[i + 3] == ',') {
			found = true;
			strcpy(nmea_str, data + i + 4);
			break;
		}
	}

	if (found) {
		char *gga, *str;

		str = nmea_str;
		gga = strsep(&str, ",");

		while (gga != 0) {
			switch (ind) {
			case 0: {
				// Time
				int h, m, s, ds;
				if (sscanf(gga, "%02d%02d%02d.%d", &h, &m, &s, &ds) == 4) {
					ms = h * 60 * 60 * 1000;
					ms += m * 60 * 1000;
					ms += s * 1000;
					ms += ds * 10;
				} else {
					ms = -1;
				}
			} break;

			case 1: {
				// Latitude
				lat = nmea_parse_val(gga);
			} break;

			case 2:
				// Latitude direction
				if (*gga == 'S' || *gga == 's') {
					lat = -lat;
				}
				break;

			case 3: {
				// Longitude
				lon = nmea_parse_val(gga);
			} break;

			case 4:
				// Longitude direction
				if (*gga == 'W' || *gga == 'w') {
					lon = -lon;
				}
				break;

			case 5:
				// Fix type
				if (sscanf(gga, "%d", &fix_type) != 1) {
					fix_type = -1;
				}
				break;

			case 6:
				// Satellites
				if (sscanf(gga, "%d", &sats) != 1) {
					sats = 0;
				}
				break;

			case 8:
				// Altitude
				if (sscanf(gga, "%lf", &height) != 1) {
					height = 0.0;
				}
				break;

			case 10: {
				// Altitude 2
				double h2 = 0.0;
				if (sscanf(gga, "%lf", &h2) != 1) {
					h2 = 0.0;
				}
				height += h2;
			} break;

			default:
				break;
			}

			gga = strsep(&str, ",");
			ind++;
		}
	}

	if (ms >= 0) {
		m_nma_last_time = ms;

#if !(UBLOX_EN && UBLOX_USE_PPS) && !GPS_EXT_PPS
		m_ms_today = ms;
#endif
	}

	// Only use valid fixes
	if (fix_type == 1 || fix_type == 2 || fix_type == 4 || fix_type == 5) {
		// Convert llh to ecef
		double sinp = sin(lat * D_PI / D(180.0));
		double cosp = cos(lat * D_PI / D(180.0));
		double sinl = sin(lon * D_PI / D(180.0));
		double cosl = cos(lon * D_PI / D(180.0));
		double e2 = FE_WGS84 * (D(2.0) - FE_WGS84);
		double v = RE_WGS84 / sqrt(D(1.0) - e2 * sinp * sinp);

		chMtxLock(&m_mutex_gps);

		m_gps.lat = lat;
		m_gps.lon = lon;
		m_gps.height = height;
		m_gps.fix_type = fix_type;
		m_gps.sats = sats;
		m_gps.ms = ms;
		m_gps.x = (v + height) * cosp * cosl;
		m_gps.y = (v + height) * cosp * sinl;
		m_gps.z = (v * (D(1.0) - e2) + height) * sinp;

		// Continue if ENU frame is initialized
		if (m_gps.local_init_done) {
			float dx = (float)(m_gps.x - m_gps.ix);
			float dy = (float)(m_gps.y - m_gps.iy);
			float dz = (float)(m_gps.z - m_gps.iz);

			m_gps.lx = m_gps.r1c1 * dx + m_gps.r1c2 * dy + m_gps.r1c3 * dz;
			m_gps.ly = m_gps.r2c1 * dx + m_gps.r2c2 * dy + m_gps.r2c3 * dz;
			m_gps.lz = m_gps.r3c1 * dx + m_gps.r3c2 * dy + m_gps.r3c3 * dz;

			float px = m_gps.lx;
			float py = m_gps.ly;

			// Apply antenna offset
			const float s_yaw = sinf(-m_pos.yaw * M_PI / 180.0);
			const float c_yaw = cosf(-m_pos.yaw * M_PI / 180.0);
			px -= c_yaw * main_config.gps_ant_x + s_yaw * main_config.gps_ant_y;
			py -= s_yaw * main_config.gps_ant_x + c_yaw * main_config.gps_ant_y;

			chMtxLock(&m_mutex_pos);

			m_pos.px_gps_last = m_pos.px_gps;
			m_pos.py_gps_last = m_pos.py_gps;
			m_pos.pz_gps_last = m_pos.pz_gps;
			m_pos.gps_ms_last = m_pos.gps_ms;

			m_pos.px_gps = px;
			m_pos.py_gps = py;
			m_pos.pz_gps = m_gps.lz;
			m_pos.gps_ms = m_gps.ms;
			m_pos.gps_fix_type = m_gps.fix_type;

			// Correct position
			// Optionally require RTK and good ublox quality indication.
			if (main_config.gps_comp &&
					(!main_config.gps_req_rtk || (fix_type == 4 || fix_type == 5)) &&
					(!main_config.gps_use_ubx_info || m_ubx_pos_valid)) {

				m_pos.gps_last_corr_diff = sqrtf(SQ(m_pos.px - m_pos.px_gps) +
						SQ(m_pos.py - m_pos.py_gps));
				log_update_corr_pos(&m_pos);

				correct_pos_gps(&m_pos);
				m_pos.gps_corr_time = chVTGetSystemTimeX();

#if MAIN_MODE == MAIN_MODE_CAR
				m_pos.pz = m_pos.pz_gps - m_pos.gps_ground_level;
#elif MAIN_MODE == MAIN_MODE_MULTIROTOR
				// Update height from GPS if ultrasound measurements haven't been received for a while
				if (ST2MS(chVTTimeElapsedSinceX(m_pos.ultra_update_time)) > 250) {
					m_pos.pz = m_pos.pz_gps - m_pos.gps_ground_level;
				}
#endif
			}

			m_pos.gps_corr_cnt = 0.0;

			chMtxUnlock(&m_mutex_pos);
		} else {
			init_gps_local(&m_gps);
			m_gps.local_init_done = true;
		}

		m_gps.update_time = chVTGetSystemTimeX();

		chMtxUnlock(&m_mutex_gps);
	}

	return found;
}

void pos_reset_attitude(void) {
	m_attitude_init_done = false;
}

/**
 * Get time since GPS correction was applied.
 *
 * @return
 * The time since GPS correction was applied in milliseconds.
 */
int pos_time_since_gps_corr(void) {
	return ST2MS(chVTTimeElapsedSinceX(m_pos.gps_corr_time));
}

static void cmd_terminal_delay_info(int argc, const char **argv) {
	if (argc == 2) {
		if (strcmp(argv[1], "0") == 0) {
			m_pos_history_print = 0;
			commands_printf("OK\n");
		} else if (strcmp(argv[1], "1") == 0) {
			m_pos_history_print = 1;
			commands_printf("OK\n");
		} else {
			commands_printf("Invalid argument %s\n", argv[1]);
		}
	} else {
		commands_printf("Wrong number of arguments\n");
	}
}

static void mpu9150_read(void) {
	float accel[3], gyro[3], mag[3];
	mpu9150_get_accel_gyro_mag(accel, gyro, mag);

	static unsigned int cnt_last = 0;
	volatile unsigned int cnt = TIM6->CNT;
	unsigned int time_elapsed = (cnt - cnt_last) % 65536;
	cnt_last = cnt;
	float dt = (float)time_elapsed / (float)ITERATION_TIMER_FREQ;

	update_orientation_angles(accel, gyro, mag, dt);

#if MAIN_MODE == MAIN_MODE_CAR
	// Read MC values every 10 iterations (should be 100 Hz)
	static int mc_read_cnt = 0;
	mc_read_cnt++;
	if (mc_read_cnt >= 10) {
		mc_read_cnt = 0;
		bldc_interface_get_values();
	}
#endif

#if MAIN_MODE == MAIN_MODE_MULTIROTOR
	if (mr_control_is_throttle_over_tres()) {
		chMtxLock(&m_mutex_pos);
		mr_update_pos(&m_pos, dt);
		chMtxUnlock(&m_mutex_pos);
	}
#endif

	// Update time today
	if (m_ms_today >= 0) {
		int time_elapsed_ms = time_elapsed / (ITERATION_TIMER_FREQ / 1000);

		// Accumulated error to avoid drift over time
		static int diff = 0;
		diff += time_elapsed % (ITERATION_TIMER_FREQ / 1000);
		int diff_div = diff / (ITERATION_TIMER_FREQ / 1000);
		diff -= diff_div * (ITERATION_TIMER_FREQ / 1000);

		m_ms_today += time_elapsed_ms + diff_div;

		if (m_ms_today >= MS_PER_DAY) {
			m_ms_today -= MS_PER_DAY;
		}
	}

#if MAIN_MODE == MAIN_MODE_MULTIROTOR
	mr_control_run_iteration(dt);
#endif

#ifdef LOG_EN_SW
	static int sw_cnt = 0;
	if (!palReadPad(GPIOD, 3) && sw_cnt > 100) {
		sw_cnt = 0;
		log_update_sw_pos(&m_pos);
	}
	sw_cnt++;
#endif
}

static void update_orientation_angles(float *accel, float *gyro, float *mag, float dt) {
	gyro[0] = gyro[0] * M_PI / 180.0;
	gyro[1] = gyro[1] * M_PI / 180.0;
	gyro[2] = gyro[2] * M_PI / 180.0;

	m_mag_raw[0] = mag[0];
	m_mag_raw[1] = mag[1];
	m_mag_raw[2] = mag[2];

	/*
	 * Hard and soft iron compensation
	 *
	 * http://davidegironi.blogspot.it/2013/01/magnetometer-calibration-helper-01-for.html#.UriTqkMjulM
	 *
	 * xt_raw = x_raw - offsetx;
	 * yt_raw = y_raw - offsety;
	 * zt_raw = z_raw - offsetz;
	 * x_calibrated = scalefactor_x[1] * xt_raw + scalefactor_x[2] * yt_raw + scalefactor_x[3] * zt_raw;
	 * y_calibrated = scalefactor_y[1] * xt_raw + scalefactor_y[2] * yt_raw + scalefactor_y[3] * zt_raw;
	 * z_calibrated = scalefactor_z[1] * xt_raw + scalefactor_z[2] * yt_raw + scalefactor_z[3] * zt_raw;
	 */
	if (main_config.mag_comp) {
		float mag_t[3];

		mag_t[0] = mag[0] - main_config.mag_cal_cx;
		mag_t[1] = mag[1] - main_config.mag_cal_cy;
		mag_t[2] = mag[2] - main_config.mag_cal_cz;

		mag[0] = main_config.mag_cal_xx * mag_t[0] + main_config.mag_cal_xy * mag_t[1] + main_config.mag_cal_xz * mag_t[2];
		mag[1] = main_config.mag_cal_yx * mag_t[0] + main_config.mag_cal_yy * mag_t[1] + main_config.mag_cal_yz * mag_t[2];
		mag[2] = main_config.mag_cal_zx * mag_t[0] + main_config.mag_cal_zy * mag_t[1] + main_config.mag_cal_zz * mag_t[2];
	}

	// Swap mag X and Y to match the accelerometer
	{
		float tmp[3];
		tmp[0] = mag[1];
		tmp[1] = mag[0];
		tmp[2] = mag[2];
		mag[0] = tmp[0];
		mag[1] = tmp[1];
		mag[2] = tmp[2];
	}

	// Rotate board yaw orientation
	float rotf = 0.0;

	// The MPU9250 footprint is rotated 180 degrees compared to the one
	// for the MPU9150 on our PCB. Make sure that the code behaves the
	// same regardless which one is used.
	if (mpu9150_is_mpu9250()) {
		rotf += 180.0;
	}

#ifdef BOARD_YAW_ROT
	rotf += BOARD_YAW_ROT;
#endif

	rotf *= M_PI / 180.0;
	utils_norm_angle_rad(&rotf);

	float cRot = cosf(rotf);
	float sRot = sinf(rotf);

	m_accel[0] = cRot * accel[0] + sRot * accel[1];
	m_accel[1] = cRot * accel[1] - sRot * accel[0];
	m_accel[2] = accel[2];
	m_gyro[0] = cRot * gyro[0] + sRot * gyro[1];
	m_gyro[1] = cRot * gyro[1] - sRot * gyro[0];
	m_gyro[2] = gyro[2];
	m_mag[0] = cRot * mag[0] + sRot * mag[1];
	m_mag[1] = cRot * mag[1] - sRot * mag[0];
	m_mag[2] = mag[2];

	if (!m_attitude_init_done) {
		ahrs_update_initial_orientation(m_accel, m_mag, (ATTITUDE_INFO*)&m_att);
		m_attitude_init_done = true;
	} else {
		//		ahrs_update_mahony_imu(gyro, accel, dt, (ATTITUDE_INFO*)&m_att);
		ahrs_update_madgwick_imu(m_gyro, m_accel, dt, (ATTITUDE_INFO*)&m_att);
	}

	float roll = ahrs_get_roll((ATTITUDE_INFO*)&m_att);
	float pitch = ahrs_get_pitch((ATTITUDE_INFO*)&m_att);
	float yaw = ahrs_get_yaw((ATTITUDE_INFO*)&m_att);

	// Apply tilt compensation for magnetometer values and calculate magnetic
	// field angle. See:
	// https://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf
	// Notice that hard and soft iron compensation is applied above
	float mx = -m_mag[0];
	float my = m_mag[1];
	float mz = m_mag[2];

	float sr = sinf(roll);
	float cr = cosf(roll);
	float sp = sinf(pitch);
	float cp = cosf(pitch);

	float c_mx = mx * cp + my * sr * sp + mz * sp * cr;
	float c_my = my * cr - mz * sr;

	float yaw_mag = atan2f(-c_my, c_mx) - M_PI / 2.0;

	chMtxLock(&m_mutex_pos);

	m_pos.roll = roll * 180.0 / M_PI;
	m_pos.pitch = pitch * 180.0 / M_PI;
	m_pos.roll_rate = -m_gyro[0] * 180.0 / M_PI;
	m_pos.pitch_rate = m_gyro[1] * 180.0 / M_PI;

	if (main_config.mag_use) {
		static float yaw_now = 0;
		static float yaw_imu_last = 0;

		float yaw_imu_diff = utils_angle_difference_rad(yaw, yaw_imu_last);
		yaw_imu_last = yaw;
		yaw_now += yaw_imu_diff;

		float diff = utils_angle_difference_rad(yaw_mag, yaw_now);
		yaw_now += SIGN(diff) * main_config.yaw_mag_gain * M_PI / 180.0 * dt;
		utils_norm_angle_rad(&yaw_now);

		m_imu_yaw = yaw_now * 180.0 / M_PI;
	} else {
		m_imu_yaw = yaw * 180.0 / M_PI;
	}

	utils_norm_angle(&m_imu_yaw);
	m_pos.yaw_rate = -m_gyro[2] * 180.0 / M_PI;

	// Correct yaw
#if MAIN_MODE == MAIN_MODE_CAR
	if (main_config.car.yaw_use_odometry) {
		if (main_config.car.yaw_imu_gain > 1e-10) {
			float ang_diff = utils_angle_difference(m_pos.yaw, m_imu_yaw - m_yaw_offset_gps);

			if (ang_diff > 1.2 * main_config.car.yaw_imu_gain) {
				m_pos.yaw -= main_config.car.yaw_imu_gain;
				utils_norm_angle(&m_pos.yaw);
			} else if (ang_diff < -1.2 * main_config.car.yaw_imu_gain) {
				m_pos.yaw += main_config.car.yaw_imu_gain;
				utils_norm_angle(&m_pos.yaw);
			} else {
				m_pos.yaw -= ang_diff;
				utils_norm_angle(&m_pos.yaw);
			}
		}
	} else {
		m_pos.yaw = m_imu_yaw - m_yaw_offset_gps;
		utils_norm_angle(&m_pos.yaw);
	}
#else
	m_pos.yaw = m_imu_yaw - m_yaw_offset_gps;
	utils_norm_angle(&m_pos.yaw);
#endif

	m_pos.q0 = m_att.q0;
	m_pos.q1 = m_att.q1;
	m_pos.q2 = m_att.q2;
	m_pos.q3 = m_att.q3;

	chMtxUnlock(&m_mutex_pos);
}

static void init_gps_local(GPS_STATE *gps) {
	gps->ix = gps->x;
	gps->iy = gps->y;
	gps->iz = gps->z;

	float so = sinf((float)gps->lon * M_PI / 180.0);
	float co = cosf((float)gps->lon * M_PI / 180.0);
	float sa = sinf((float)gps->lat * M_PI / 180.0);
	float ca = cosf((float)gps->lat * M_PI / 180.0);

	// ENU
	gps->r1c1 = -so;
	gps->r1c2 = co;
	gps->r1c3 = 0.0;

	gps->r2c1 = -sa * co;
	gps->r2c2 = -sa * so;
	gps->r2c3 = ca;

	gps->r3c1 = ca * co;
	gps->r3c2 = ca * so;
	gps->r3c3 = sa;

	gps->lx = 0.0;
	gps->ly = 0.0;
	gps->lz = 0.0;
}

static double nmea_parse_val(char *str) {
	int ind = -1;
	int len = strlen(str);
	double retval = 0.0;

	for (int i = 2;i < len;i++) {
		if (str[i] == '.') {
			ind = i - 2;
			break;
		}
	}

	if (ind >= 0) {
		char a[len + 1];
		memcpy(a, str, ind);
		a[ind] = ' ';
		memcpy(a + ind + 1, str + ind, len - ind);

		double l1, l2;
		if (sscanf(a, "%lf %lf", &l1, &l2) == 2) {
			retval = l1 + l2 / D(60.0);
		}
	}

	return retval;
}

static void ublox_relposned_rx(ubx_nav_relposned *pos) {
	bool valid = true;

	if (pos->acc_n > main_config.gps_ubx_max_acc) {
		valid = false;
	} else if (pos->acc_e > main_config.gps_ubx_max_acc) {
		valid = false;
	} else if (pos->acc_d > main_config.gps_ubx_max_acc) {
		valid = false;
	} else if (!pos->carr_soln) {
		valid = false;
	} else if (!pos->fix_ok) {
		valid = false;
	} else if (!pos->rel_pos_valid) {
		valid = false;
	}

	m_ubx_pos_valid = valid;
}

static void save_pos_history(void) {
	m_pos_history[m_pos_history_ptr].px = m_pos.px;
	m_pos_history[m_pos_history_ptr].py = m_pos.py;
	m_pos_history[m_pos_history_ptr].pz = m_pos.pz;
	m_pos_history[m_pos_history_ptr].yaw = m_pos.yaw;
	m_pos_history[m_pos_history_ptr].speed = m_pos.speed;
	m_pos_history[m_pos_history_ptr].time = m_ms_today;

	m_pos_history_ptr++;
	if (m_pos_history_ptr >= POS_HISTORY_LEN) {
		m_pos_history_ptr = 0;
	}
}

static POS_POINT get_closest_point_to_time(int32_t time) {
	int32_t ind = m_pos_history_ptr > 0 ? m_pos_history_ptr - 1 : POS_HISTORY_LEN - 1;
	int32_t min_diff = abs(time - m_pos_history[ind].time);
	int32_t ind_use = ind;

	int cnt = 0;
	for (;;) {
		ind = ind > 0 ? ind - 1 : POS_HISTORY_LEN - 1;

		if (ind == m_pos_history_ptr) {
			break;
		}

		int32_t diff = abs(time - m_pos_history[ind].time);

		if (diff < min_diff) {
			min_diff = diff;
			ind_use = ind;
		} else {
			break;
		}

		cnt++;
	}

	if (m_pos_history_print) {
		commands_printf("Age: %d ms Ind: %d, Diff: %d ms PPS_CNT: %d",
				m_ms_today - time, cnt, min_diff, m_pps_cnt);
	}

	return m_pos_history[ind_use];
}

static void correct_pos_gps(POS_STATE *pos) {
#if MAIN_MODE == MAIN_MODE_MULTIROTOR
	pos->gps_corr_cnt = sqrtf(SQ(pos->px_gps - pos->px_gps_last) +
			SQ(pos->py_gps - pos->py_gps_last));
#endif

	// Angle
	if (fabsf(pos->speed * 3.6) > 0.5) {
		float yaw_gps = atan2f(pos->py_gps - pos->gps_ang_corr_y_last_gps,
				pos->px_gps - pos->gps_ang_corr_x_last_gps);
		float yaw_car = atan2f(pos->py - pos->gps_ang_corr_y_last_car,
				pos->px - pos->gps_ang_corr_x_last_car);
		float yaw_diff = utils_angle_difference_rad(yaw_gps, yaw_car) * 180.0 / M_PI;

		utils_step_towards(&m_yaw_offset_gps, m_yaw_offset_gps + yaw_diff,
				main_config.gps_corr_gain_yaw * pos->gps_corr_cnt);

		// TODO: Finish and test new angle correction
		//		float dx = closest.px - pos->px_gps;
		//		float dy = closest.py - pos->py_gps;
		//		float d = sqrtf(SQ(dx) + SQ(dy));
		//		float al2 = atan2f(dy, dx);
		//		float al3 = utils_angle_difference((closest.yaw * (M_PI / 180.0)), al2);
		//		float ds = d * cosf(al3);
		//		if (d < 0.1) {
		//			d = 0.1;
		//		}
		//		m_yaw_offset_gps += (ds / d) * al3 * main_config.gps_corr_gain_yaw;
	}

	utils_norm_angle(&m_yaw_offset_gps);

	// Position
	float gain = main_config.gps_corr_gain_stat +
			main_config.gps_corr_gain_dyn * pos->gps_corr_cnt;

	POS_POINT closest = get_closest_point_to_time(pos->gps_ms);

	POS_POINT closest_corr = closest;
	utils_step_towards(&closest_corr.px, pos->px_gps, gain);
	utils_step_towards(&closest_corr.py, pos->py_gps, gain);
	pos->px += closest_corr.px - closest.px;
	pos->py += closest_corr.py - closest.py;

	pos->gps_ang_corr_x_last_gps = pos->px_gps;
	pos->gps_ang_corr_y_last_gps = pos->py_gps;
	pos->gps_ang_corr_x_last_car = closest.px;
	pos->gps_ang_corr_y_last_car = closest.py;

	// Update multirotor state
#if MAIN_MODE == MAIN_MODE_MULTIROTOR
	const systime_t time_now = chVTGetSystemTime();
	static systime_t time_last = 0;
	float dt = (float)(time_now - time_last) / (float)CH_CFG_ST_FREQUENCY;
	time_last = time_now;

	if (dt > 2.0 || dt < 0.01) {
		return;
	}

	// Velocity
	const float dt_gps = (pos->gps_ms - pos->gps_ms_last) / 1000.0;
	const float vx_gps = (pos->px_gps - pos->px_gps_last) / dt_gps;
	const float vy_gps = (pos->py_gps - pos->py_gps_last) / dt_gps;
	float error_vx = pos->vx - vx_gps;
	float error_vy = pos->vy - vy_gps;

	utils_truncate_number_abs(&error_vx, main_config.mr.max_corr_error);
	utils_truncate_number_abs(&error_vy, main_config.mr.max_corr_error);

	const float error_vx_diff = (error_vx - pos->error_vx_last);
	const float error_vy_diff = (error_vy - pos->error_vy_last);
	pos->error_vx_last = error_vx;
	pos->error_vy_last = error_vy;

	const float vcx_p = error_vx * main_config.mr.vel_gain_p;
	const float vcy_p = error_vy * main_config.mr.vel_gain_p;

	pos->vel_corr_x_int += error_vx * main_config.mr.vel_gain_i * dt;
	pos->vel_corr_y_int += error_vy * main_config.mr.vel_gain_i * dt;
	utils_truncate_number(&pos->vel_corr_x_int, -1.0, 1.0);
	utils_truncate_number(&pos->vel_corr_y_int, -1.0, 1.0);

	const float vcx_d = error_vx_diff * main_config.mr.vel_gain_d / dt;
	const float vcy_d = error_vy_diff * main_config.mr.vel_gain_d / dt;

	const float vcx_out = vcx_p + pos->vel_corr_x_int + vcx_d;
	const float vcy_out = vcy_p + pos->vel_corr_y_int + vcy_d;

	pos->vx -= vcx_out;
	pos->vy -= vcy_out;

	// Tilt
	const float acx_p = error_vx * main_config.mr.tilt_gain_p;
	const float acy_p = error_vy * main_config.mr.tilt_gain_p;

	pos->tilt_corr_x_int += error_vx * main_config.mr.tilt_gain_i * dt;
	pos->tilt_corr_y_int += error_vy * main_config.mr.tilt_gain_i * dt;
	utils_truncate_number(&pos->tilt_corr_x_int, -1.0, 1.0);
	utils_truncate_number(&pos->tilt_corr_y_int, -1.0, 1.0);

	const float acx_d = error_vx_diff * main_config.mr.tilt_gain_d / dt;
	const float acy_d = error_vy_diff * main_config.mr.tilt_gain_d / dt;

	const float acx_out = acx_p + pos->tilt_corr_x_int + acx_d;
	const float acy_out = acy_p + pos->tilt_corr_y_int + acy_d;

	const float cosy = cosf(-pos->yaw * M_PI / 180.0);
	const float siny = sinf(-pos->yaw * M_PI / 180.0);

	pos->tilt_pitch_err += acx_out * cosy + acy_out * siny;
	pos->tilt_roll_err += acy_out * cosy - acx_out * siny;

	utils_truncate_number_abs(&pos->tilt_roll_err, main_config.mr.max_tilt_error);
	utils_truncate_number_abs(&pos->tilt_pitch_err, main_config.mr.max_tilt_error);
#endif
}

#if MAIN_MODE == MAIN_MODE_CAR
static void mc_values_received(mc_values *val) {
	m_mc_val = *val;

	static int32_t last_tacho = 0;

	// Reset tacho the first time.
	static bool tacho_read = false;
	if (!tacho_read) {
		tacho_read = true;
		last_tacho = val->tachometer;
	}

	float distance = (float)(val->tachometer - last_tacho) * main_config.car.gear_ratio
			* (2.0 / main_config.car.motor_poles) * (1.0 / 6.0) * main_config.car.wheel_diam * M_PI;
	last_tacho = val->tachometer;

	float steering_angle = (servo_simple_get_pos_now()
			- main_config.car.steering_center)
													* ((2.0 * main_config.car.steering_max_angle_rad)
															/ main_config.car.steering_range);

	chMtxLock(&m_mutex_pos);

	if (fabsf(distance) > 1e-6) {
		float angle_rad = -m_pos.yaw * M_PI / 180.0;

		m_pos.gps_corr_cnt += fabsf(distance);

		if (!main_config.car.yaw_use_odometry || fabsf(steering_angle) < 0.00001) {
			m_pos.px += cosf(angle_rad) * distance;
			m_pos.py += sinf(angle_rad) * distance;
		} else {
			const float turn_rad_rear = main_config.car.axis_distance / tanf(steering_angle);
			float turn_rad_front = sqrtf(
					main_config.car.axis_distance * main_config.car.axis_distance
					+ turn_rad_rear * turn_rad_rear);

			if (turn_rad_rear < 0) {
				turn_rad_front = -turn_rad_front;
			}

			const float angle_diff = (distance * 2.0) / (turn_rad_rear + turn_rad_front);

			m_pos.px += turn_rad_rear * (sinf(angle_rad + angle_diff) - sinf(angle_rad));
			m_pos.py += turn_rad_rear * (cosf(angle_rad - angle_diff) - cosf(angle_rad));
			angle_rad += angle_diff;
			utils_norm_angle_rad(&angle_rad);
			m_pos.yaw = -angle_rad * 180.0 / M_PI;
			utils_norm_angle(&m_pos.yaw);
		}
	}

	m_pos.speed = val->rpm * main_config.car.gear_ratio
			* (2.0 / main_config.car.motor_poles) * (1.0 / 60.0)
			* main_config.car.wheel_diam * M_PI;

	save_pos_history();

	chMtxUnlock(&m_mutex_pos);
}
#endif

#if MAIN_MODE == MAIN_MODE_MULTIROTOR
static void srf_distance_received(float distance) {
	chMtxLock(&m_mutex_pos);
	m_pos.pz = distance;
	m_pos.ultra_update_time = chVTGetSystemTimeX();

	if (!mr_control_is_throttle_over_tres()) {
		m_pos.gps_ground_level = m_pos.pz_gps - m_pos.pz;
	}

	chMtxUnlock(&m_mutex_pos);
}

static void mr_update_pos(POS_STATE *pos, float dt) {
	float roll = pos->roll + pos->tilt_roll_err;
	float pitch = pos->pitch + pos->tilt_pitch_err;
	float yaw = pos->yaw;

	// Too much tilt means that this won't work anyway. Return in that case.
	if (fabsf(roll) > 45.0 || fabsf(pitch) > 45.0) {
		pos->vx = 0;
		pos->vy = 0;
		return;
	}

	roll = roll * M_PI / 180.0;
	pitch = pitch * M_PI / 180.0;
	yaw = yaw * M_PI / 180.0;

	const float acc_v = 9.82;
	const float cos_y = cosf(-yaw);
	const float sin_y = sinf(-yaw);

	const float dvx = -acc_v * tanf(pitch) * dt;
	const float dvy = -acc_v * tanf(roll) * dt;

	pos->vx += cos_y * dvx - sin_y * dvy;
	pos->vy += cos_y * dvy + sin_y * dvx;
	pos->px += pos->vx * dt;
	pos->py += pos->vy * dt;

	// Apply position and velocity limits
	if (utils_truncate_number(&pos->px, main_config.mr.map_min_x, main_config.mr.map_max_x)) {
		pos->vx = 0.0;
	} else {
		utils_truncate_number_abs(&pos->vx, main_config.mr.vel_max);
	}

	if (utils_truncate_number(&pos->py, main_config.mr.map_min_y, main_config.mr.map_max_y)) {
		pos->vy = 0;
	} else {
		utils_truncate_number_abs(&pos->vy, main_config.mr.vel_max);
	}

	// Exponential decay
	const float decay_factor = powf(main_config.mr.vel_decay_e, dt);
	pos->vx *= decay_factor;
	pos->vy *= decay_factor;

	// Linear decay
	utils_step_towards(&pos->vx, 0.0, main_config.mr.vel_decay_l * dt);
	utils_step_towards(&pos->vy, 0.0, main_config.mr.vel_decay_l * dt);

	// Update speed sum
	pos->speed = sqrtf(SQ(pos->vx) + SQ(pos->vy));
}
#endif
