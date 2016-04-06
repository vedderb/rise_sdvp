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

#include <math.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "led.h"
#include "mpu9150.h"
#include "MahonyAHRS.h"
#include "bldc_interface.h"
#include "utils.h"
#include "servo_simple.h"

// Defines
#define ITERATION_TIMER_FREQ			50000

// Private variables
static ATTITUDE_INFO m_att;
static POS_STATE m_pos;
static bool m_attitude_init_done;
static float m_accel[3];
static float m_gyro[3];
static float m_mag[3];
static mc_values m_mc_val;
static float m_imu_yaw;
static float m_yaw_offset;

// Private functions
static void mpu9150_read(void);
static void update_orientation_angles(float *accel, float *gyro, float *mag, float dt);
static void mc_values_received(mc_values *val);

void pos_init(void) {
	MahonyAHRSInitAttitudeInfo(&m_att);
	m_attitude_init_done = false;
	memset(&m_pos, 0, sizeof(m_pos));
	memset(&m_mc_val, 0, sizeof(m_mc_val));
	m_imu_yaw = 0.0;
	m_yaw_offset = 0.0;

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
	bldc_interface_set_rx_value_func(mc_values_received);
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
		mag[0] = m_mag[0];
		mag[1] = m_mag[1];
		mag[2] = m_mag[2];
	}
}

void pos_get_quaternions(float *q) {
	q[0] = m_pos.q0;
	q[1] = m_pos.q1;
	q[2] = m_pos.q2;
	q[3] = m_pos.q3;
}

void pos_get_pos(POS_STATE *p) {
	*p = m_pos;
}

void pos_set_xya(float x, float y, float angle) {
	m_pos.px = x;
	m_pos.py = y;
	m_pos.yaw = angle;

	m_yaw_offset = m_imu_yaw - angle;
}

void pos_get_mc_val(mc_values *v) {
	*v = m_mc_val;
}

static void mpu9150_read(void) {
	float accel[3], gyro[3], mag[3];
	mpu9150_get_accel_gyro_mag(accel, gyro, mag);

	float dt = (float)TIM6->CNT / (float)ITERATION_TIMER_FREQ;
	TIM6->CNT = 0;

	update_orientation_angles(accel, gyro, mag, dt);

	// Read MC values every 10 iterations (should be 100 Hz)
	static int mc_read_cnt = 0;
	mc_read_cnt++;
	if (mc_read_cnt >= 10) {
		mc_read_cnt = 0;
		bldc_interface_get_values();
	}
}

static void update_orientation_angles(float *accel, float *gyro, float *mag, float dt) {
	gyro[0] = gyro[0] * M_PI / 180.0;
	gyro[1] = gyro[1] * M_PI / 180.0;
	gyro[2] = gyro[2] * M_PI / 180.0;

	m_accel[0] = accel[0];
	m_accel[1] = accel[1];
	m_accel[2] = accel[2];
	m_gyro[0] = gyro[0];
	m_gyro[1] = gyro[1];
	m_gyro[2] = gyro[2];
	m_mag[0] = mag[0];
	m_mag[1] = mag[1];
	m_mag[2] = mag[2];

	// Swap X and Y to match the accelerometer of the MPU9150
	float mag_tmp[3];
	mag_tmp[0] = mag[1];
	mag_tmp[1] = mag[0];
	mag_tmp[2] = mag[2];

	if (!m_attitude_init_done) {
		MahonyAHRSupdateInitialOrientation(accel, mag_tmp, (ATTITUDE_INFO*)&m_att);
		m_attitude_init_done = true;
	} else {
		if (mpu9150_mag_updated()) {
			MahonyAHRSupdate(gyro, accel, mag_tmp, dt, (ATTITUDE_INFO*)&m_att);
		} else {
			MahonyAHRSupdateIMU(gyro, accel, dt, (ATTITUDE_INFO*)&m_att);
		}
	}

	m_pos.roll = MahonyAHRSGetRoll((ATTITUDE_INFO*)&m_att) * 180.0 / M_PI;
	m_pos.pitch = MahonyAHRSGetPitch((ATTITUDE_INFO*)&m_att) * 180.0 / M_PI;
	m_imu_yaw = MahonyAHRSGetYaw((ATTITUDE_INFO*)&m_att) * 180.0 / M_PI;
	utils_norm_angle(&m_imu_yaw);
	m_pos.roll_rate = -gyro[0] * 180.0 / M_PI;
	m_pos.pitch_rate = gyro[1] * 180.0 / M_PI;
	m_pos.yaw_rate = -gyro[2] * 180.0 / M_PI;

	// Correct yaw
	if (main_config.yaw_imu_gain > 1e-10) {
		float ang_diff = utils_angle_difference(m_pos.yaw, m_imu_yaw - m_yaw_offset);

		if (ang_diff > 1.2 * main_config.yaw_imu_gain) {
			m_pos.yaw -= main_config.yaw_imu_gain;
			utils_norm_angle(&m_pos.yaw);
		} else if (ang_diff < -1.2 * main_config.yaw_imu_gain) {
			m_pos.yaw += main_config.yaw_imu_gain;
			utils_norm_angle(&m_pos.yaw);
		}
	}

//	m_pos.yaw = yaw;

	m_pos.q0 = m_att.q0;
	m_pos.q1 = m_att.q1;
	m_pos.q2 = m_att.q2;
	m_pos.q3 = m_att.q3;
}

static void mc_values_received(mc_values *val) {
	m_mc_val = *val;

	m_pos.speed = val->rpm * main_config.gear_ratio
			* (2.0 / main_config.motor_poles) * (1.0 / 60.0)
			* main_config.wheel_diam * M_PI;

	static int32_t last_tacho = 0;

	// Reset tacho the first time.
	static bool tacho_read = false;
	if (!tacho_read) {
		tacho_read = true;
		last_tacho = val->tachometer;
	}

	float distance = (float)(val->tachometer - last_tacho) * main_config.gear_ratio
			* (2.0 / main_config.motor_poles) * (1.0 / 6.0) * main_config.wheel_diam * M_PI;
	last_tacho = val->tachometer;

	float steering_angle = (servo_simple_get_pos_now()
			- main_config.steering_center)
			* ((2.0 * main_config.steering_max_angle_rad)
					/ (main_config.steering_left - main_config.steering_right));

	if (fabsf(distance) > 1e-6) {
		float angle_rad = -m_pos.yaw * M_PI / 180.0;

		if (fabsf(steering_angle) < 0.00001) {
			m_pos.px += cosf(angle_rad) * distance;
			m_pos.py += sinf(angle_rad) * distance;
		} else {
			const float turn_rad_rear = main_config.axis_distance / tanf(steering_angle);
			float turn_rad_front = sqrtf(
					main_config.axis_distance * main_config.axis_distance
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
		}
	}
}
