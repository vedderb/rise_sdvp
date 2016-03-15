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
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "led.h"
#include "mpu9150.h"
#include "MahonyAHRS.h"

// Defines
#define ITERATION_TIMER_FREQ			50000

// Private variables
static ATTITUDE_INFO m_att_no_mag, m_att_mag;
static float m_roll_now, m_pitch_now, m_yaw_now;
static bool m_attitude_init_done;
static float m_accel[3];
static float m_gyro[3];
static float m_mag[3];

// Private functions
static void mpu9150_read(void);
static void update_orientation_angles(float *accel, float *gyro, float *mag, float dt);

void pos_init(void) {
	MahonyAHRSInitAttitudeInfo(&m_att_mag);
	MahonyAHRSInitAttitudeInfo(&m_att_no_mag);
	m_attitude_init_done = false;

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
}

void pos_get_attitude(float *rpy, float *accel, float *gyro, float *mag) {
	if (rpy) {
		rpy[0] = m_roll_now;
		rpy[1] = m_pitch_now;
		rpy[2] = m_yaw_now;
	}

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

static void mpu9150_read(void) {
	float accel[3], gyro[3], mag[3];
	mpu9150_get_accel_gyro_mag(accel, gyro, mag);

	float dt = (float)TIM6->CNT / (float)ITERATION_TIMER_FREQ;
	TIM6->CNT = 0;

	update_orientation_angles(accel, gyro, mag, dt);
}

static void update_orientation_angles(float *accel, float *gyro, float *mag, float dt) {
	gyro[0] = gyro[0] * M_PI / 180.0;
	gyro[1] = gyro[1] * M_PI / 180.0;
	gyro[2] = gyro[2] * M_PI / 180.0;

	// Swap X and Y to match the accelerometer of the MPU9150
	// TODO: Is this correct?
	float mag_tmp[3];
	mag_tmp[0] = mag[1];
	mag_tmp[1] = mag[0];
	mag_tmp[2] = mag[2];

	m_accel[0] = accel[0];
	m_accel[1] = accel[1];
	m_accel[2] = accel[2];
	m_gyro[0] = gyro[0];
	m_gyro[1] = gyro[1];
	m_gyro[2] = gyro[2];
	m_mag[0] = mag_tmp[0];
	m_mag[1] = mag_tmp[1];
	m_mag[2] = mag_tmp[2];

	if (!m_attitude_init_done) {
		MahonyAHRSupdateInitialOrientation(accel, mag_tmp, (ATTITUDE_INFO*)&m_att_mag);
		MahonyAHRSupdateInitialOrientation(accel, mag_tmp, (ATTITUDE_INFO*)&m_att_no_mag);
		m_attitude_init_done = true;
	} else {
		if (mpu9150_mag_updated() && fabsf(m_roll_now) < 25.0 && fabsf(m_pitch_now) < 25.0) {
			MahonyAHRSupdate(gyro, accel, mag_tmp, dt, (ATTITUDE_INFO*)&m_att_mag);
		} else {
			MahonyAHRSupdateIMU(gyro, accel, dt, (ATTITUDE_INFO*)&m_att_mag);
		}

		MahonyAHRSupdateIMU(gyro, accel, dt, (ATTITUDE_INFO*)&m_att_no_mag);
	}

	m_roll_now = MahonyAHRSGetRoll((ATTITUDE_INFO*)&m_att_no_mag) * 180 / M_PI;// - roll_offset;
	m_pitch_now = MahonyAHRSGetPitch((ATTITUDE_INFO*)&m_att_no_mag) * 180 / M_PI;// - pitch_offset;
	m_yaw_now = MahonyAHRSGetYaw((ATTITUDE_INFO*)&m_att_mag) * 180 / M_PI;// - yaw_offset;
}
