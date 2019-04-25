/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

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

#include "hydraulic.h"
#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "pwm_esc.h"
#include "utils.h"
#include <math.h>

// Settings
#define SERVO_LEFT				1
#define SERVO_RIGHT				2
#define SPEED_M_S				0.6
#define TIMEOUT_SECONDS			2.0

// Private variables
static float m_speed_now = 0.0;
static float m_distance_now = 0.0;
static float m_timeout_cnt = 0.0;

// Threads
static THD_WORKING_AREA(hydro_thread_wa, 1024);
static THD_FUNCTION(hydro_thread, arg);

void hydraulic_init(void) {
	main_config.mr.motor_pwm_min_us = 1000;
	main_config.mr.motor_pwm_max_us = 2100;

	pwm_esc_init();
	pwm_esc_set_all(0.5);

	chThdCreateStatic(hydro_thread_wa, sizeof(hydro_thread_wa), NORMALPRIO, hydro_thread, NULL);
}

/**
 * Get current speed
 *
 * @return
 * Speed in m/s
 */
float hydraulic_get_speed(void) {
	return m_speed_now;
}

/**
 * Get travel distance
 *
 * @param reset
 * Reset distance counter
 *
 * @return
 * Travel distance in meters
 */
float hydraulic_get_distance(bool reset) {
	float ret = m_distance_now;

	if (reset) {
		m_distance_now = 0.0;
	}

	return ret;
}

/**
 * Set speed in m/s
 *
 * @param speed
 * Speed in m/s
 */
void hydraulic_set_speed(float speed) {
	m_timeout_cnt = 0.0;

	if (fabsf(speed) < 0.01) {
		pwm_esc_set(SERVO_LEFT, 0.5);
		pwm_esc_set(SERVO_RIGHT, 0.5);
		m_speed_now = 0.0;
	} else {
		// TODO: Update this
		pwm_esc_set(SERVO_LEFT, speed > 0.0 ? 1.0 : 0.0);
		pwm_esc_set(SERVO_RIGHT, speed > 0.0 ? 0.0 : 1.0);
		m_speed_now = SIGN(speed) * SPEED_M_S;
	}
}

void hydraulic_set_throttle_raw(float throttle) {
	m_timeout_cnt = 0.0;
	utils_truncate_number_abs(&throttle, 1.0);

	if (fabsf(throttle) > 0.9) {
		m_speed_now = SIGN(throttle) * SPEED_M_S;
	} else {
		m_speed_now = 0.0;
	}

	float pos = utils_map(throttle, -1.0, 1.0, 0.0, 1.0);
	pwm_esc_set(SERVO_LEFT, pos);
	pwm_esc_set(SERVO_RIGHT, 1.0 - pos);
}

static THD_FUNCTION(hydro_thread, arg) {
	(void)arg;

	chRegSetThreadName("Hydraulic");

	for(;;) {
		if (fabsf(m_speed_now) > 0.01) {
			m_distance_now += m_speed_now * 0.01;
		}

		chThdSleepMilliseconds(10);

		m_timeout_cnt += 0.01;

		if (m_timeout_cnt >= TIMEOUT_SECONDS) {
			pwm_esc_set(SERVO_LEFT, 0.5);
			pwm_esc_set(SERVO_RIGHT, 0.5);
			m_speed_now = 0.0;
		}
	}
}
