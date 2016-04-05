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

#include "autopilot.h"
#include <math.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "servo_simple.h"
#include "utils.h"
#include "pos.h"
#include "bldc_interface.h"

// Defines
#define AP_HZ						100 // Hz

// Private variables
static THD_WORKING_AREA(ap_thread_wa, 512);
static ROUTE_POINT m_route[AP_ROUTE_SIZE];
static bool m_is_active;
static int m_point_last;
static int m_point_now;
static bool m_has_prev_point;
static float m_override_speed;
static bool m_is_speed_override;

// Private functions
static THD_FUNCTION(ap_thread, arg);
static void steering_angle_to_point(
		float current_x,
		float current_y,
		float current_angle,
		float goal_x,
		float goal_y,
		float *steering_angle,
		float *distance);

void autopilot_init(void) {
	memset(m_route, 0, sizeof(m_route));
	m_is_active = false;
	m_point_now = 0;
	m_point_last = 0;
	m_has_prev_point = false;
	m_override_speed = 0.0;
	m_is_speed_override = false;

	chThdCreateStatic(ap_thread_wa, sizeof(ap_thread_wa),
			NORMALPRIO, ap_thread, NULL);
}

void autopilot_add_point(ROUTE_POINT *p) {
	m_route[m_point_last++] = *p;

	if (m_point_last >= AP_ROUTE_SIZE) {
		m_point_last = 0;
	}

	if (!m_has_prev_point) {
		int p_last = m_point_now - 1;
		if (p_last < 0) {
			p_last = AP_ROUTE_SIZE - 1;
		}

		m_route[p_last] = *p;
		m_has_prev_point = true;
	}
}

void autopilot_remove_last_point(void) {
	if (m_point_last != m_point_now) {
		m_point_last --;
		if (m_point_last < 0) {
			m_point_last = AP_ROUTE_SIZE - 1;
		}
	}
}

void autopilot_clear_route(void) {
	m_is_active = false;
	m_has_prev_point = false;
	m_point_now = 0;
	m_point_last = 0;
}

void autopilot_set_active(bool active) {
	m_is_active = active;
}

bool autopilot_is_active(void) {
	return m_is_active;
}

/**
 * Override the speed with a fixed speed instead of using the value defined by
 * the route.
 *
 * @param is_override
 * True for override, false for using the route speed.
 *
 * @param speed
 * The speed to use. Ignored if is_override is false.
 */
void autopilot_set_speed_override(bool is_override, float speed) {
	m_is_speed_override = is_override;
	m_override_speed = speed;
}

/**
 * Set the motor speed.
 *
 * @param speed
 * Speed in m/s.
 */
void autopilot_set_motor_speed(float speed) {
	float rpm = speed / (main_config.gear_ratio
			* (2.0 / main_config.motor_poles) * (1.0 / 60.0)
			* main_config.wheel_diam * M_PI);
	bldc_interface_set_rpm((int)rpm);
}

static THD_FUNCTION(ap_thread, arg) {
	(void)arg;

	chRegSetThreadName("Autopilot");

	for(;;) {
		chThdSleep(CH_CFG_ST_FREQUENCY / AP_HZ);

		if (!m_is_active) {
			continue;
		}

		if (m_point_now == m_point_last) {
			// The end of the route is reached.
			servo_simple_set_pos_ramp(main_config.steering_center);
			bldc_interface_set_current_brake(10.0);
			continue;
		}

		float distance, steering_angle;
		float servo_pos;
		static int max_steering = 0;

		POS_STATE p;
		ROUTE_POINT rp_now, rp_prev;

		int p_last = m_point_now - 1;
		if (p_last < 0) {
			p_last = AP_ROUTE_SIZE - 1;
		}

		rp_now = m_route[m_point_now];
		rp_prev = m_route[p_last];
		pos_get_pos(&p);

		steering_angle_to_point(p.px, p.py, -p.yaw * M_PI / 180.0, rp_now.px,
				rp_now.py, &steering_angle, &distance);

		// Scale maximum steering by speed
		const float div = 1.0 + fabsf(p.speed) * 0.1;
		float steering_scale = 1.0 / (div * div);
		float max_rad = main_config.steering_max_angle_rad * steering_scale;

		if (steering_angle >= max_rad) {
			steering_angle = max_rad;
			max_steering++;
		} else if (steering_angle <= -max_rad) {
			steering_angle = -max_rad;
			max_steering++;
		} else {
			max_steering = 0;
		}

		servo_pos = steering_angle / ((2.0 * main_config.steering_max_angle_rad)
				/ (main_config.steering_left - main_config.steering_right))
											+ main_config.steering_center;

		const float dist_previous = utils_point_distance(p.px, p.py, rp_prev.px, rp_prev.py);
		const float fract_prev = distance / (dist_previous + distance);

		float speed;
		if (m_is_speed_override) {
			speed = m_override_speed;
		} else {
			speed = (rp_prev.speed * fract_prev) + (rp_now.speed * (1.0 - fract_prev));
		}

		servo_simple_set_pos_ramp(servo_pos);
		autopilot_set_motor_speed(speed);

		// Check it it is time to go to the next point
		if (distance < 0.3 || (max_steering > 25 && distance < 3.0)) {
			max_steering = 0;

			m_point_now++;
			if (m_point_now >= AP_ROUTE_SIZE) {
				m_point_now = 0;
			}
		}
	}
}

static void steering_angle_to_point(
		float current_x,
		float current_y,
		float current_angle,
		float goal_x,
		float goal_y,
		float *steering_angle,
		float *distance) {

	const float D = utils_point_distance(goal_x, goal_y, current_x, current_y);
	*distance = D;
	const float gamma = current_angle - atan2f((goal_y-current_y), (goal_x-current_x));
	const float dx = D * cosf(gamma);
	const float dy = D * sinf(gamma);

	if (dy == 0.0) {
		*steering_angle = 0.0;
		return;
	}

	float circle_radius = -(dx * dx + dy * dy) / (2.0 * dy);

	/*
	 * Add correction if the arc is much longer than the total distance.
	 * TODO: Find a good model.
	 */
	float angle_correction = 1.0 + D * 0.5;
	if (angle_correction > 5.0) {
		angle_correction = 5.0;
	}

	*steering_angle = atanf(main_config.axis_distance / circle_radius) * angle_correction;
}
