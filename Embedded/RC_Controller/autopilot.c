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
static ROUTE_POINT m_rp_now;
static float m_rad_now;

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
	memset(&m_rp_now, 0, sizeof(ROUTE_POINT));
	m_rad_now = -1.0;

	chThdCreateStatic(ap_thread_wa, sizeof(ap_thread_wa),
			NORMALPRIO, ap_thread, NULL);
}

void autopilot_add_point(ROUTE_POINT *p) {
	// Check if the same point is sent again.
	ROUTE_POINT *lp = &m_route[m_point_last];
	if (utils_point_distance(lp->px, lp->py, p->px, p->py) < 1e-4) {
		return;
	}

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

	// When repeating routes, the previous point for the first
	// point is the end point of the current route.
	if (main_config.ap_repeat_routes) {
		m_route[AP_ROUTE_SIZE - 1] = *p;
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

/**
 * Get steering scale factor based in the current speed.
 *
 * @return
 * Steering scale factor. 1.0 at low speed, decreasing at high speed.
 */
float autopilot_get_steering_scale(void) {
	const float div = 1.0 + fabsf(pos_get_speed()) * 0.1;
	return 1.0 / (div * div);
}

/**
 * Get the current radius for calculating the next goal point.
 *
 * @return
 * The radius in meters.
 */
float autopilot_get_rad_now(void) {
	return m_rad_now;
}

/**
 * Get the current goal point for the autopilot.
 *
 * @rp
 * Pointer to store the goal to.
 */
void autopilot_get_goal_now(ROUTE_POINT *rp) {
	*rp = m_rp_now;
}

static THD_FUNCTION(ap_thread, arg) {
	(void)arg;

	chRegSetThreadName("Autopilot");

	for(;;) {
		chThdSleep(CH_CFG_ST_FREQUENCY / AP_HZ);

		if (!m_is_active) {
			m_rad_now = -1.0;
			continue;
		}

		int len = m_point_last;
		if (m_point_now > m_point_last) {
			len = AP_ROUTE_SIZE + m_point_last - m_point_now;
		}

		if (len >= 2) {
			POS_STATE p;
			pos_get_pos(&p);

			int add = 5;
			if (add > len) {
				add = len;
			}

			int start = m_point_now;
			int end = m_point_now + add;

			float cx = p.px;
			float cy = p.py;

			// Speed-dependent radius
			m_rad_now = main_config.ap_base_rad / autopilot_get_steering_scale();

			ROUTE_POINT last;
			ROUTE_POINT *closest1 = &m_route[0];
			ROUTE_POINT *closest2 = &m_route[1];
			bool has = false;
			bool closest_set = false;
			int current = 0;
			for (int i = start;i < end;i++) {
				int ind = i;
				int indn = i + 1;

				if (ind >= m_point_last) {
					if (m_point_now <= m_point_last) {
						ind -= m_point_last;
					} else {
						if (ind >= AP_ROUTE_SIZE) {
							ind -= AP_ROUTE_SIZE;
						}
					}
				}

				if (indn >= m_point_last) {
					if (m_point_now <= m_point_last) {
						indn -= m_point_last;
					} else {
						if (indn >= AP_ROUTE_SIZE) {
							indn -= AP_ROUTE_SIZE;
						}
					}
				}

				ROUTE_POINT int1, int2;
				ROUTE_POINT *p1, *p2;
				p1 = &m_route[ind];
				p2 = &m_route[indn];

				if (!closest_set) {
					closest1 = p1;
					closest_set = true;
				}

				// Closest point
				if (utils_point_distance(cx, cy, p1->px, p1->py) <
						utils_point_distance(cx, cy, closest1->px, closest1->py)) {
					closest1 = p1;
				} else {
					// Make sure that closest2 is at least further away
					closest2 = p1;
				}

				int res = utils_circle_line_int(cx, cy, m_rad_now, p1, p2, &int1, &int2);

				if (!has && res > 0) {
					has = true;
					last = int1;
					current = i;
				}

				if (res == 1) {
					if (utils_rp_distance(p2, &int1) <
							utils_rp_distance(&last, p2)) {
						last = int1;
						current = i;
					}
				}

				if (res == 2) {
					if (utils_rp_distance(p2, &int2) <
							utils_rp_distance(&last, p2)) {
						last = int2;
						current = i;
					}
				}
			}

			// Next closest point
			for (int i = start;i < end;i++) {
				int ind = i;
				if (ind >= m_point_last) {
					if (m_point_now <= m_point_last) {
						ind -= m_point_last;
					} else {
						if (ind >= AP_ROUTE_SIZE) {
							ind -= AP_ROUTE_SIZE;
						}
					}
				}

				ROUTE_POINT *p1 = &m_route[ind];

				if (utils_point_distance(cx, cy, p1->px, p1->py) <
						utils_point_distance(cx, cy, closest2->px, closest2->py) &&
						p1 != closest1) {
					closest2 = p1;
				}
			}

			if (!has) {
				ROUTE_POINT carp;
				carp.px = cx;
				carp.py = cy;
				bool lastSet = false;

				for (int i = start;i < end;i++) {
					int ind = i;
					int indn = i + 1;

					if (ind >= m_point_last) {
						if (m_point_now <= m_point_last) {
							ind -= m_point_last;
						} else {
							if (ind >= AP_ROUTE_SIZE) {
								ind -= AP_ROUTE_SIZE;
							}
						}
					}

					if (indn >= m_point_last) {
						if (m_point_now <= m_point_last) {
							indn -= m_point_last;
						} else {
							if (indn >= AP_ROUTE_SIZE) {
								indn -= AP_ROUTE_SIZE;
							}
						}
					}

					ROUTE_POINT tmp;
					ROUTE_POINT *p1, *p2;
					p1 = &m_route[ind];
					p2 = &m_route[indn];
					utils_closest_point_line(p1, p2, cx, cy, &tmp);

					if (!lastSet || utils_rp_distance(&tmp, &carp) < utils_rp_distance(&last, &carp)) {
						last = tmp;
						lastSet = true;
						current = i;
					}
				}
			}

			if (current > (m_point_now + 1) && current > 0) {
				m_point_now = current -  1;

				if (m_point_now >= m_point_last && main_config.ap_repeat_routes) {
					m_point_now = 0;
				}
			}

			m_rp_now = last;

			float distance, steering_angle;
			float servo_pos;
			static int max_steering = 0;

			steering_angle_to_point(p.px, p.py, -p.yaw * M_PI / 180.0, last.px,
					last.py, &steering_angle, &distance);

			// Scale maximum steering by speed
			float max_rad = main_config.steering_max_angle_rad * autopilot_get_steering_scale();

			if (steering_angle >= max_rad) {
				steering_angle = max_rad;
				max_steering++;
			} else if (steering_angle <= -max_rad) {
				steering_angle = -max_rad;
				max_steering++;
			} else {
				max_steering = 0;
			}

			servo_pos = steering_angle
					/ ((2.0 * main_config.steering_max_angle_rad)
							/ main_config.steering_range)
							+ main_config.steering_center;

			const float dist_next = utils_point_distance(cx, cy, closest1->px, closest1->py);
			const float dist_previous = utils_point_distance(cx, cy, closest2->px, closest2->py);
			const float fract_prev = dist_next / (dist_previous + dist_next);

			float speed;
			if (m_is_speed_override) {
				speed = m_override_speed;
			} else {
				speed = (closest2->speed * fract_prev) + (closest1->speed * (1.0 - fract_prev));
			}

			servo_simple_set_pos_ramp(servo_pos);
			autopilot_set_motor_speed(speed);
		} else {
			servo_simple_set_pos_ramp(main_config.steering_center);
			bldc_interface_set_current_brake(10.0);
			m_rad_now = -1.0;
			continue;
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
	float angle_correction = 1.0 + D * 0.2;
	if (angle_correction > 5.0) {
		angle_correction = 5.0;
	}

	*steering_angle = atanf(main_config.axis_distance / circle_radius) * angle_correction;
}
