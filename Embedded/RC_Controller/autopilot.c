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

#include "autopilot.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "ch.h"
#include "hal.h"
#include "servo_simple.h"
#include "utils.h"
#include "pos.h"
#include "bldc_interface.h"
#include "commands.h"
#include "terminal.h"
#include "comm_can.h"

// Defines
#define AP_HZ						100 // Hz

// Private variables
static THD_WORKING_AREA(ap_thread_wa, 2048);
static ROUTE_POINT m_route[AP_ROUTE_SIZE];
static bool m_is_active;
static int m_point_last; // The last point on the route
static int m_point_now; // The first point in the currently considered part of the route
static bool m_has_prev_point;
static float m_override_speed;
static bool m_is_speed_override;
static ROUTE_POINT m_rp_now; // The point in space we are following now
static float m_rad_now;
static ROUTE_POINT m_point_rx_prev;
static bool m_point_rx_prev_set;
static mutex_t m_ap_lock;
static int32_t m_start_time;
static bool m_sync_rx;
static int m_print_closest_point;
static bool m_en_dynamic_rad;
static bool m_en_angle_dist_comp;
static int m_route_look_ahead;
static int m_route_left;

#if HAS_DIFF_STEERING
static float m_turn_rad_now;
#endif

// Private functions
static THD_FUNCTION(ap_thread, arg);
static void steering_angle_to_point(
		float current_x,
		float current_y,
		float current_angle,
		float goal_x,
		float goal_y,
		float *steering_angle,
		float *distance,
		float *circle_radius);
static bool add_point(ROUTE_POINT *p, bool first);
static void clear_route(void);
static void terminal_state(int argc, const char **argv);
static void terminal_print_closest(int argc, const char **argv);
static void terminal_dynamic_rad(int argc, const char **argv);
static void terminal_angle_dist_comp(int argc, const char **argv);
static void terminal_look_ahead(int argc, const char **argv);

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
	memset(&m_point_rx_prev, 0, sizeof(ROUTE_POINT));
	m_point_rx_prev_set = false;
	chMtxObjectInit(&m_ap_lock);
	m_start_time = 0;
	m_sync_rx = false;
	m_print_closest_point = false;
	m_en_dynamic_rad = true;
	m_en_angle_dist_comp = true;
	m_route_look_ahead = 8;
	m_route_left = 0;

#if HAS_DIFF_STEERING
	m_turn_rad_now = 1e6;
#endif

	terminal_register_command_callback(
			"ap_state",
			"Print the state of the autopilot",
			"",
			terminal_state);

	terminal_register_command_callback(
			"ap_print_closest",
			"Print and plot distance to closest route point.\n"
			"  0 - Disabled\n"
			"  n - Print closest point every n:th iteration.",
			"[print_rate]",
			terminal_print_closest);

	terminal_register_command_callback(
			"ap_dynamic_rad",
			"Enable or disable dynamic radius.\n"
			"  0 - Disabled\n"
			"  1 - Enabled",
			"[enabled]",
			terminal_dynamic_rad);

	terminal_register_command_callback(
			"ap_ang_dist_comp",
			"Enable or disable steering angle distance compensation.\n"
			"  0 - Disabled\n"
			"  1 - Enabled",
			"[enabled]",
			terminal_angle_dist_comp);

	terminal_register_command_callback(
			"ap_set_look_ahead",
			"Set the look-ahead distance along the route in points",
			"[points]",
			terminal_look_ahead);

	chThdCreateStatic(ap_thread_wa, sizeof(ap_thread_wa),
			NORMALPRIO, ap_thread, NULL);
}

/**
 * Add a point to the route
 *
 * @param p
 * The point to add
 *
 * @param first
 * True if this is the first point in the packet. Used to check for duplicate packets.
 *
 * @return
 * True if the point was added, false otherwise.
 */
bool autopilot_add_point(ROUTE_POINT *p, bool first) {
	chMtxLock(&m_ap_lock);

	bool res = add_point(p, first);

	chMtxUnlock(&m_ap_lock);

	return res;
}

void autopilot_remove_last_point(void) {
	chMtxLock(&m_ap_lock);

	if (m_point_last != m_point_now) {
		m_point_last--;
		if (m_point_last < 0) {
			m_point_last = AP_ROUTE_SIZE - 1;
		}
	}

	chMtxUnlock(&m_ap_lock);
}

void autopilot_clear_route(void) {
	chMtxLock(&m_ap_lock);

	clear_route();

	chMtxUnlock(&m_ap_lock);
}

bool autopilot_replace_route(ROUTE_POINT *p) {
	bool ret = false;

	chMtxLock(&m_ap_lock);

	if (!m_is_active) {
		clear_route();
		add_point(p, true);
		ret = true;
	} else {
		bool time_mode = p->time > 0;

		while (m_point_last != m_point_now) {
			m_point_last--;
			if (m_point_last < 0) {
				m_point_last = AP_ROUTE_SIZE - 1;
			}

			// If we use time stamps (times are > 0), only overwrite the newer
			// part of the route.
			if (time_mode && p->time >= m_route[m_point_last].time) {
				break;
			}
		}

		m_has_prev_point = m_point_last != m_point_now;

		// In time mode, only add the point if its timestamp was ahead of the point
		// we currently follow.
		if (!time_mode || m_point_last != m_point_now) {
			add_point(p, true);
			ret = true;
		}
	}

	chMtxUnlock(&m_ap_lock);

	return ret;
}

void autopilot_sync_point(int32_t point, int32_t time, int32_t min_time_diff) {
	chMtxLock(&m_ap_lock);

	// Only update run when the autopilot is active.
//	if (!m_is_active) {
//		chMtxUnlock(&m_ap_lock);
//		return;
//	}

	int start = m_point_now + 1;
	if (start >= AP_ROUTE_SIZE) {
		start = 0;
	}

	if (start == m_point_last) {
		chMtxUnlock(&m_ap_lock);
		return;
	}

	POS_STATE p;
	pos_get_pos(&p);

	// Car center
	const float car_cx = p.px;
	const float car_cy = p.py;
	ROUTE_POINT car_pos;
	car_pos.px = car_cx;
	car_pos.py = car_cy;

	int point_i = start;
	int point_prev = 0;
	float dist_tot = 0.0;

	// Calculate remaining length
	for (;;) {
		if (point_i == start) {
			dist_tot += utils_rp_distance(&car_pos, &m_route[point_i]);
		} else {
			dist_tot += utils_rp_distance(&m_route[point_prev], &m_route[point_i]);
		}

		if (point_i == (m_point_last - 1) || point_i == point) {
			break;
		}

		point_prev = point_i;
		point_i++;
		if (point_i >= AP_ROUTE_SIZE) {
			point_i = 0;
		}
	}

	float speed = dist_tot / ((float)time / 1000.0);
	utils_truncate_number_abs(&speed, main_config.ap_max_speed);

	if (time < min_time_diff || dist_tot < main_config.ap_base_rad) {
//		m_sync_rx = false;
		chMtxUnlock(&m_ap_lock);
		return;
	}

	point_i = m_point_now;
	while (point_i <= point && point_i != m_point_last) {
		m_route[point_i].speed = speed;

		point_i++;
		if (point_i >= AP_ROUTE_SIZE) {
			point_i = 0;
		}
	}

	m_sync_rx = true;

	chMtxUnlock(&m_ap_lock);
}

void autopilot_set_active(bool active) {
	chMtxLock(&m_ap_lock);

	if (active && !m_is_active) {
		m_start_time = pos_get_ms_today();
//		m_sync_rx = false;
	}

	m_is_active = active;

	chMtxUnlock(&m_ap_lock);
}

bool autopilot_is_active(void) {
	return m_is_active;
}

int autopilot_get_route_len(void) {
	return m_point_last;
}

int autopilot_get_point_now(void) {
	return m_point_now;
}

int autopilot_get_route_left(void) {
	return m_route_left;
}

ROUTE_POINT autopilot_get_route_point(int ind) {
	ROUTE_POINT res;
	memset(&res, 0, sizeof(ROUTE_POINT));

	if (ind < m_point_last) {
		res = m_route[ind];
	}

	return res;
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
	if (!main_config.car.disable_motor) {
#if HAS_DIFF_STEERING
		float diff_speed_half = 0.0;

		if (fabsf(m_turn_rad_now) > 0.1) {
			diff_speed_half = speed * (main_config.car.axis_distance / (2.0 * m_turn_rad_now));
		}

		float rpm_r = (speed + diff_speed_half) / (main_config.car.gear_ratio
				* (2.0 / main_config.car.motor_poles) * (1.0 / 60.0)
				* main_config.car.wheel_diam * M_PI);

		float rpm_l = (speed - diff_speed_half) / (main_config.car.gear_ratio
				* (2.0 / main_config.car.motor_poles) * (1.0 / 60.0)
				* main_config.car.wheel_diam * M_PI);

		comm_can_lock_vesc();
		comm_can_set_vesc_id(DIFF_STEERING_VESC_LEFT);
		bldc_interface_set_rpm((int)rpm_l);
		comm_can_set_vesc_id(DIFF_STEERING_VESC_RIGHT);
		bldc_interface_set_rpm((int)rpm_r);
		comm_can_unlock_vesc();
#else
		float rpm = speed / (main_config.car.gear_ratio
					* (2.0 / main_config.car.motor_poles) * (1.0 / 60.0)
					* main_config.car.wheel_diam * M_PI);

		bldc_interface_set_rpm((int)rpm);
#endif
	}
}

/**
 * Get steering scale factor based in the current speed.
 *
 * @return
 * Steering scale factor. 1.0 at low speed, decreasing at high speed.
 */
float autopilot_get_steering_scale(void) {
	const float div = 1.0 + fabsf(pos_get_speed()) * 0.05;
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

#if HAS_DIFF_STEERING
void autopilot_set_turn_rad(float rad) {
	m_turn_rad_now = rad;
}
#endif

static THD_FUNCTION(ap_thread, arg) {
	(void)arg;

	chRegSetThreadName("Autopilot");

	for(;;) {
		chThdSleep(CH_CFG_ST_FREQUENCY / AP_HZ);

		bool route_end = false;

		chMtxLock(&m_ap_lock);

		if (!m_is_active) {
			m_rad_now = -1.0;
			chMtxUnlock(&m_ap_lock);
			continue;
		}

		// the length of the route that is left
		int len = m_point_last;

		// This means that the route has wrapped around
		// (should only happen when ap_repeat_routes == false)
		if (m_point_now > m_point_last) {
			len = AP_ROUTE_SIZE + m_point_last - m_point_now;
		}

		m_route_left = len - m_point_now;
		if (m_route_left < 0) {
			m_route_left += AP_ROUTE_SIZE;
		}

		// Time of today according to our clock
		int ms_today = pos_get_ms_today();

		if (len >= 2) {
			POS_STATE pos_now;
			pos_get_pos(&pos_now);

			// Car center
			const float car_cx = pos_now.px;
			const float car_cy = pos_now.py;
			ROUTE_POINT car_pos;
			car_pos.px = car_cx;
			car_pos.py = car_cy;

			// Look m_route_look_ahead points ahead, or less than that if the route is shorter
			int add = m_route_look_ahead;
			if (add > len) {
				add = len;
			}

			int start = m_point_now;
			int end = m_point_now + add;

			// Speed-dependent radius
			m_rad_now = main_config.ap_base_rad /
					(m_en_dynamic_rad ? autopilot_get_steering_scale() : 1.0);

			ROUTE_POINT rp_now; // The point we should follow now.
			int circle_intersections = 0;
			bool last_point_reached = false;

			// Last point in route
			int last_point_ind = m_point_last - 1;
			if (last_point_ind < 0) {
				last_point_ind += AP_ROUTE_SIZE;
			}

			ROUTE_POINT *rp_last = &m_route[last_point_ind]; // Last point on route
			ROUTE_POINT *rp_ls1 = &m_route[0]; // First point on goal line segment
			ROUTE_POINT *rp_ls2 = &m_route[1]; // Second point on goal line segment

			ROUTE_POINT *closest1_speed = &m_route[0];
			ROUTE_POINT *closest2_speed = &m_route[1];

			for (int i = start;i < end;i++) {
				int ind = i; // First point index for this iteration
				int indn = i + 1; // Next point index for this iteration

				// Wrap around
				if (ind >= m_point_last) {
					if (m_point_now <= m_point_last) {
						ind -= m_point_last;
					} else {
						if (ind >= AP_ROUTE_SIZE) {
							ind -= AP_ROUTE_SIZE;
						}
					}
				}

				// Wrap around
				if (indn >= m_point_last) {
					if (m_point_now <= m_point_last) {
						indn -= m_point_last;
					} else {
						if (indn >= AP_ROUTE_SIZE) {
							indn -= AP_ROUTE_SIZE;
						}
					}
				}

				// Check for circle intersection. If there are many intersections
				// found in this loop, the last one will be used.
				ROUTE_POINT int1, int2;
				ROUTE_POINT *p1, *p2;
				p1 = &m_route[ind];
				p2 = &m_route[indn];

				// If the next point has a time before the current point and repeat route is
				// active we have completed a full route. Increase its time by the repetition time.
				if (main_config.ap_repeat_routes && utils_time_before(p2->time, p1->time)) {
					p2->time += main_config.ap_time_add_repeat_ms;
					if (p2->time > MS_PER_DAY) {
						p2->time -= MS_PER_DAY;
					}
				}

				int res = utils_circle_line_int(car_cx, car_cy, m_rad_now, p1, p2, &int1, &int2);

				if (res) {
					closest1_speed = p1;
					closest2_speed = p2;
				}

				// One intersection. Use it.
				if (res == 1) {
					circle_intersections++;
					rp_now = int1;
				}

				// Two intersections. Use the point with the most "progress" on the route.
				if (res == 2) {
					circle_intersections += 2;

					if (utils_rp_distance(&int1, p2) < utils_rp_distance(&int2, p2)) {
						rp_now = int1;
					} else {
						rp_now = int2;
					}
				}

				if (res > 0) {
					rp_ls1 = &m_route[ind];
					rp_ls2 = &m_route[indn];
				}

				// If we aren't repeating routes and there is an intersection on the last
				// line segment, go straight to the last point.
				if (!main_config.ap_repeat_routes) {
					if (indn == last_point_ind && circle_intersections > 0) {
						if (res > 0) {
							last_point_reached = true;
						}
						break;
					}
				}
			}

			// Look for closest points
			ROUTE_POINT closest; // Closest point on route to car
			ROUTE_POINT *closest1 = &m_route[0]; // Start of closest line segment
			ROUTE_POINT *closest2 = &m_route[1]; // End of closest line segment
			int closest1_ind = 0; // Index of the first closest point

			{
				bool closest_set = false;

				for (int i = start;i < end;i++) {
					int ind = i; // First point index for this iteration
					int indn = i + 1; // Next point index for this iteration

					// Wrap around
					if (ind >= m_point_last) {
						if (m_point_now <= m_point_last) {
							ind -= m_point_last;
						} else {
							if (ind >= AP_ROUTE_SIZE) {
								ind -= AP_ROUTE_SIZE;
							}
						}
					}

					// Wrap around
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
					utils_closest_point_line(p1, p2, car_cx, car_cy, &tmp);

					if (!closest_set || utils_rp_distance(&tmp, &car_pos) < utils_rp_distance(&closest, &car_pos)) {
						closest_set = true;
						closest = tmp;
						closest1 = p1;
						closest2 = p2;
						closest1_ind = ind;
					}

					// Do not look past the last point if we aren't repeating routes.
					if (!main_config.ap_repeat_routes) {
						if (indn == last_point_ind) {
							break;
						}
					}
				}
			}

			if (circle_intersections == 0) {
				closest1_speed = closest1;
				closest2_speed = closest2;
			}

			static int sample = 0;
			static int print_before = 0;
			if (m_print_closest_point) {
				if (print_before == 0) {
					sample = 0;
				}

				if (sample % m_print_closest_point == 0) {
					float diff = utils_rp_distance(&closest, &car_pos) * 100.0;
					float speed = pos_now.speed * 3.6;

					commands_plot_set_graph(0);
					commands_send_plot_points((float)sample, diff);
					commands_plot_set_graph(1);
					commands_send_plot_points((float)sample, speed);
					commands_plot_set_graph(2);
					commands_send_plot_points((float)sample, pos_now.yaw * 0.1);
					commands_plot_set_graph(3);
					commands_plot_set_graph(3);
					commands_send_plot_points((float)sample, m_rad_now * 10.0);

					commands_printf("D: %.1f cm, S: %.2f km/h, Yaw: %.1f deg, Rad: %.2f m",
							(double)diff, (double)speed, (double)pos_now.yaw, (double)m_rad_now);
				}

				sample++;
			}
			print_before = m_print_closest_point;

			if (last_point_reached) {
				rp_now = *rp_last;
			} else {
				// Use the closest point on the considered route if no
				// circle intersection is found.
				if (circle_intersections == 0) {
					rp_now = closest;
					rp_ls1 = closest1;
					rp_ls2 = closest2;
				}
			}

			// Check if the end of route is reached
			if (!main_config.ap_repeat_routes && m_route_left < 3 &&
					utils_rp_distance(&m_route[last_point_ind], &car_pos) < m_rad_now) {
				route_end = true;
			}

			m_point_now = closest1_ind;
			m_rp_now = rp_now;

			if (!route_end) {
				float distance = 0.0;
				float steering_angle = 0.0;
				float circle_radius = 1000.0;

				steering_angle_to_point(pos_now.px, pos_now.py, -pos_now.yaw * M_PI / 180.0, rp_now.px,
						rp_now.py, &steering_angle, &distance, &circle_radius);

#if !HAS_DIFF_STEERING
				// Scale maximum steering by speed
				float max_rad = main_config.car.steering_max_angle_rad * autopilot_get_steering_scale();

				if (steering_angle >= max_rad) {
					steering_angle = max_rad;
				} else if (steering_angle <= -max_rad) {
					steering_angle = -max_rad;
				}

				float servo_pos = steering_angle
						/ ((2.0 * main_config.car.steering_max_angle_rad)
								/ main_config.car.steering_range)
								+ main_config.car.steering_center;
#endif

				float speed = 0.0;

				if (main_config.ap_mode_time && !m_sync_rx) {
					if (ms_today >= 0) {
						// Calculate speed such that the route points are reached at their
						// specified time. Notice that the direct distance between the car
						// and the points is used and not the arc that the car drives. This
						// should still work well enough.

						int32_t dist_prev = (int32_t)(utils_rp_distance(&rp_now, rp_ls1) * 1000.0);
						int32_t dist_tot = (int32_t)(utils_rp_distance(&rp_now, rp_ls1) * 1000.0);
						dist_tot += (int32_t)(utils_rp_distance(&rp_now, rp_ls2) * 1000.0);
						int32_t time = utils_map_int(dist_prev, 0, dist_tot, rp_ls1->time, rp_ls2->time);
						float dist_car = utils_rp_distance(&car_pos, &rp_now);

						int32_t t_diff = time - ms_today;

						if (main_config.ap_mode_time == 2) {
							t_diff += m_start_time;
						}

						if (t_diff < 0) {
							t_diff += 24 * 60 * 60 * 1000;
						}

						if (t_diff > 0) {
							speed = dist_car / ((float)t_diff / 1000.0);
						} else {
							speed = 0.0;
						}
					} else {
						speed = 0.0;
					}
				} else {
					// Calculate the speed based on the average speed between the two closest points
					const float dist_prev = utils_rp_distance(&rp_now, closest1_speed);
					const float dist_tot = utils_rp_distance(&rp_now, closest1_speed) + utils_rp_distance(&rp_now, closest2_speed);
					speed = utils_map(dist_prev, 0.0, dist_tot, closest1_speed->speed, closest2_speed->speed);
				}

				if (m_is_speed_override) {
					speed = m_override_speed;
				}

				utils_truncate_number_abs(&speed, main_config.ap_max_speed);

#if HAS_DIFF_STEERING
				autopilot_set_turn_rad(circle_radius);
#else
				servo_simple_set_pos_ramp(servo_pos);
#endif
				autopilot_set_motor_speed(speed);
			}
		} else {
			route_end = true;
		}

		if (route_end) {
			servo_simple_set_pos_ramp(main_config.car.steering_center);
			if (!main_config.car.disable_motor) {
				bldc_interface_set_current_brake(10.0);
			}
			m_rad_now = -1.0;
		}

		chMtxUnlock(&m_ap_lock);
	}
}

static void steering_angle_to_point(
		float current_x,
		float current_y,
		float current_angle,
		float goal_x,
		float goal_y,
		float *steering_angle,
		float *distance,
		float *circle_radius) {

	const float D = utils_point_distance(goal_x, goal_y, current_x, current_y);
	*distance = D;
	const float gamma = current_angle - atan2f((goal_y-current_y), (goal_x-current_x));
	const float dx = D * cosf(gamma);
	const float dy = D * sinf(gamma);

	if (fabsf(dy) <= 0.000001) {
		*steering_angle = 0.0;
		*circle_radius = 9000.0;
		return;
	}

	float R = -(dx * dx + dy * dy) / (2.0 * dy);

	/*
	 * Add correction if the arc is much longer than the total distance.
	 * TODO: Find a good model.
	 */
	float angle_correction = 1.0 + (m_en_angle_dist_comp ? D * 0.2 : 0.0);
	if (angle_correction > 5.0) {
		angle_correction = 5.0;
	}

	R /= angle_correction;

	*circle_radius = R;
	*steering_angle = atanf(main_config.car.axis_distance / R);
}

static bool add_point(ROUTE_POINT *p, bool first) {
	if (first && m_point_rx_prev_set &&
			utils_point_distance(m_point_rx_prev.px, m_point_rx_prev.py, p->px, p->py) < 1e-4) {
		return false;
	}

	if (first) {
		m_point_rx_prev = *p;
		m_point_rx_prev_set = true;
	}

	m_route[m_point_last++] = *p;

	if (m_point_last >= AP_ROUTE_SIZE) {
		m_point_last = 0;
	}

	// Make sure that there always is a valid point when looking backwards in the route
	if (!m_has_prev_point) {
		int p_last = m_point_now - 1;
		if (p_last < 0) {
			p_last += AP_ROUTE_SIZE;
		}

		m_route[p_last] = *p;
		m_has_prev_point = true;
	}

	// When repeating routes, the previous point for the first
	// point is the end point of the current route.
	if (main_config.ap_repeat_routes) {
		m_route[AP_ROUTE_SIZE - 1] = *p;
	}

	return true;
}

static void clear_route(void) {
	m_is_active = false;
	m_has_prev_point = false;
	m_point_now = 0;
	m_point_last = 0;
	m_point_rx_prev_set = false;
	m_start_time = pos_get_ms_today();
	m_sync_rx = false;
	memset(&m_rp_now, 0, sizeof(ROUTE_POINT));
	memset(&m_point_rx_prev, 0, sizeof(ROUTE_POINT));
}

static void terminal_state(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf(
			"m_is_active: %i\n"
			"m_has_prev_point: %i\n"
			"m_point_now: %i\n"
			"m_point_last: %i\n"
			"m_point_rx_prev_set: %i\n"
			"m_start_time: %i\n"
			"m_route_left: %i\n",

			m_is_active,
			m_has_prev_point,
			m_point_now,
			m_point_last,
			m_point_rx_prev_set,
			m_start_time,
			m_route_left);
}

static void terminal_print_closest(int argc, const char **argv) {
	if (argc == 2) {
		int n = -1;
		sscanf(argv[1], "%d", &n);

		if (n < 0) {
			commands_printf("Invalid argument\n");
		} else {
			if (n > 0) {
				commands_printf("OK. Printing closest point every %d iterations.\n", n);
				commands_init_plot("Sample", "Value");
				commands_plot_add_graph("Diff (cm)");
				commands_plot_add_graph("Speed (km/h)");
				commands_plot_add_graph("Yaw (0.1 degrees)");
				commands_plot_add_graph("Radius (0.1 m)");
			} else {
				commands_printf("OK. Not printing closest point.\n", n);
			}

			m_print_closest_point = n;
		}
	} else {
		commands_printf("Wrong number of arguments\n");
	}
}

static void terminal_dynamic_rad(int argc, const char **argv) {
	if (argc == 2) {
		if (strcmp(argv[1], "0") == 0) {
			m_en_dynamic_rad = 0;
			commands_printf("OK\n");
		} else if (strcmp(argv[1], "1") == 0) {
			m_en_dynamic_rad = 1;
			commands_printf("OK\n");
		} else {
			commands_printf("Invalid argument %s\n", argv[1]);
		}
	} else {
		commands_printf("Wrong number of arguments\n");
	}
}

static void terminal_angle_dist_comp(int argc, const char **argv) {
	if (argc == 2) {
		if (strcmp(argv[1], "0") == 0) {
			m_en_angle_dist_comp = 0;
			commands_printf("Angle distance compensation disabled\n");
		} else if (strcmp(argv[1], "1") == 0) {
			m_en_angle_dist_comp = 1;
			commands_printf("Angle distance compensation enabled\n");
		} else {
			commands_printf("Invalid argument %s\n", argv[1]);
		}
	} else {
		commands_printf("Wrong number of arguments\n");
	}
}

static void terminal_look_ahead(int argc, const char **argv) {
	if (argc == 2) {
		int n = -1;
		sscanf(argv[1], "%d", &n);

		if (n < 1) {
			commands_printf("Invalid argument\n");
		} else {
			m_route_look_ahead = n;
			commands_printf("Now looking %d points ahead along the route", n);
		}
	} else {
		commands_printf("Wrong number of arguments\n");
	}
}
