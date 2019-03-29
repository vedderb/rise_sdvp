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

#ifndef AUTOPILOT_H_
#define AUTOPILOT_H_

#include "conf_general.h"

// Functions
void autopilot_init(void);
bool autopilot_add_point(ROUTE_POINT *p, bool first);
void autopilot_remove_last_point(void);
void autopilot_clear_route(void);
bool autopilot_replace_route(ROUTE_POINT *p);
void autopilot_sync_point(int32_t point, int32_t time, int32_t min_time_diff);
void autopilot_set_active(bool active);
bool autopilot_is_active(void);
int autopilot_get_route_len(void);
int autopilot_get_point_now(void);
int autopilot_get_route_left(void);
ROUTE_POINT autopilot_get_route_point(int ind);
void autopilot_set_speed_override(bool is_override, float speed);
void autopilot_set_motor_speed(float speed);
float autopilot_get_steering_scale(void);
float autopilot_get_rad_now(void);
void autopilot_get_goal_now(ROUTE_POINT *rp);

#if HAS_DIFF_STEERING
void autopilot_set_turn_rad(float rad);
#endif

#endif /* AUTOPILOT_H_ */
