/*
	Copyright 2018 Benjamin Vedder	benjamin@vedder.se

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

#ifndef POS_UWB_H_
#define POS_UWB_H_

#include "conf_general.h"
#include "ch.h"
#include "hal.h"

void pos_uwb_init(void);
void pos_uwb_update_dr(float imu_yaw, float travel_dist,
		float steering_angle, float speed);
void pos_uwb_add_anchor(UWB_ANCHOR a);
void pos_uwb_clear_anchors(void);
void pos_uwb_get_pos(POS_STATE *p);
void pos_uwb_set_xya(float x, float y, float angle);

#endif /* POS_UWB_H_ */
