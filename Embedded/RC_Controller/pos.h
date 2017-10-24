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

#ifndef POS_H_
#define POS_H_

#include "conf_general.h"
#include "ch.h"
#include "hal.h"

// Functions
void pos_init(void);
void pos_pps_cb(EXTDriver *extp, expchannel_t channel);
void pos_get_imu(float *accel, float *gyro, float *mag);
void pos_get_quaternions(float *q);
void pos_get_pos(POS_STATE *p);
void pos_get_gps(GPS_STATE *p);
float pos_get_speed(void);
void pos_set_xya(float x, float y, float angle);
void pos_set_yaw_offset(float angle);
void pos_set_enu_ref(double lat, double lon, double height);
void pos_get_enu_ref(double *llh);
void pos_reset_enu_ref(void);
void pos_get_mc_val(mc_values *v);
int32_t pos_get_ms_today(void);
void pos_set_ms_today(int32_t ms);
bool pos_input_nmea(const char *data);
void pos_reset_attitude(void);
int pos_time_since_gps_corr(void);

#endif /* POS_H_ */
