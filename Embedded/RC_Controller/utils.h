/*
	Copyright 2012 - 2018 Benjamin Vedder	benjamin@vedder.se

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

#ifndef UTILS_H_
#define UTILS_H_

#include <stdbool.h>
#include <string.h>
#include "conf_general.h"

void utils_step_towards(float *value, float goal, float step);
float utils_calc_ratio(float low, float high, float val);
void utils_norm_angle(float *angle);
void utils_norm_angle_360(float *angle);
void utils_norm_angle_rad(float *angle);
int utils_truncate_number(float *number, float min, float max);
int utils_truncate_number_abs(float *number, float max);
float utils_map(float x, float in_min, float in_max, float out_min, float out_max);
int utils_map_int(int x, int in_min, int in_max, int out_min, int out_max);
void utils_deadband(float *value, float tres, float max);
float utils_angle_difference(float angle1, float angle2);
float utils_angle_difference_rad(float angle1, float angle2);
float utils_weight_angle(float angle1, float angle2, float ratio);
float utils_avg_angles_rad_fast(float *angles, float *weights, int angles_num);
float utils_middle_of_3(float a, float b, float c);
int utils_middle_of_3_int(int a, int b, int c);
float utils_fast_inv_sqrt(float x);
float utils_fast_atan2(float y, float x);
bool utils_saturate_vector_2d(float *x, float *y, float max);
void utils_fast_sincos(float angle, float *sin, float *cos);
void utils_fast_sincos_better(float angle, float *sin, float *cos);
float utils_point_distance(float x1, float y1, float x2, float y2);
float utils_rp_distance(const ROUTE_POINT *p1, const ROUTE_POINT *p2);
int utils_circle_line_int(float cx, float cy, float rad,
		const ROUTE_POINT *point1, const ROUTE_POINT *point2,
		ROUTE_POINT *int1, ROUTE_POINT *int2);
void utils_closest_point_line(const ROUTE_POINT *point1, const ROUTE_POINT *point2,
		float px, float py, ROUTE_POINT *res);
void utils_llh_to_xyz(double lat, double lon, double height, double *x, double *y, double *z);
void utils_xyz_to_llh(double x, double y, double z, double *lat, double *lon, double *height);
void utils_create_enu_matrix(double lat, double lon, double *enuMat);
void utils_llh_to_enu(const double *iLlh, const double *llh, double *xyz);
void utils_enu_to_llh(const double *iLlh, const double *xyz, double *llh);
void utils_byte_to_binary(int x, char *b);
bool utils_time_before(int32_t t1, int32_t t2);
void utils_ms_to_hhmmss(int ms, int *hh, int *mm, int *ss);
int utils_decode_nmea_gga(const char *data, nmea_gga_info_t *gga);
int utils_decode_nmea_gsv(const char *system_str, const char *data, nmea_gsv_info_t *gsv_info);
void utils_sync_nmea_gsv_info(nmea_gsv_info_t *old_info, nmea_gsv_info_t *new_info);
void utils_sys_lock_cnt(void);
void utils_sys_unlock_cnt(void);

// Return the sign of the argument. -1 if negative, 1 if zero or positive.
#define SIGN(x)				((x < 0) ? -1 : 1)

// Minimum of two values
#define MIN(a, b)			((a < b) ? a : b)

// Squared
#define SQ(x)				((x) * (x))

// Return the age of a timestamp in seconds
#define UTILS_AGE_S(x)		((float)chVTTimeElapsedSinceX(x) / (float)CH_CFG_ST_FREQUENCY)

// nan and infinity check for floats
#define UTILS_IS_INF(x)		((x) == (1.0 / 0.0) || (x) == (-1.0 / 0.0))
#define UTILS_IS_NAN(x)		((x) != (x))

/**
 * A simple low pass filter.
 *
 * @param value
 * The filtered value.
 *
 * @param sample
 * Next sample.
 *
 * @param filter_constant
 * Filter constant. Range 0.0 to 1.0, where 1.0 gives the unfiltered value.
 */
#define UTILS_LP_FAST(value, sample, filter_constant)	(value -= (filter_constant) * (value - (sample)))

// Constants
#define FE_WGS84						(D(1.0)/D(298.257223563)) // earth flattening (WGS84)
#define RE_WGS84						D(6378137.0)           // earth semimajor axis (WGS84) (m)

#endif /* UTILS_H_ */
