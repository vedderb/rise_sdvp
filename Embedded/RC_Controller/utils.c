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

#include "utils.h"
#include "ch.h"
#include "hal.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// Private variables
static volatile int sys_lock_cnt = 0;

// Private functions
static double nmea_parse_val(char *str);

void utils_step_towards(float *value, float goal, float step) {
	if (*value < goal) {
		if ((*value + step) < goal) {
			*value += step;
		} else {
			*value = goal;
		}
	} else if (*value > goal) {
		if ((*value - step) > goal) {
			*value -= step;
		} else {
			*value = goal;
		}
	}
}

float utils_calc_ratio(float low, float high, float val) {
	return (val - low) / (high - low);
}

/**
 * Make sure that -180 <= angle < 180
 *
 * @param angle
 * The angle to normalize in degrees.
 * WARNING: Don't use too large angles.
 */
void utils_norm_angle(float *angle) {
	while (*angle < -180.0) {
		*angle += 360.0;
	}

	while (*angle >  180.0) {
		*angle -= 360.0;
	}
}

/**
 * Make sure that 0 <= angle < 360
 *
 * @param angle
 * The angle to normalize in degrees.
 * WARNING: Don't use too large angles.
 */
void utils_norm_angle_360(float *angle) {
	while (*angle < 0.0) {
		*angle += 360.0;
	}

	while (*angle >  360.0) {
		*angle -= 360.0;
	}
}

/**
 * Make sure that -pi <= angle < pi,
 *
 * TODO: Maybe use fmodf instead?
 *
 * @param angle
 * The angle to normalize in radians.
 * WARNING: Don't use too large angles.
 */
void utils_norm_angle_rad(float *angle) {
	while (*angle < -M_PI) {
		*angle += 2.0 * M_PI;
	}

	while (*angle >  M_PI) {
		*angle -= 2.0 * M_PI;
	}
}

int utils_truncate_number(float *number, float min, float max) {
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < min) {
		*number = min;
		did_trunc = 1;
	}

	return did_trunc;
}

int utils_truncate_number_abs(float *number, float max) {
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < -max) {
		*number = -max;
		did_trunc = 1;
	}

	return did_trunc;
}

float utils_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int utils_map_int(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Truncate absolute values less than tres to zero. The value
 * tres will be mapped to 0 and the value max to max.
 */
void utils_deadband(float *value, float tres, float max) {
	if (fabsf(*value) < tres) {
		*value = 0.0;
	} else {
		float k = max / (max - tres);
		if (*value > 0.0) {
			*value = k * *value + max * (1.0 - k);
		} else {
			*value = -(k * -*value + max * (1.0 - k));
		}

	}
}

/**
 * Get the difference between two angles. Will always be between -180 and +180 degrees.
 * @param angle1
 * The first angle
 * @param angle2
 * The second angle
 * @return
 * The difference between the angles
 */
float utils_angle_difference(float angle1, float angle2) {
	//	utils_norm_angle(&angle1);
	//	utils_norm_angle(&angle2);
	//
	//	if (fabsf(angle1 - angle2) > 180.0) {
	//		if (angle1 < angle2) {
	//			angle1 += 360.0;
	//		} else {
	//			angle2 += 360.0;
	//		}
	//	}
	//
	//	return angle1 - angle2;

	// Faster in most cases
	float difference = angle1 - angle2;
	while (difference < -180.0) difference += 2.0 * 180.0;
	while (difference > 180.0) difference -= 2.0 * 180.0;
	return difference;
}

/**
 * Get the difference between two angles. Will always be between -pi and +pi radians.
 * @param angle1
 * The first angle in radians
 * @param angle2
 * The second angle in radians
 * @return
 * The difference between the angles in radians
 */
float utils_angle_difference_rad(float angle1, float angle2) {
	float difference = angle1 - angle2;
	while (difference < -M_PI) difference += 2.0 * M_PI;
	while (difference > M_PI) difference -= 2.0 * M_PI;
	return difference;
}

/**
 * Run a "complementary filter" on two angles
 * @param angle1
 * The first angle
 * @param angle2
 * The second angle
 * @param ratio
 * The ratio between the first and second angle.
 * @return
 */
float utils_weight_angle(float angle1, float angle2, float ratio) {
	utils_norm_angle(&angle1);
	utils_norm_angle(&angle2);
	utils_truncate_number(&ratio, 0.0, 1.0);

	if (fabsf(angle1 - angle2) > 180.0) {
		if (angle1 < angle2) {
			angle1 += 360.0;
		} else {
			angle2 += 360.0;
		}
	}

	float angle_weighted = angle1 * ratio + angle2 * (1 - ratio);
	utils_norm_angle(&angle_weighted);

	return angle_weighted;
}

/**
 * Takes the average of a number of angles.
 *
 * @param angles
 * The angles in radians.
 *
 * @param angles_num
 * The number of angles.
 *
 * @param weights
 * The weight of the summarized angles
 *
 * @return
 * The average angle.
 */
float utils_avg_angles_rad_fast(float *angles, float *weights, int angles_num) {
	float s_sum = 0.0;
	float c_sum = 0.0;

	for (int i = 0; i < angles_num; i++) {
		float s, c;
		utils_fast_sincos_better(angles[i], &s, &c);
		s_sum += s * weights[i];
		c_sum += c * weights[i];
	}

	return utils_fast_atan2(s_sum, c_sum);
}

/**
 * Get the middle value of three values
 *
 * @param a
 * First value
 *
 * @param b
 * Second value
 *
 * @param c
 * Third value
 *
 * @return
 * The middle value
 */
float utils_middle_of_3(float a, float b, float c) {
	float middle;

	if ((a <= b) && (a <= c)) {
		middle = (b <= c) ? b : c;
	} else if ((b <= a) && (b <= c)) {
		middle = (a <= c) ? a : c;
	} else {
		middle = (a <= b) ? a : b;
	}
	return middle;
}

/**
 * Get the middle value of three values
 *
 * @param a
 * First value
 *
 * @param b
 * Second value
 *
 * @param c
 * Third value
 *
 * @return
 * The middle value
 */
int utils_middle_of_3_int(int a, int b, int c) {
	int middle;

	if ((a <= b) && (a <= c)) {
		middle = (b <= c) ? b : c;
	} else if ((b <= a) && (b <= c)) {
		middle = (a <= c) ? a : c;
	} else {
		middle = (a <= b) ? a : b;
	}
	return middle;
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float utils_fast_inv_sqrt(float x) {
	union {
		float as_float;
		long as_int;
	} un;

	float xhalf = 0.5f*x;
	un.as_float = x;
	un.as_int = 0x5f3759df - (un.as_int >> 1);
	un.as_float = un.as_float * (1.5f - xhalf * un.as_float * un.as_float);
	return un.as_float;
}

/**
 * Fast atan2
 *
 * See http://www.dspguru.com/dsp/tricks/fixed-point-atan2-with-self-normalization
 *
 * @param y
 * y
 *
 * @param x
 * x
 *
 * @return
 * The angle in radians
 */
float utils_fast_atan2(float y, float x) {
	float abs_y = fabsf(y) + 1e-10; // kludge to prevent 0/0 condition
	float angle;

	if (x >= 0) {
		float r = (x - abs_y) / (x + abs_y);
		float rsq = r * r;
		angle = ((0.1963 * rsq) - 0.9817) * r + (M_PI / 4.0);
	} else {
		float r = (x + abs_y) / (abs_y - x);
		float rsq = r * r;
		angle = ((0.1963 * rsq) - 0.9817) * r + (3.0 * M_PI / 4.0);
	}

	if (y < 0) {
		return(-angle);
	} else {
		return(angle);
	}
}

/**
 * Truncate the magnitude of a vector.
 *
 * @param x
 * The first component.
 *
 * @param y
 * The second component.
 *
 * @param max
 * The maximum magnitude.
 *
 * @return
 * True if saturation happened, false otherwise
 */
bool utils_saturate_vector_2d(float *x, float *y, float max) {
	bool retval = false;
	float mag = sqrtf(*x * *x + *y * *y);
	max = fabsf(max);

	if (mag < 1e-10) {
		mag = 1e-10;
	}

	if (mag > max) {
		const float f = max / mag;
		*x *= f;
		*y *= f;
		retval = true;
	}

	return retval;
}

/**
 * Fast sine and cosine implementation.
 *
 * See http://lab.polygonal.de/?p=205
 *
 * @param angle
 * The angle in radians
 * WARNING: Don't use too large angles.
 *
 * @param sin
 * A pointer to store the sine value.
 *
 * @param cos
 * A pointer to store the cosine value.
 */
void utils_fast_sincos(float angle, float *sin, float *cos) {
	//always wrap input angle to -PI..PI
	while (angle < -M_PI) {
		angle += 2.0 * M_PI;
	}

	while (angle >  M_PI) {
		angle -= 2.0 * M_PI;
	}

	// compute sine
	if (angle < 0.0) {
		*sin = 1.27323954 * angle + 0.405284735 * angle * angle;
	} else {
		*sin = 1.27323954 * angle - 0.405284735 * angle * angle;
	}

	// compute cosine: sin(x + PI/2) = cos(x)
	angle += 0.5 * M_PI;

	if (angle >  M_PI) {
		angle -= 2.0 * M_PI;
	}

	if (angle < 0.0) {
		*cos = 1.27323954 * angle + 0.405284735 * angle * angle;
	} else {
		*cos = 1.27323954 * angle - 0.405284735 * angle * angle;
	}
}

/**
 * Fast sine and cosine implementation.
 *
 * See http://lab.polygonal.de/?p=205
 *
 * @param angle
 * The angle in radians
 * WARNING: Don't use too large angles.
 *
 * @param sin
 * A pointer to store the sine value.
 *
 * @param cos
 * A pointer to store the cosine value.
 */
void utils_fast_sincos_better(float angle, float *sin, float *cos) {
	//always wrap input angle to -PI..PI
	while (angle < -M_PI) {
		angle += 2.0 * M_PI;
	}

	while (angle >  M_PI) {
		angle -= 2.0 * M_PI;
	}

	//compute sine
	if (angle < 0.0) {
		*sin = 1.27323954 * angle + 0.405284735 * angle * angle;

		if (*sin < 0.0) {
			*sin = 0.225 * (*sin * -*sin - *sin) + *sin;
		} else {
			*sin = 0.225 * (*sin * *sin - *sin) + *sin;
		}
	} else {
		*sin = 1.27323954 * angle - 0.405284735 * angle * angle;

		if (*sin < 0.0) {
			*sin = 0.225 * (*sin * -*sin - *sin) + *sin;
		} else {
			*sin = 0.225 * (*sin * *sin - *sin) + *sin;
		}
	}

	// compute cosine: sin(x + PI/2) = cos(x)
	angle += 0.5 * M_PI;
	if (angle >  M_PI) {
		angle -= 2.0 * M_PI;
	}

	if (angle < 0.0) {
		*cos = 1.27323954 * angle + 0.405284735 * angle * angle;

		if (*cos < 0.0) {
			*cos = 0.225 * (*cos * -*cos - *cos) + *cos;
		} else {
			*cos = 0.225 * (*cos * *cos - *cos) + *cos;
		}
	} else {
		*cos = 1.27323954 * angle - 0.405284735 * angle * angle;

		if (*cos < 0.0) {
			*cos = 0.225 * (*cos * -*cos - *cos) + *cos;
		} else {
			*cos = 0.225 * (*cos * *cos - *cos) + *cos;
		}
	}
}

/**
 * Calculate the distance between two points.
 *
 * @param x1
 * Point 1 x coordinate.
 *
 * @param y1
 * Point 1 y coordinate.
 *
 * @param x2
 * Point 2 x coordinate.
 *
 * @param y2
 * Point 2 y coordinate.
 *
 * @return
 * The distance between the points.
 */
float utils_point_distance(float x1, float y1, float x2, float y2) {
	float dx = x2 - x1;
	float dy = y2 - y1;
	return sqrtf(dx * dx + dy * dy);
}

/**
 * Calculate the distance between two route points.
 *
 * @param p1
 * The first route point
 *
 * @param p2
 * The second route point
 *
 * @return
 * The distance between the route points
 */
float utils_rp_distance(const ROUTE_POINT *p1, const ROUTE_POINT *p2) {
	float dx = p2->px - p1->px;
	float dy = p2->py - p1->py;
	return sqrtf(dx * dx + dy * dy);
}

/**
 * Calculate the intersection point(s) between a circle and a line segment.
 *
 * @param cx
 * Circle center x
 *
 * @param cy
 * Circle center y
 *
 * @param rad
 * Circle radius
 *
 * @param point1
 * Line segment start
 *
 * @param point2
 * Line segment end
 *
 * @param int1
 * First intersection
 *
 * @param int2
 * Second intersection
 *
 * @return
 * The number of intersections found, can be 0, 1 or 2
 *
 */
int utils_circle_line_int(float cx, float cy, float rad,
		const ROUTE_POINT *point1, const ROUTE_POINT *point2,
		ROUTE_POINT *int1, ROUTE_POINT *int2) {
	float dx, dy, a, b, c, det, t, x, y;

	const float p1x = point1->px;
	const float p1y = point1->py;
	const float p2x = point2->px;
	const float p2y = point2->py;

	float maxx = p1x;
	float minx = p2x;
	float maxy = p1y;
	float miny = p2y;

	if (maxx < minx) {
		maxx = p2x;
		minx = p1x;
	}

	if (maxy < miny) {
		maxy = p2y;
		miny = p1y;
	}

	dx = p2x - p1x;
	dy = p2y - p1y;

	a = dx * dx + dy * dy;
	b = 2 * (dx * (p1x - cx) + dy * (p1y - cy));
	c = (p1x - cx) * (p1x - cx) + (p1y - cy) * (p1y - cy) - rad * rad;

	det = b * b - 4 * a * c;

	int ints = 0;
	if ((a <= 1e-6) || (det < 0.0)) {
		// No real solutions.
	} else if (det == 0) {
		// One solution.
		t = -b / (2 * a);
		x = p1x + t * dx;
		y = p1y + t * dy;

		if (x >= minx && x <= maxx &&
				y >= miny && y <= maxy) {
			int1->px = x;
			int1->py = y;
			ints++;
		}
	} else {
		// Two solutions.
		t = (-b + sqrtf(det)) / (2 * a);
		x = p1x + t * dx;
		y = p1y + t * dy;

		if (x >= minx && x <= maxx &&
				y >= miny && y <= maxy) {
			int1->px = x;
			int1->py = y;
			ints++;
		}

		t = (-b - sqrtf(det)) / (2 * a);
		x = p1x + t * dx;
		y = p1y + t * dy;

		if (x >= minx && x <= maxx &&
				y >= miny && y <= maxy) {
			if (ints) {
				int2->px = x;
				int2->py = y;
			} else {
				int1->px = x;
				int1->py = y;
			}

			ints++;
		}
	}

	return ints;
}

/**
 * Calculate the closest point on a line segment to another point
 *
 * @param point1
 * Line segment start
 *
 * @param point2
 * Line segment end
 *
 * @param px
 * Point x
 *
 * @param py
 * Point y
 *
 * @param res
 * Closest point
 *
 */
void utils_closest_point_line(const ROUTE_POINT *point1, const ROUTE_POINT *point2,
		float px, float py, ROUTE_POINT *res) {
	const float p1x = point1->px;
	const float p1y = point1->py;
	const float p2x = point2->px;
	const float p2y = point2->py;

	const float d1x = px - p1x;
	const float d1y = py - p1y;
	const float dx = p2x - p1x;
	const float dy = p2y - p1y;

	const float ab2 = dx * dx + dy * dy;
	const float ap_ab = d1x * dx + d1y * dy;
	float t = ap_ab / ab2;

	if (t < 0.0) {
		t = 0.0;
	} else if (t > 1.0) {
		t = 1.0;
	}

	res->px = p1x + dx * t;
	res->py = p1y + dy * t;
}

void utils_llh_to_xyz(double lat, double lon, double height, double *x, double *y, double *z) {
	double sinp = sin(lat * D_PI / D(180.0));
	double cosp = cos(lat * D_PI / D(180.0));
	double sinl = sin(lon * D_PI / D(180.0));
	double cosl = cos(lon * D_PI / D(180.0));
	double e2 = FE_WGS84 * (D(2.0) - FE_WGS84);
	double v = RE_WGS84 / sqrt(D(1.0) - e2 * sinp * sinp);

	*x = (v + height) * cosp * cosl;
	*y = (v + height) * cosp * sinl;
	*z = (v * (D(1.0) - e2) + height) * sinp;
}

void utils_xyz_to_llh(double x, double y, double z, double *lat, double *lon, double *height) {
	double e2 = FE_WGS84 * (D(2.0) - FE_WGS84);
	double r2 = x * x + y * y;
	double za = z;
	double zk = 0.0;
	double sinp = 0.0;
	double v = RE_WGS84;

	while (fabs(za - zk) >= D(1E-4)) {
		zk = za;
		sinp = za / sqrt(r2 + za * za);
		v = RE_WGS84 / sqrt(D(1.0) - e2 * sinp * sinp);
		za = z + v * e2 * sinp;
	}

	*lat = (r2 > D(1E-12) ? atan(za / sqrt(r2)) : (z > D(0.0) ? D_PI / D(2.0) : -D_PI / D(2.0))) * D(180.0) / D_PI;
	*lon = (r2 > D(1E-12) ? atan2(y, x) : D(0.0)) * D(180.0) / D_PI;
	*height = sqrt(r2 + za * za) - v;
}

void utils_create_enu_matrix(double lat, double lon, double *enuMat) {
	double so = sin(lon * D_PI / D(180.0));
	double co = cos(lon * D_PI / D(180.0));
	double sa = sin(lat * D_PI / D(180.0));
	double ca = cos(lat * D_PI / D(180.0));

	// ENU
	enuMat[0] = -so;
	enuMat[1] = co;
	enuMat[2] = 0.0;

	enuMat[3] = -sa * co;
	enuMat[4] = -sa * so;
	enuMat[5] = ca;

	enuMat[6] = ca * co;
	enuMat[7] = ca * so;
	enuMat[8] = sa;
}

void utils_llh_to_enu(const double *iLlh, const double *llh, double *xyz) {
	double ix, iy, iz;
	utils_llh_to_xyz(iLlh[0], iLlh[1], iLlh[2], &ix, &iy, &iz);

	double x, y, z;
	utils_llh_to_xyz(llh[0], llh[1], llh[2], &x, &y, &z);

	double enuMat[9];
	utils_create_enu_matrix(iLlh[0], iLlh[1], enuMat);

	double dx = x - ix;
	double dy = y - iy;
	double dz = z - iz;

	xyz[0] = enuMat[0] * dx + enuMat[1] * dy + enuMat[2] * dz;
	xyz[1] = enuMat[3] * dx + enuMat[4] * dy + enuMat[5] * dz;
	xyz[2] = enuMat[6] * dx + enuMat[7] * dy + enuMat[8] * dz;
}

void utils_enu_to_llh(const double *iLlh, const double *xyz, double *llh) {
	double ix, iy, iz;
	utils_llh_to_xyz(iLlh[0], iLlh[1], iLlh[2], &ix, &iy, &iz);

	double enuMat[9];
	utils_create_enu_matrix(iLlh[0], iLlh[1], enuMat);

	double x = enuMat[0] * xyz[0] + enuMat[3] * xyz[1] + enuMat[6] * xyz[2] + ix;
	double y = enuMat[1] * xyz[0] + enuMat[4] * xyz[1] + enuMat[7] * xyz[2] + iy;
	double z = enuMat[2] * xyz[0] + enuMat[5] * xyz[1] + enuMat[8] * xyz[2] + iz;

	utils_xyz_to_llh(x, y, z, &llh[0], &llh[1], &llh[2]);
}

/**
 * Create string representation of the binary content of a byte
 *
 * @param x
 * The byte.
 *
 * @param b
 * Array to store the string representation in.
 */
void utils_byte_to_binary(int x, char *b) {
	b[0] = '\0';

	int z;
	for (z = 128; z > 0; z >>= 1) {
		strcat(b, ((x & z) == z) ? "1" : "0");
	}
}

/**
 * Check if t1 is before t2. Considers wrap-around at 24h.
 *
 * @param t1
 * Time t1, milliseconds today
 *
 * @param t2
 * Time t2, milliseconds today
 *
 * @return
 * true if t1 is before t2, false otherwise.
 */
bool utils_time_before(int32_t t1, int32_t t2) {
	if (t1 < t2 && (t2 - t1) < (MS_PER_DAY / 2)) {
		return true;
	} else {
		return false;
	}
}

/**
 * Convert milliseconds to hours, minutes, seconds
 *
 * @param ms
 * Milliseconds today
 *
 * @param hh
 * Pointer to store hours at
 *
 * @param mm
 * Pointer to store minutes at
 *
 * @param ss
 * Pointer to store seconds at
 */
void utils_ms_to_hhmmss(int ms, int *hh, int *mm, int *ss) {
	*ss = (int) (ms / 1000) % 60;
	*mm = (int) ((ms / (1000 * 60)) % 60);
	*hh   = (int) ((ms / (1000 * 60 * 60)) % 24);
}

/**
 * Decode NMEA GGA message.
 *
 * @param data
 * NMEA string.
 *
 * @param gga
 * GGA struct to fill.
 *
 * @return
 * -1: Type is not GGA
 * >= 0: Number of decoded fields.
 */
int utils_decode_nmea_gga(const char *data, nmea_gga_info_t *gga) {
	static char nmea_str[1024];
	int ms = -1;
	double lat = 0.0;
	double lon = 0.0;
	double height = 0.0;
	int fix_type = 0;
	int sats = 0;
	float hdop = 0.0;
	float diff_age = -1.0;

	int dec_fields = 0;

	bool found = false;
	int len = strlen(data);

	for (int i = 0;i < 10;i++) {
		if ((i + 5) >= len) {
			break;
		}

		if (    data[i] == 'G' &&
				data[i + 1] == 'G' &&
				data[i + 2] == 'A' &&
				data[i + 3] == ',') {
			found = true;
			strcpy(nmea_str, data + i + 4);
			break;
		}
	}

	if (found) {
		char *gga, *str;
		int ind = 0;

		str = nmea_str;
		gga = strsep(&str, ",");

		while (gga != 0) {
			switch (ind) {
			case 0: {
				// Time
				int h, m, s, ds;
				dec_fields++;

				if (sscanf(gga, "%02d%02d%02d.%d", &h, &m, &s, &ds) == 4) {
					ms = h * 60 * 60 * 1000;
					ms += m * 60 * 1000;
					ms += s * 1000;
					ms += ds * 10;
				} else {
					ms = -1;
				}
			} break;

			case 1: {
				// Latitude
				dec_fields++;
				lat = nmea_parse_val(gga);
			} break;

			case 2:
				// Latitude direction
				dec_fields++;
				if (*gga == 'S' || *gga == 's') {
					lat = -lat;
				}
				break;

			case 3: {
				// Longitude
				dec_fields++;
				lon = nmea_parse_val(gga);
			} break;

			case 4:
				// Longitude direction
				dec_fields++;
				if (*gga == 'W' || *gga == 'w') {
					lon = -lon;
				}
				break;

			case 5:
				// Fix type
				dec_fields++;
				if (sscanf(gga, "%d", &fix_type) != 1) {
					fix_type = 0;
				}
				break;

			case 6:
				// Satellites
				dec_fields++;
				if (sscanf(gga, "%d", &sats) != 1) {
					sats = 0;
				}
				break;

			case 7:
				// hdop
				dec_fields++;
				if (sscanf(gga, "%f", &hdop) != 1) {
					hdop = 0.0;
				}
				break;

			case 8:
				// Altitude
				dec_fields++;
				if (sscanf(gga, "%lf", &height) != 1) {
					height = 0.0;
				}
				break;

			case 10: {
				// Altitude 2
				double h2 = 0.0;
				dec_fields++;
				if (sscanf(gga, "%lf", &h2) != 1) {
					h2 = 0.0;
				}

				height += h2;
			} break;

			case 12: {
				// Correction age
				dec_fields++;
				if (sscanf(gga, "%f", &diff_age) != 1) {
					diff_age = -1.0;
				}
			} break;

			default:
				break;
			}

			gga = strsep(&str, ",");
			ind++;
		}
	} else {
		dec_fields = -1;
	}

	gga->lat = lat;
	gga->lon = lon;
	gga->height = height;
	gga->fix_type = fix_type;
	gga->n_sat = sats;
	gga->t_tow = ms;
	gga->h_dop = hdop;
	gga->diff_age = diff_age;

	return dec_fields;
}

/**
 * Decode NMEA GSV message.
 *
 * @param system_str
 * Satellite system string:
 * GP: GPS
 * GN: GLONASS
 * GA: GALILEO
 *
 * @param data
 * NMEA string.
 *
 * @param gsv_info
 * GSV struct to fill.
 *
 * @return
 * -2: Unknown error
 * -1: Wrong type (not gsv)
 * 0: Decode ok, waiting for more sentences
 * 1: All sentences decoded, data ready to be used
 */
int utils_decode_nmea_gsv(const char *system_str, const char *data, nmea_gsv_info_t *gsv_info) {
	int retval = -2;
	static char nmea_str[1024];

	bool found = false;
	int len = strlen(data);

	for (int i = 0;i < 10;i++) {
		if ((i + 7) >= len) {
			break;
		}

		if (    data[i] == system_str[0] &&
				data[i + 1] == system_str[1] &&
				data[i + 2] == 'G' &&
				data[i + 3] == 'S' &&
				data[i + 4] == 'V' &&
				data[i + 5] == ',') {
			found = true;
			strcpy(nmea_str, data + i + 6);
			break;
		}
	}

	if (found) {
		char *gsv, *str;
		int ind = 0;

		str = nmea_str;
		gsv = strsep(&str, ",");

		while (gsv != 0) {
			if (gsv[0] == '*') {
				break;
			}

			switch (ind) {
			case 0: {
				// Number of sentences
				if (sscanf(gsv, "%d", &(gsv_info->sentences)) != 1) {
					gsv_info->sentences = 0;
				}
			} break;

			case 1: {
				// Sentence now
				int sentence = 0;
				if (sscanf(gsv, "%d", &sentence) != 1) {
					sentence = 0;
				}

				if (sentence == 1) {
					gsv_info->sat_last = 0;
				}
			} break;

			case 2: {
				// Sats
				if (sscanf(gsv, "%d", &(gsv_info->sat_num)) != 1) {
					gsv_info->sat_num = 0;
				}
			} break;

			case 3: {
				// PRN
				if (gsv_info->sat_last < 32) {
					int prn = 0;
					sscanf(gsv, "%d", &prn);
					gsv_info->sats[gsv_info->sat_last].prn = prn;
				}
			} break;

			case 4: {
				// Elevation
				if (gsv_info->sat_last < 32) {
					float elev = 0.0;
					sscanf(gsv, "%f", &elev);
					gsv_info->sats[gsv_info->sat_last].elevation = elev;
				}
			} break;

			case 5: {
				// Azimuth
				if (gsv_info->sat_last < 32) {
					float azimuth = 0.0;
					sscanf(gsv, "%f", &azimuth);
					gsv_info->sats[gsv_info->sat_last].azimuth = azimuth;
				}
			} break;

			case 6: {
				// SNR
				if (gsv_info->sat_last < 32) {
					float snr = 0.0;
					sscanf(gsv, "%f", &snr);
					gsv_info->sats[gsv_info->sat_last].snr = snr;
					gsv_info->sat_last++;
					ind = 2;
				}
			} break;

			default:
				break;
			}

			gsv = strsep(&str, ",");
			ind++;
		}

		if (gsv_info->sat_last == gsv_info->sat_num) {
			retval = 1;
		} else {
			retval = 0;
		}
	} else {
		retval = -1;
	}

	return retval;
}

/**
 * Synchronize nmea gsv info structs.
 *
 * @param old_info
 * The info struct to update.
 *
 * @param new_info
 * The new info struct to take data from.
 */
void utils_sync_nmea_gsv_info(nmea_gsv_info_t *old_info, nmea_gsv_info_t *new_info) {
	for (int i = 0;i < new_info->sat_num;i++) {
		for (int j = 0;j < old_info->sat_num;j++) {
			if (new_info->sats[i].prn == old_info->sats[j].prn) {
				new_info->sats[i].base_lock = old_info->sats[j].base_lock;
				new_info->sats[i].base_snr = old_info->sats[j].base_snr;
				new_info->sats[i].local_lock = old_info->sats[j].local_lock;
			}
		}

		old_info->sats[i] = new_info->sats[i];
	}

	old_info->sentences = new_info->sentences;
	old_info->sat_num = new_info->sat_num;
	old_info->sat_last = new_info->sat_last;
}

/**
 * A system locking function with a counter. For every lock, a corresponding unlock must
 * exist to unlock the system. That means, if lock is called five times, unlock has to
 * be called five times as well. Note that chSysLock and chSysLockFromIsr are the same
 * for this port.
 */
void utils_sys_lock_cnt(void) {
	if (!sys_lock_cnt) {
		chSysLock();
	}
	sys_lock_cnt++;
}

/**
 * A system unlocking function with a counter. For every lock, a corresponding unlock must
 * exist to unlock the system. That means, if lock is called five times, unlock has to
 * be called five times as well. Note that chSysUnlock and chSysUnlockFromIsr are the same
 * for this port.
 */
void utils_sys_unlock_cnt(void) {
	if (sys_lock_cnt) {
		sys_lock_cnt--;
		if (!sys_lock_cnt) {
			chSysUnlock();
		}
	}
}

// Private functions
static double nmea_parse_val(char *str) {
	int ind = -1;
	int len = strlen(str);
	double retval = D(0.0);

	for (int i = 2;i < len;i++) {
		if (str[i] == '.') {
			ind = i - 2;
			break;
		}
	}

	if (ind >= 0) {
		char a[len + 1];
		memcpy(a, str, ind);
		a[ind] = ' ';
		memcpy(a + ind + 1, str + ind, len - ind);

		double l1, l2;
		if (sscanf(a, "%lf %lf", &l1, &l2) == 2) {
			retval = l1 + l2 / D(60.0);
		}
	}

	return retval;
}
