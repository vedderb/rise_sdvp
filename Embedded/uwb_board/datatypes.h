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

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>
#include <stdbool.h>

// For double precision literals
#define D(x) 						((double)x##L)
#define D_PI						D(3.14159265358979323846)

// Sizes
#define LOG_NAME_MAX_LEN			20

// Constants
#define MS_PER_DAY					(24 * 60 * 60 * 1000)

// CAN ID mask for DecaWave module
#define CAN_MASK_DW					(5 << 8)
#define CAN_DW_ID_ANY				255

// CAN messages
typedef enum {
	CMD_DW_RANGE = 0,
	CMD_DW_PING,
	CMD_DW_REBOOT,
	CMD_DW_UPTIME
} CMD_DW;

// Orientation data
typedef struct {
	float q0;
	float q1;
	float q2;
	float q3;
	float integralFBx;
	float integralFBy;
	float integralFBz;
	float accMagP;
	int initialUpdateDone;
} ATTITUDE_INFO;

// Car configuration
typedef struct {
	// Magnetometer calibration
	float mag_cal_cx;
	float mag_cal_cy;
	float mag_cal_cz;
	float mag_cal_xx;
	float mag_cal_xy;
	float mag_cal_xz;
	float mag_cal_yx;
	float mag_cal_yy;
	float mag_cal_yz;
	float mag_cal_zx;
	float mag_cal_zy;
	float mag_cal_zz;
} MAIN_CONFIG;

#endif /* DATATYPES_H_ */
