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

/*
 * datatypes.h
 *
 *  Created on: 10 mars 2016
 *      Author: benjamin
 */

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>
#include <stdbool.h>

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

typedef enum {
	MOTE_PACKET_FILL_RX_BUFFER = 0,
	MOTE_PACKET_FILL_RX_BUFFER_LONG,
	MOTE_PACKET_PROCESS_RX_BUFFER,
	MOTE_PACKET_PROCESS_SHORT_BUFFER,
} MOTE_PACKET;

typedef enum {
	COMM_PRINTF = 0,
	COMM_GET_IMU,
	COMM_TERMINAL_CMD
} COMM_PACKET_ID;

typedef struct {
	int id;
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
