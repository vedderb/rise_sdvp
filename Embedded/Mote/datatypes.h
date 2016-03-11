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

#endif /* DATATYPES_H_ */
