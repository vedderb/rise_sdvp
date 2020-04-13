/*
	Copyright 2020 Benjamin Vedder	benjamin@vedder.se

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

/*
 * SID: xxx xxxx xxxx
 *               BRID  Board ID
 *          MSG        Message
 *      MSK            Board Mask
 */

typedef enum {
	CAN_IO_PACKET_SET_VALVE = 0,
	CAN_IO_PACKET_SET_VALVES_ALL,
	CAN_IO_PACKET_SET_VALVE_PWM_DUTY,
	CAN_IO_PACKET_ADC_VOLTAGES_0_1_2_3,
	CAN_IO_PACKET_ADC_VOLTAGES_4_5_6_7,
	CAN_IO_PACKET_AS5047_ANGLE,
	CAN_IO_PACKET_LIM_SW,
	CAN_IO_PACKET_ADC0_HIGH_TIME,
	CAN_IO_PACKET_ADC0_LOW_TIME,
	CAN_IO_PACKET_ADC0_HIGH_LOW_CNT
} CAN_IO_PACKET;

typedef struct {
	float low_time_last;
	float high_time_last;
	float low_time_current;
	float high_time_current;
	uint32_t toggle_low_cnt;
	uint32_t toggle_high_cnt;
	bool is_high;
} ADC_CNT_t;

#endif /* DATATYPES_H_ */
