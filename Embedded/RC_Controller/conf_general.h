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
 * conf_general.h
 *
 *  Created on: 10 mars 2016
 *      Author: benjamin
 */

#ifndef CONF_GENERAL_H_
#define CONF_GENERAL_H_

#include "datatypes.h"

// CC2520 Settings
#define CC2520_RF_CHANNEL			12
#define CC2520_PAN_ID				0xfa11
#define CC2520_NODE_ADDRESS			0x001
#define CC2520_DEST_ADDRESS			0xffff // 0xffff = broadcast

// Software version
#define FW_VERSION_MAJOR			1
#define FW_VERSION_MINOR			0

// General settings
#define ID_ALL						255
#define MAG_COMPENSATE				1 // Should be 0 when capturing samples for the calibration
#define VESC_ID						ID_ALL // id, or ID_ALL for any VESC

// Servo settings
#define SERVO_OUT_RATE_HZ			50
#define SERVO_OUT_PULSE_MIN_US		1000
#define SERVO_OUT_PULSE_MAX_US		2000

// Global variables
extern MAIN_CONFIG main_config;

// Functions
void conf_general_init(void);

#endif /* CONF_GENERAL_H_ */
