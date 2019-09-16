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

#ifndef CONF_GENERAL_H_
#define CONF_GENERAL_H_

#include "datatypes.h"

#define MAIN_MODE_CAR 				0
#define MAIN_MODE_MOTE_2400			1
#define MAIN_MODE_MOTE_400			2
#define MAIN_MODE_MOTE_HYBRID		3 // Use 400 for slow and critical communication and 2400 for the rest.
#define MAIN_MODE_MULTIROTOR		4
#define MAIN_MODE_M8T_BASE_2400		5
#define MAIN_MODE_M8T_BASE_400		6

// Main mode
#ifndef MAIN_MODE
#define MAIN_MODE					MAIN_MODE_CAR
#endif

// Mode macros
#define MAIN_MODE_IS_MOTE			(MAIN_MODE == MAIN_MODE_MOTE_2400 || MAIN_MODE == MAIN_MODE_MOTE_400 || MAIN_MODE == MAIN_MODE_MOTE_HYBRID)
#define MAIN_MODE_IS_VEHICLE		(MAIN_MODE == MAIN_MODE_CAR || MAIN_MODE == MAIN_MODE_MULTIROTOR)
#define MAIN_MODE_IS_BASE			(MAIN_MODE == MAIN_MODE_M8T_BASE_2400 || MAIN_MODE == MAIN_MODE_M8T_BASE_400)

// Firmware version
#define FW_VERSION_MAJOR			12
#define FW_VERSION_MINOR			1

// Default car settings
//#define CAR_TERO // Benjamins tero car
//#define EBIKE_BENJAMIN // Benjamins ebike

// Differential steering
#ifndef HAS_DIFF_STEERING
#define HAS_DIFF_STEERING			0
#endif
#ifndef DIFF_STEERING_VESC_LEFT
#define DIFF_STEERING_VESC_LEFT		0
#endif
#ifndef DIFF_STEERING_VESC_RIGHT
#define DIFF_STEERING_VESC_RIGHT	1
#endif

// Hydraulic drive
#ifndef HAS_HYDRAULIC_DRIVE
#define HAS_HYDRAULIC_DRIVE			0
#endif

// VESC for steering
// ID of VESC for steering.
// -1: No steering VESC
#ifndef SERVO_VESC_ID
#define SERVO_VESC_ID				-1
#endif
/*
 * Angle should be increasing from S1 to S2 (possibly
 * passing 0). The steering mapping is done on top
 * of S1 and S2.
 */
#ifndef SERVO_VESC_S1
#define SERVO_VESC_S1				331.0
#endif
#ifndef SERVO_VESC_S2
#define SERVO_VESC_S2				30.0
#endif
#ifndef SERVO_VESC_P_GAIN
#define SERVO_VESC_P_GAIN			2.0
#endif
#ifndef SERVO_VESC_I_GAIN
#define SERVO_VESC_I_GAIN			1.0
#endif
#ifndef SERVO_VESC_D_GAIN
#define SERVO_VESC_D_GAIN			0.1
#endif
#ifndef SERVO_VESC_D_FILTER
#define SERVO_VESC_D_FILTER			0.05
#endif
#ifndef SERVO_VESC_INVERTED
#define SERVO_VESC_INVERTED			0
#endif

// Ublox settings
#ifndef UBLOX_EN
#define UBLOX_EN					1
#endif
#ifndef UBLOX_USE_PPS
#define UBLOX_USE_PPS				1
#endif

// External PPS signal for accurate time synchronization and delay compensation on PD4.
#ifndef GPS_EXT_PPS
#define GPS_EXT_PPS					0
#endif

// CAN settings
#define CAN_EN_DW					1

// Log configuration to enable. Choose one only.
//#define LOG_EN_CARREL
//#define LOG_EN_ITRANSIT

// CC2520 Settings
#define CC2520_RF_CHANNEL			12
#define CC2520_PAN_ID				0xfa11
#define CC2520_NODE_ADDRESS			0x001
#define CC2520_DEST_ADDRESS			0xffff // 0xffff = broadcast

// General settings
#define ID_ALL						255
#define ID_CAR_CLIENT				254 // Packet for car client only
#ifndef VESC_ID
#define VESC_ID						ID_ALL // id, or ID_ALL for any VESC (not used in diff steering mode)
#endif
#define ID_MOTE						254 // If the packet is for the mote and not to be forwarded in mote mode

#ifdef CAR_TERO
#ifndef BOARD_YAW_ROT
#define BOARD_YAW_ROT				90.0
#endif
#endif

// Car parameters
#ifndef BOARD_YAW_ROT
#define BOARD_YAW_ROT				-90.0
#endif

// Servo settings
#define SERVO_OUT_RATE_HZ			50
#define SERVO_OUT_PULSE_MIN_US		1000
#define SERVO_OUT_PULSE_MAX_US		2000

// Autopilot settings
#define AP_ROUTE_SIZE				1000

// Global variables
extern MAIN_CONFIG main_config;
extern int main_id;

// Functions
void conf_general_init(void);
void conf_general_get_default_main_config(MAIN_CONFIG *conf);
void conf_general_read_main_conf(MAIN_CONFIG *conf);
bool conf_general_store_main_config(MAIN_CONFIG *conf);

#endif /* CONF_GENERAL_H_ */
