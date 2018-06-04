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
#define FW_VERSION_MAJOR			9
#define FW_VERSION_MINOR			0

// Default car settings
//#define CAR_TERO // Benjamins tero car
//#define EBIKE_BENJAMIN // Benjamins ebike

// Defaults for different cars
#ifdef CAR_TERO
#define BOARD_YAW_ROT				180.0
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
//#define LOG_EN_DW
#define LOG_EN_SW
#define LOG_EN_CORR

// Anchor IDs to range against in DW logging mode. -1 to disable.
#define LOG_DW_ANCHOR0				122
#define LOG_DW_ANCHOR1				234
#define LOG_DW_ANCHOR2				35
#define LOG_DW_FORCE_CC1120			1

// CC2520 Settings
#define CC2520_RF_CHANNEL			12
#define CC2520_PAN_ID				0xfa11
#define CC2520_NODE_ADDRESS			0x001
#define CC2520_DEST_ADDRESS			0xffff // 0xffff = broadcast

// General settings
#define ID_ALL						255
#define VESC_ID						ID_ALL // id, or ID_ALL for any VESC
#define ID_MOTE						254 // If the packet is for the mote and not to be forwarded in mote mode

// Car parameters
#ifndef BOARD_YAW_ROT
#define BOARD_YAW_ROT				-90.0
#endif

// Radar settings
#ifndef RADAR_EN
#define RADAR_EN					0
#endif
#define RADAR_CENTER_FREQ			76.5e9
#define RADAR_FREQ_SPAN				1.0e9
#define RADAR_FREQ_PONTS			1024
#define RADAR_SWEEP_TIME			0.075
#define RADAR_MAP_PLOT_AVG_FACTOR	3.0
#define RADAR_MAP_PLOT_MAX_DIV		6.0
#define RADAR_PLOT_MODE				0
#define RADAR_MAP_PLOT_START		20
#define RADAR_MAP_PLOT_END			300

// radar_cont
#define RADAR_CONT_EN				0

// Servo settings
#define SERVO_OUT_RATE_HZ			50
#define SERVO_OUT_PULSE_MIN_US		1000
#define SERVO_OUT_PULSE_MAX_US		2000

// Autopilot settings
#define AP_ROUTE_SIZE				500

// Global variables
extern MAIN_CONFIG main_config;
extern int main_id;

// Functions
void conf_general_init(void);
void conf_general_get_default_main_config(MAIN_CONFIG *conf);
void conf_general_read_main_conf(MAIN_CONFIG *conf);
bool conf_general_store_main_config(MAIN_CONFIG *conf);

#endif /* CONF_GENERAL_H_ */
