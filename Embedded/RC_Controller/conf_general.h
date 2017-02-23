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

#ifndef CONF_GENERAL_H_
#define CONF_GENERAL_H_

#include "datatypes.h"

#define MAIN_MODE_CAR 				0
#define MAIN_MODE_MOTE_2400			1
#define MAIN_MODE_MOTE_400			2
#define MAIN_MODE_MOTE_HYBRID		3 // Use 400 for slow and critical communication and 2400 for the rest.
#define MAIN_MODE_QUADCOPTER		4

// Main mode
#define MAIN_MODE					MAIN_MODE_CAR

// Mode macros
#define MAIN_MODE_IS_MOTE			(MAIN_MODE == MAIN_MODE_MOTE_2400 || MAIN_MODE == MAIN_MODE_MOTE_400 || MAIN_MODE == MAIN_MODE_MOTE_HYBRID)

// Firmware version
#define FW_VERSION_MAJOR			8
#define FW_VERSION_MINOR			2

// Default car settings
//#define CAR_TERO // Benjamins tero car

// Defaults for different cars
#ifdef CAR_TERO
#define IMU_ROT_180					1
#endif

// Log settings
#define LOG_EN_CARREL
//#define LOG_EN_ITRANSIT
#define LOG_INTERVAL_MS				10

// CC2520 Settings
#define CC2520_RF_CHANNEL			12
#define CC2520_PAN_ID				0xfa11
#define CC2520_NODE_ADDRESS			0x001
#define CC2520_DEST_ADDRESS			0xffff // 0xffff = broadcast

// General settings
#define ID_ALL						255
#define VESC_ID						ID_ALL // id, or ID_ALL for any VESC
#define ID_MOTE						254 // If the packet is for the mote and not to be forwarded

// Car parameters
#ifndef IMU_ROT_180
#define IMU_ROT_180					0
#endif

// Radar settings
#define RADAR_EN					0
#define RADAR_CENTER_FREQ			76.5e9
#define RADAR_FREQ_SPAN				1.0e9
#define RADAR_FREQ_PONTS			1024
#define RADAR_SWEEP_TIME			0.075

// Ublox settings
#define UBLOX_EN					1

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
