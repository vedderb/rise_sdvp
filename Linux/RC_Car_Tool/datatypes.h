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

// Sizes
#define LOG_NAME_MAX_LEN			20

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
    FAULT_CODE_NONE = 0,
    FAULT_CODE_OVER_VOLTAGE,
    FAULT_CODE_UNDER_VOLTAGE,
    FAULT_CODE_DRV8302,
    FAULT_CODE_ABS_OVER_CURRENT,
    FAULT_CODE_OVER_TEMP_FET,
    FAULT_CODE_OVER_TEMP_MOTOR
} mc_fault_code;

typedef struct {
    uint8_t fw_major;
    uint8_t fw_minor;
    double roll;
    double pitch;
    double yaw;
    double accel[3];
    double gyro[3];
    double mag[3];
    double px;
    double py;
    double speed;
    double vin;
    double temp_fet;
    mc_fault_code mc_fault;
    double px_gps;
    double py_gps;
    double ap_goal_px;
    double ap_goal_py;
    double ap_rad;
} CAR_STATE;

typedef enum {
	MOTE_PACKET_FILL_RX_BUFFER = 0,
	MOTE_PACKET_FILL_RX_BUFFER_LONG,
	MOTE_PACKET_PROCESS_RX_BUFFER,
	MOTE_PACKET_PROCESS_SHORT_BUFFER,
} MOTE_PACKET;

// Car configuration
typedef struct {
    // Settings
    bool mag_comp; // Should be 0 when capturing samples for the calibration
    float yaw_imu_gain;

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

    // Car parameters
    float gear_ratio;
    float wheel_diam;
    float motor_poles;
    float steering_max_angle_rad; // = arctan(axist_distance / turn_radius_at_maximum_steering_angle)
    float steering_center;
    float steering_range;
    float steering_ramp_time; // Ramp time constant for the steering servo in seconds.
    float axis_distance;

    // GPS parameters
    float gps_ant_x; // Antenna offset from vehicle center in X
    float gps_ant_y; // Antenna offset from vehicle center in Y
    bool gps_comp; // Use GPS position correction
    float gps_corr_gain_stat; // Static GPS correction gain
    float gps_corr_gain_dyn; // Dynamic GPS correction gain
    float gps_corr_gain_yaw; // Gain for yaw correction

    // Autopilot parameters
    bool ap_repeat_routes; // Repeat the same route when the end is reached
    float ap_base_rad; // Radius around car at 0 speed

    // Logging
    bool log_en;
    char log_name[LOG_NAME_MAX_LEN + 1];
} MAIN_CONFIG;

typedef enum {
    CMD_PRINTF = 0,
    CMD_GET_STATE,
    CMD_TERMINAL_CMD,
    CMD_VESC_FWD,
    CMD_RC_CONTROL,
    CMD_SET_POS,
    CMD_SET_POS_ACK,
    CMD_SET_ENU_REF,
    CMD_GET_ENU_REF,
    CMD_AP_ADD_POINTS,
    CMD_AP_REMOVE_LAST_POINT,
    CMD_AP_CLEAR_POINTS,
    CMD_AP_SET_ACTIVE,
    CMD_SET_SERVO_DIRECT,
    CMD_SEND_RTCM_USB,
    CMD_SEND_NMEA_RADIO,
    CMD_SET_MAIN_CONFIG,
    CMD_GET_MAIN_CONFIG,
    CMD_GET_MAIN_CONFIG_DEFAULT,
    CMD_SET_YAW_OFFSET,
    CMD_SET_YAW_OFFSET_ACK,
    CMD_LOG_LINE_USB
} CMD_PACKET;

// RC control modes
typedef enum {
    RC_MODE_CURRENT = 0,
    RC_MODE_DUTY,
    RC_MODE_PID,
    RC_MODE_CURRENT_BRAKE
} RC_MODE;

typedef enum {
    JS_TYPE_HK = 0,
    JS_TYPE_PS4
} JS_TYPE;

#endif /* DATATYPES_H_ */
