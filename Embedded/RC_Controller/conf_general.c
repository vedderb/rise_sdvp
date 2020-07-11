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

#include "conf_general.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "eeprom.h"
#include "utils.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "terminal.h"
#include "commands.h"

// Settings
#define EEPROM_BASE_MAINCONF		1000

// Global variables
MAIN_CONFIG main_config;
#if MAIN_MODE_IS_MOTE || MAIN_MODE_IS_BASE
int main_id = ID_MOTE;
#else
int main_id = 0;
#endif
uint16_t VirtAddVarTab[NB_OF_VAR];

// Private functions
static void terminal_cmd_set_id(int argc, const char **argv);
static void terminal_cmd_set_id_quiet(int argc, const char **argv);

void conf_general_init(void) {
#if HAS_ID_SW
	palSetPadMode(GPIOE, 8, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOE, 9, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOE, 10, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOE, 11, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOE, 12, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOE, 13, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOE, 14, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOE, 15, PAL_MODE_INPUT_PULLUP);

	chThdSleepMilliseconds(10);

	// Read address from switches
	main_id = (~(palReadPort(GPIOE) >> 8)) & 0x0F;
#else
	main_id = 0;
#endif

	memset(VirtAddVarTab, 0, sizeof(VirtAddVarTab));

	for (unsigned int i = 0;i < (sizeof(MAIN_CONFIG) / 2);i++) {
		VirtAddVarTab[i] = EEPROM_BASE_MAINCONF + i;
	}

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	EE_Init();

	conf_general_read_main_conf(&main_config);

	terminal_register_command_callback(
			"set_id",
			"Set the ID of the board.",
			"[id]",
			terminal_cmd_set_id);
	terminal_register_command_callback(
			"set_id_quiet",
			"Set the ID of the board, and print no reply.",
			"[id]",
			terminal_cmd_set_id_quiet);
}

/**
 * Load the compiled default app_configuration.
 *
 * @param conf
 * A pointer to store the default configuration to.
 */
void conf_general_get_default_main_config(MAIN_CONFIG *conf) {
	// Default settings
	conf->mag_use = false;
	conf->mag_comp = true;
	conf->yaw_mag_gain = 1.2;

	conf->mag_cal_cx = 0.0;
	conf->mag_cal_cy = 0.0;
	conf->mag_cal_cz = 0.0;
	conf->mag_cal_xx = 1.0;
	conf->mag_cal_xy = 0.0;
	conf->mag_cal_xz = 0.0;
	conf->mag_cal_yx = 0.0;
	conf->mag_cal_yy = 1.0;
	conf->mag_cal_yz = 0.0;
	conf->mag_cal_zx = 0.0;
	conf->mag_cal_zy = 0.0;
	conf->mag_cal_zz = 1.0;

	conf->gps_ant_x = 0.0;
	conf->gps_ant_y = 0.0;
	conf->gps_comp = true;
	conf->gps_req_rtk = true;
	conf->gps_use_rtcm_base_as_enu_ref = true;
	conf->gps_corr_gain_stat = 0.05;
	conf->gps_corr_gain_dyn = 0.05;
	conf->gps_corr_gain_yaw = 1.0;
	conf->gps_send_nmea = true;
	conf->gps_use_ubx_info = true;
	conf->gps_ubx_max_acc = 0.12;

	conf->uwb_max_corr = 0.1;

	conf->ap_repeat_routes = true;
	conf->ap_base_rad = 0.8;
	conf->ap_rad_time_ahead = 0.8;
	conf->ap_mode_time = false;
	conf->ap_max_speed = 30.0 / 3.6;
	conf->ap_time_add_repeat_ms = 60 * 1000;

	conf->log_rate_hz = 50;
	conf->log_en = false;
	strcpy(conf->log_name, "New Log");
	conf->log_mode_ext = 0;
	conf->log_uart_baud = 115200;

	// Default car settings
	conf->car.yaw_use_odometry = false;
	conf->car.yaw_imu_gain = 0.5;
	conf->car.disable_motor = false;
	conf->car.simulate_motor = false;
	conf->car.clamp_imu_yaw_stationary = true;
	conf->car.use_uwb_pos = false;

	conf->car.gear_ratio = (1.0 / 3.0) * (21.0 / 37.0);
	conf->car.wheel_diam = 0.11;
	conf->car.motor_poles = 4.0;
	conf->car.steering_max_angle_rad = 0.42041;
	conf->car.steering_center = 0.5;
	conf->car.steering_range = 0.58;
	conf->car.steering_ramp_time = 0.0;
	conf->car.axis_distance = 0.475;

	// Default multirotor settings
	conf->mr.vel_decay_e = 0.8;
	conf->mr.vel_decay_l = 0.02;
	conf->mr.vel_max = 80.0 / 3.6;

	conf->mr.map_min_x = -500.0;
	conf->mr.map_max_x = 500.0;
	conf->mr.map_min_y = -500.0;
	conf->mr.map_max_y = 500.0;

	conf->mr.vel_gain_p = 0.1;
	conf->mr.vel_gain_i = 0.0;
	conf->mr.vel_gain_d = 0.2;

	conf->mr.tilt_gain_p = 0.2;
	conf->mr.tilt_gain_i = 0.0;
	conf->mr.tilt_gain_d = 0.05;

	conf->mr.max_corr_error = 0.5;
	conf->mr.max_tilt_error = 6.0;

	conf->mr.ctrl_gain_roll_p = 0.8;
	conf->mr.ctrl_gain_roll_i = 1.0;
	conf->mr.ctrl_gain_roll_dp = 0.3;
	conf->mr.ctrl_gain_roll_de = 0.2;

	conf->mr.ctrl_gain_pitch_p = 0.8;
	conf->mr.ctrl_gain_pitch_i = 1.0;
	conf->mr.ctrl_gain_pitch_dp = 0.3;
	conf->mr.ctrl_gain_pitch_de = 0.2;

	conf->mr.ctrl_gain_yaw_p = 3.0;
	conf->mr.ctrl_gain_yaw_i = 0.2;
	conf->mr.ctrl_gain_yaw_dp = 0.4;
	conf->mr.ctrl_gain_yaw_de = 0.2;

	conf->mr.ctrl_gain_pos_p = 0.8;
	conf->mr.ctrl_gain_pos_i = 0.09;
	conf->mr.ctrl_gain_pos_d = 0.6;

	conf->mr.ctrl_gain_alt_p = 0.1;
	conf->mr.ctrl_gain_alt_i = 0.1;
	conf->mr.ctrl_gain_alt_d = 0.14;

	conf->mr.js_gain_tilt = 1.0;
	conf->mr.js_gain_yaw = 0.6;
	conf->mr.js_mode_rate = false;

	conf->mr.motor_fl_f = 0;
	conf->mr.motor_bl_l = 1;
	conf->mr.motor_fr_r = 2;
	conf->mr.motor_br_b = 3;
	conf->mr.motors_x = true;
	conf->mr.motors_cw = true;
	conf->mr.motor_pwm_min_us = 1200;
	conf->mr.motor_pwm_max_us = 2000;

	// Custom parameters based on ID
	switch (main_id) {
	case 1:
		conf->car.steering_center = 0.5;
		conf->gps_ant_x = 0.42;
		break;

	default:
		break;
	}

#ifdef CAR_TERO
	conf->car.gear_ratio = 0.1;
	conf->car.wheel_diam = 0.155;
	conf->car.motor_poles = 4.0;
	conf->car.steering_max_angle_rad = 0.39952;
	conf->car.steering_center = 0.45;
	conf->car.steering_range = -0.8;
	conf->car.steering_ramp_time = 0.6;
	conf->car.axis_distance = 0.57;

	conf->gps_ant_x = -0.08; // TODO: IMU_ROT_180 sign?
	conf->gps_ant_y = 0.0;
#endif

#ifdef EBIKE_BENJAMIN
	conf->car.disable_motor = true;
	conf->car.gear_ratio = 1.0;
	conf->car.wheel_diam = 0.655;
	conf->car.motor_poles = 46.0;

	conf->gps_ant_x = -0.1;
	conf->gps_ant_y = 0.0;

	conf->mag_cal_cx = 7.1270;
	conf->mag_cal_cy = -209.5598;
	conf->mag_cal_cz = 106.1795;

	conf->mag_cal_xx = 0.8868;
	conf->mag_cal_xy = 0.0028;
	conf->mag_cal_xz = -0.0046;

	conf->mag_cal_yx = 0.0028;
	conf->mag_cal_yy = 0.7761;
	conf->mag_cal_yz = 0.0539;

	conf->mag_cal_zx = -0.0046;
	conf->mag_cal_zy = 0.0539;
	conf->mag_cal_zz = 0.9869;
#endif

	// Only the SLU testbot for now
#if HAS_DIFF_STEERING
	conf->car.gear_ratio = 1.0;
	conf->car.axis_distance = 0.5;
	conf->car.wheel_diam = 0.3;
	conf->car.motor_poles = 22.0;
	conf->gps_ant_x = 0.5;
#endif

#ifdef IS_DRANGEN
	conf->car.steering_center = 0.5;
	conf->car.steering_range = -0.9;
	conf->car.axis_distance = 1.0;
	conf->car.steering_max_angle_rad = atanf(conf->car.axis_distance / 1.5);
#endif

#ifdef IS_MACTRAC
	conf->car.steering_center = 0.5;
	conf->car.steering_range = -1.0;
	conf->car.axis_distance = 1.7;
	conf->car.steering_max_angle_rad = atanf(conf->car.axis_distance / 1.5);
	conf->gps_corr_gain_yaw = 2.0;
	conf->ap_base_rad = 4.0;
	conf->ap_repeat_routes = false;

	conf->gps_ant_x = 1.25;
	conf->gps_ant_y = -0.3;

	conf->log_en = true;
	conf->log_mode_ext = LOG_EXT_ETHERNET;
	conf->log_rate_hz = 10;
#endif
}

/**
 * Read MAIN_CONFIG from EEPROM. If this fails, default values will be used.
 *
 * @param conf
 * A pointer to a MAIN_CONFIG struct to write the configuration to.
 */
void conf_general_read_main_conf(MAIN_CONFIG *conf) {
	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;
	uint16_t var;

	for (unsigned int i = 0;i < (sizeof(MAIN_CONFIG) / 2);i++) {
		if (EE_ReadVariable(EEPROM_BASE_MAINCONF + i, &var) == 0) {
			conf_addr[2 * i] = (var >> 8) & 0xFF;
			conf_addr[2 * i + 1] = var & 0xFF;
		} else {
			is_ok = false;
			break;
		}
	}

	// Set the default configuration
	if (!is_ok) {
		conf_general_get_default_main_config(conf);
	}
}

/**
 * Write MAIN_CONFIG to EEPROM.
 *
 * @param conf
 * A pointer to the configuration that should be stored.
 */
bool conf_general_store_main_config(MAIN_CONFIG *conf) {
	utils_sys_lock_cnt();
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE);

	bool is_ok = true;
	uint8_t *conf_addr = (uint8_t*)conf;
	uint16_t var;

	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	for (unsigned int i = 0;i < (sizeof(MAIN_CONFIG) / 2);i++) {
		var = (conf_addr[2 * i] << 8) & 0xFF00;
		var |= conf_addr[2 * i + 1] & 0xFF;

		if (EE_WriteVariable(EEPROM_BASE_MAINCONF + i, var) != FLASH_COMPLETE) {
			is_ok = false;
			break;
		}
	}

//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
	utils_sys_unlock_cnt();

	return is_ok;
}

static void terminal_cmd_set_id(int argc, const char **argv) {
	if (argc == 2) {
		int set = -1;
		sscanf(argv[1], "%d", &set);

		if (set < 0 || set > 254) {
			commands_printf("Invalid argument\n");
		} else {
			main_id = set;
			commands_printf("ID is now %d\n", main_id);
		}
	} else {
		commands_printf("Wrong number of arguments\n");
	}
}

static void terminal_cmd_set_id_quiet(int argc, const char **argv) {
	if (argc == 2) {
		int set = -1;
		sscanf(argv[1], "%d", &set);

		if (set < 0 || set > 254) {
			// Wrong id
		} else {
			main_id = set;
		}
	}
}
