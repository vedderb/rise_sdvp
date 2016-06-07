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
 * conf_general.c
 *
 *  Created on: 11 mars 2016
 *      Author: benjamin
 */

#include "conf_general.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "eeprom.h"
#include "utils.h"
#include <string.h>

// Settings
#define EEPROM_BASE_MAINCONF		1000

// Global variables
MAIN_CONFIG main_config;
int main_id;
uint16_t VirtAddVarTab[NB_OF_VAR];

void conf_general_init(void) {
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

	memset(VirtAddVarTab, 0, sizeof(VirtAddVarTab));

	for (unsigned int i = 0;i < (sizeof(MAIN_CONFIG) / 2);i++) {
		VirtAddVarTab[i] = EEPROM_BASE_MAINCONF + i;
	}

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	EE_Init();

	conf_general_read_main_conf(&main_config);
}

/**
 * Load the compiled default app_configuration.
 *
 * @param conf
 * A pointer to store the default configuration to.
 */
void conf_general_get_default_main_config(MAIN_CONFIG *conf) {
	conf->mag_comp = true;
	conf->yaw_imu_gain = 0.1;

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

	conf->gear_ratio = (1.0 / 3.0) * (21.0 / 37.0);
	conf->wheel_diam = 0.12;
	conf->motor_poles = 4.0;
	conf->steering_max_angle_rad = 0.42041;
	conf->steering_center = 0.5;
	conf->steering_range = 0.58;
	conf->steering_ramp_time = 0.6;
	conf->axis_distance = 0.475;

	conf->gps_ant_x = 0.42;
	conf->gps_ant_y = 0.0;
	conf->gps_comp = false;
	conf->gps_corr_gain_stat = 0.05;
	conf->gps_corr_gain_dyn = 0.05;
	conf->gps_corr_gain_yaw = 1.0;

	conf->ap_repeat_routes = true;
	conf->ap_base_rad = 1.2;

	// Custom parameters based on car ID
	switch (main_id) {
	case 0:
		conf->mag_cal_cx = 13.8906;
		conf->mag_cal_cy = 41.6703;
		conf->mag_cal_cz = -38.28;

		conf->mag_cal_xx = 0.813398;
		conf->mag_cal_xy = 0.0421102;
		conf->mag_cal_xz = 0.00674955;

		conf->mag_cal_yx = 0.0421102;
		conf->mag_cal_yy = 0.835037;
		conf->mag_cal_yz = -0.0759237;

		conf->mag_cal_zx = 0.00674955;
		conf->mag_cal_zy = -0.0759237;
		conf->mag_cal_zz = 0.964149;

		conf->steering_center = 0.53;
		break;

	default:
		break;
	}
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
