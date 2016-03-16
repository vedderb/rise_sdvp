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

// Global variables
MAIN_CONFIG main_config;

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

	main_config.id = (~(palReadPort(GPIOE) >> 8)) & 0x0F;

	// Magnetometer compensation
	switch (main_config.id) {
	case 0:
		main_config.mag_cal_cx = 6.67419;
		main_config.mag_cal_cy = -6.24658;
		main_config.mag_cal_cz = 5.05975;

		main_config.mag_cal_xx = 0.934036;
		main_config.mag_cal_xy = -0.00158248;
		main_config.mag_cal_xz = 0.00402214;

		main_config.mag_cal_yx = -0.00158248;
		main_config.mag_cal_yy = 0.949697;
		main_config.mag_cal_yz = -0.00586774;

		main_config.mag_cal_zx = 0.00402214;
		main_config.mag_cal_zy = -0.00586774;
		main_config.mag_cal_zz = 0.999047;
		break;

	default:
		main_config.mag_cal_cx = 0.0;
		main_config.mag_cal_cy = 0.0;
		main_config.mag_cal_cz = 0.0;

		main_config.mag_cal_xx = 1.0;
		main_config.mag_cal_xy = 0.0;
		main_config.mag_cal_xz = 0.0;

		main_config.mag_cal_yx = 0.0;
		main_config.mag_cal_yy = 1.0;
		main_config.mag_cal_yz = 0.0;

		main_config.mag_cal_zx = 0.0;
		main_config.mag_cal_zy = 0.0;
		main_config.mag_cal_zz = 1.0;
		break;
	}
}
