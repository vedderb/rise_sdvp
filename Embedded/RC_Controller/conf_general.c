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
}
