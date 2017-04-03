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

#ifndef RADAR_H_
#define RADAR_H_

#include "conf_general.h"

#if RADAR_EN

// Functions
void radar_init(void);
void radar_setup_measurement(radar_settings_t *settings);
const radar_settings_t *radar_get_settings(void);
void radar_reset_setup(void);
void radar_sample(void);
void radar_setup_measurement_default(void);
void radar_cmd(char *cmd);

#endif

#endif /* RADAR_H_ */
