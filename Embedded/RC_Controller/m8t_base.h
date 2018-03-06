/*
	Copyright 2018 Benjamin Vedder	benjamin@vedder.se

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

#ifndef M8T_BASE_H_
#define M8T_BASE_H_

#include "conf_general.h"
#include "ch.h"
#include "hal.h"

// Functions
void m8t_base_init(void);
void m8t_base_start(void);
void m8t_base_stop(void);
void m8t_base_set_pos(double lat, double lon, double height);
void m8t_base_reset_pos(void);
void m8t_base_set_min_acc_samples(float acc, int samples);

#endif /* M8T_BASE_H_ */
