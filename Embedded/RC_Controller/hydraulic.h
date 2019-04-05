/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

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

#ifndef HYDRAULIC_H_
#define HYDRAULIC_H_

#include "conf_general.h"

void hydraulic_init(void);
float hydraulic_get_speed(void);
float hydraulic_get_distance(bool reset);
void hydraulic_set_speed(float speed);
void hydraulic_set_throttle_raw(float throttle);

#endif /* HYDRAULIC_H_ */
