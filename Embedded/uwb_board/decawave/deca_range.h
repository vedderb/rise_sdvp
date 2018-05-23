/*
	Copyright 2017 Benjamin Vedder	benjamin@vedder.se

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

#ifndef DECAWAVE_DECA_RANGE_H_
#define DECAWAVE_DECA_RANGE_H_

#include <stdint.h>
#include <stdbool.h>
#include "deca_device_api.h"
#include "ch.h"
#include "hal.h"

// Functions
void deca_range_configure(uint8_t addr);
void deca_range_poll(uint8_t dest);
void deca_range_measure(uint8_t dest, int samples);
int deca_range_send_data(uint8_t dest, uint8_t *buffer, int len);
void deca_range_set_address(uint8_t addr);
uint8_t deca_range_get_address(void);
void deca_range_set_range_func(void(*func)(float dist, uint8_t id));
void deca_range_set_data_func(void(*func)(uint8_t sender, uint8_t *buffer, int len));

#endif /* DECAWAVE_DECA_RANGE_H_ */
