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

#ifndef SRF10_H_
#define SRF10_H_

#include "ch.h"
#include "hal.h"
#include "conf_general.h"

void srf10_init(void);
void srf10_set_sample_callback(void (*func)(float distance));
msg_t srf10_set_gain(uint8_t gain);
msg_t srf10_set_range(uint8_t range);
msg_t srf10_start_ranging(void);
msg_t srf10_read_range(float *range);
msg_t srf10_reset_i2c(void);

#endif /* SRF10_H_ */
