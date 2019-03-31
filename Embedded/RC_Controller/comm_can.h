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

#ifndef COMM_CAN_H_
#define COMM_CAN_H_

#include "conf_general.h"

// Functions
void comm_can_init(void);
void comm_can_set_vesc_id(int id);
void comm_can_lock_vesc(void);
void comm_can_unlock_vesc(void);
void comm_can_transmit_eid(uint32_t id, uint8_t *data, uint8_t len);
void comm_can_transmit_sid(uint32_t id, uint8_t *data, uint8_t len);
void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, bool send);
void comm_can_dw_range(uint8_t id, uint8_t dest, int samples);
void comm_can_set_range_func(void(*func)(uint8_t id, uint8_t dest, float range));

#endif /* COMM_CAN_H_ */
