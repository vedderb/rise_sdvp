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

#ifndef COMM_CC2520_H_
#define COMM_CC2520_H_

#include "conf_general.h"

// Functions
void comm_cc2520_init(void);
void comm_cc2520_send_buffer(uint8_t *data, unsigned int len);
void comm_cc2520_send_packet(uint8_t *data, uint8_t len);

#endif /* COMM_CC2520_H_ */
