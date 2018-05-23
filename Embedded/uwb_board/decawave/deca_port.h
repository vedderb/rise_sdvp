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

#ifndef DECA_PORT_H_
#define DECA_PORT_H_

#include <stdint.h>
#include <stdbool.h>
#include "deca_device_api.h"
#include "ch.h"
#include "hal.h"

// Functions
void deca_port_init(void);
void deca_port_reset(void);
void deca_port_set_spi_slow(void);
void deca_port_set_spi_fast(void);

// Functions used by driver
int deca_port_writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer);
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer);
void deca_port_sleep(unsigned int time_ms);
decaIrqStatus_t decamutexon(void);
void decamutexoff(decaIrqStatus_t s);

// ISR Handlers
void deca_port_ext_cb(EXTDriver *extp, expchannel_t channel);

#endif /* DECA_PORT_H_ */
