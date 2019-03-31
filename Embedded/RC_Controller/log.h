/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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

#ifndef LOG_H_
#define LOG_H_

#include "ch.h"
#include "chtypes.h"
#include "chsystypes.h"
#include "datatypes.h"

// Functions
void log_init(void);
void log_set_rate(int rate_hz);
void log_set_enabled(bool enabled);
void log_set_name(char *name);
void log_set_ext(LOG_EXT_MODE mode, int baud);

#endif /* LOG_H_ */
