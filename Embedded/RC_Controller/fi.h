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

/*
 * This is a simple fault injection library that uses terminal callbacks for configuration
 * and activation.
 */

#ifndef FI_H_
#define FI_H_

#include <stdint.h>
#include <stdbool.h>

// Functions
void fi_init(void);
bool fi_is_active(void);
void fi_inject_fault_float(const char* id, float *value);
void fi_inject_fault_int(const char* id, int *value);

#endif /* FI_H_ */
