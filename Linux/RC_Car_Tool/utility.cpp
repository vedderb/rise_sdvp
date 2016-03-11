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

#include "utility.h"
#include <cmath>

namespace {
inline double roundDouble(double x) {
    return x < 0.0 ? ceil(x - 0.5) : floor(x + 0.5);
}
}

namespace utility {

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index) {
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_double16(uint8_t* buffer, double number, double scale, int32_t *index) {
    buffer_append_int16(buffer, (int16_t)(roundDouble(number * scale)), index);
}

void buffer_append_double32(uint8_t* buffer, double number, double scale, int32_t *index) {
    buffer_append_int32(buffer, (int32_t)(roundDouble(number * scale)), index);
}

int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
    int16_t res =	((uint8_t) buffer[*index]) << 8 |
                    ((uint8_t) buffer[*index + 1]);
    *index += 2;
    return res;
}

uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index) {
    uint16_t res = 	((uint8_t) buffer[*index]) << 8 |
                    ((uint8_t) buffer[*index + 1]);
    *index += 2;
    return res;
}

int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
    int32_t res =	((uint8_t) buffer[*index]) << 24 |
                    ((uint8_t) buffer[*index + 1]) << 16 |
                    ((uint8_t) buffer[*index + 2]) << 8 |
                    ((uint8_t) buffer[*index + 3]);
    *index += 4;
    return res;
}

uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index) {
    uint32_t res =	((uint8_t) buffer[*index]) << 24 |
                    ((uint8_t) buffer[*index + 1]) << 16 |
                    ((uint8_t) buffer[*index + 2]) << 8 |
                    ((uint8_t) buffer[*index + 3]);
    *index += 4;
    return res;
}

double buffer_get_double16(const uint8_t *buffer, double scale, int32_t *index) {
    return (double)buffer_get_int16(buffer, index) / scale;
}

double buffer_get_double32(const uint8_t *buffer, double scale, int32_t *index) {
    return (double)buffer_get_int32(buffer, index) / scale;
}

}
