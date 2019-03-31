/*
    Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

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

#ifndef BUFFER_H_
#define BUFFER_H_

#include <stdint.h>
#include <QObject>

namespace utility {

void buffer_append_int64(uint8_t* buffer, int64_t number, int32_t *index);
void buffer_append_uint64(uint8_t *buffer, uint64_t number, int32_t *index);
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index);
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);
void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index);
void buffer_append_double16(uint8_t* buffer, double number, double scale, int32_t *index);
void buffer_append_double32(uint8_t* buffer, double number, double scale, int32_t *index);
void buffer_append_double64(uint8_t* buffer, double number, double scale, int32_t *index);
void buffer_append_double32_auto(uint8_t* buffer, double number, int32_t *index);
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index);
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index);
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index);
uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index);
uint64_t buffer_get_uint64(const uint8_t *buffer, int32_t *index);
int64_t buffer_get_int64(const uint8_t *buffer, int32_t *index);
double buffer_get_double16(const uint8_t *buffer, double scale, int32_t *index);
double buffer_get_double32(const uint8_t *buffer, double scale, int32_t *index);
double buffer_get_double64(const uint8_t *buffer, double scale, int32_t *index);
double buffer_get_double32_auto(const uint8_t *buffer, int32_t *index);
double map(double x, double in_min, double in_max, double out_min, double out_max);
void llhToXyz(double lat, double lon, double height, double *x, double *y, double *z);
void xyzToLlh(double x, double y, double z, double *lat, double *lon, double *height);
void createEnuMatrix(double lat, double lon, double *enuMat);
void llhToEnu(const double *iLlh, const double *llh, double *xyz);
void enuToLlh(const double *iLlh, const double *xyz, double *llh);
double logn(double base, double number);
unsigned short crc16(const unsigned char *buf, unsigned int len);
int getTimeUtcToday();
void stepTowards(double *value, double goal, double step);
double sign(double x);
int truncateNumber(double *number, double min, double max);
void normAngle(double *angle);
bool waitSignal(QObject *sender, const char *signal, int timeoutMs);

}

#endif /* BUFFER_H_ */
