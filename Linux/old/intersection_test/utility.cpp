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

#include "utility.h"
#include <cmath>

namespace {
inline double roundDouble(double x) {
    return x < 0.0 ? ceil(x - 0.5) : floor(x + 0.5);
}
}

namespace utility {

#define FE_WGS84        (1.0/298.257223563) // earth flattening (WGS84)
#define RE_WGS84        6378137.0           // earth semimajor axis (WGS84) (m)

void buffer_append_int64(uint8_t *buffer, int64_t number, int32_t *index)
{
    buffer[(*index)++] = number >> 56;
    buffer[(*index)++] = number >> 48;
    buffer[(*index)++] = number >> 40;
    buffer[(*index)++] = number >> 32;
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void buffer_append_uint64(uint8_t *buffer, uint64_t number, int32_t *index)
{
    buffer[(*index)++] = number >> 56;
    buffer[(*index)++] = number >> 48;
    buffer[(*index)++] = number >> 40;
    buffer[(*index)++] = number >> 32;
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

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

void buffer_append_double64(uint8_t* buffer, double number, double scale, int32_t *index) {
    buffer_append_int64(buffer, (int64_t)(roundDouble(number * scale)), index);
}

void buffer_append_double32_auto(uint8_t *buffer, double number, int32_t *index)
{
    int e = 0;
    float sig = frexpf(number, &e);
    float sig_abs = fabsf(sig);
    uint32_t sig_i = 0;

    if (sig_abs >= 0.5) {
        sig_i = (uint32_t)((sig_abs - 0.5f) * 2.0f * 8388608.0f);
        e += 126;
    }

    uint32_t res = ((e & 0xFF) << 23) | (sig_i & 0x7FFFFF);
    if (sig < 0) {
        res |= 1 << 31;
    }

    buffer_append_uint32(buffer, res, index);
}

int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
    int16_t res =	((uint16_t) buffer[*index]) << 8 |
                    ((uint16_t) buffer[*index + 1]);
    *index += 2;
    return res;
}

uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index) {
    uint16_t res = 	((uint16_t) buffer[*index]) << 8 |
                    ((uint16_t) buffer[*index + 1]);
    *index += 2;
    return res;
}

int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
    int32_t res =	((uint32_t) buffer[*index]) << 24 |
                    ((uint32_t) buffer[*index + 1]) << 16 |
                    ((uint32_t) buffer[*index + 2]) << 8 |
                    ((uint32_t) buffer[*index + 3]);
    *index += 4;
    return res;
}

uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index) {
    uint32_t res =	((uint32_t) buffer[*index]) << 24 |
                    ((uint32_t) buffer[*index + 1]) << 16 |
                    ((uint32_t) buffer[*index + 2]) << 8 |
                    ((uint32_t) buffer[*index + 3]);
    *index += 4;
    return res;
}

int64_t buffer_get_int64(const uint8_t *buffer, int32_t *index) {
    int64_t res =	((uint64_t) buffer[*index]) << 56 |
                    ((uint64_t) buffer[*index + 1]) << 48 |
                    ((uint64_t) buffer[*index + 2]) << 40 |
                    ((uint64_t) buffer[*index + 3]) << 32 |
                    ((uint64_t) buffer[*index + 4]) << 24 |
                    ((uint64_t) buffer[*index + 5]) << 16 |
                    ((uint64_t) buffer[*index + 6]) << 8 |
                    ((uint64_t) buffer[*index + 7]);
    *index += 8;
    return res;
}

uint64_t buffer_get_uint64(const uint8_t *buffer, int32_t *index) {
    uint64_t res =	((uint64_t) buffer[*index]) << 56 |
                    ((uint64_t) buffer[*index + 1]) << 48 |
                    ((uint64_t) buffer[*index + 2]) << 40 |
                    ((uint64_t) buffer[*index + 3]) << 32 |
                    ((uint64_t) buffer[*index + 4]) << 24 |
                    ((uint64_t) buffer[*index + 5]) << 16 |
                    ((uint64_t) buffer[*index + 6]) << 8 |
                    ((uint64_t) buffer[*index + 7]);
    *index += 8;
    return res;
}

double buffer_get_double16(const uint8_t *buffer, double scale, int32_t *index) {
    return (double)buffer_get_int16(buffer, index) / scale;
}

double buffer_get_double32(const uint8_t *buffer, double scale, int32_t *index) {
    return (double)buffer_get_int32(buffer, index) / scale;
}

double buffer_get_double64(const uint8_t *buffer, double scale, int32_t *index) {
    return (double)buffer_get_int64(buffer, index) / scale;
}

double buffer_get_double32_auto(const uint8_t *buffer, int32_t *index)
{
    uint32_t res = buffer_get_uint32(buffer, index);

    int e = (res >> 23) & 0xFF;
    uint32_t sig_i = res & 0x7FFFFF;
    bool neg = res & (1 << 31);

    float sig = 0.0;
    if (e != 0 || sig_i != 0) {
        sig = (float)sig_i / (8388608.0 * 2.0) + 0.5;
        e -= 126;
    }

    if (neg) {
        sig = -sig;
    }

    return ldexpf(sig, e);
}

double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void llhToXyz(double lat, double lon, double height, double *x, double *y, double *z)
{
    double sinp = sin(lat * M_PI / 180.0);
    double cosp = cos(lat * M_PI / 180.0);
    double sinl = sin(lon * M_PI / 180.0);
    double cosl = cos(lon * M_PI / 180.0);
    double e2 = FE_WGS84 * (2.0 - FE_WGS84);
    double v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

    *x = (v + height) * cosp * cosl;
    *y = (v + height) * cosp * sinl;
    *z = (v * (1.0 - e2) + height) * sinp;
}

void xyzToLlh(double x, double y, double z, double *lat, double *lon, double *height)
{
    double e2 = FE_WGS84 * (2.0 - FE_WGS84);
    double r2 = x * x + y * y;
    double za = z;
    double zk = 0.0;
    double sinp = 0.0;
    double v = RE_WGS84;

    while (fabs(za - zk) >= 1E-4) {
        zk = za;
        sinp = za / sqrt(r2 + za * za);
        v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);
        za = z + v * e2 * sinp;
    }

    *lat = (r2 > 1E-12 ? atan(za / sqrt(r2)) : (z > 0.0 ? M_PI / 2.0 : -M_PI / 2.0)) * 180.0 / M_PI;
    *lon = (r2 > 1E-12 ? atan2(y, x) : 0.0) * 180.0 / M_PI;
    *height = sqrt(r2 + za * za) - v;
}

void createEnuMatrix(double lat, double lon, double *enuMat)
{
    double so = sin(lon * M_PI / 180.0);
    double co = cos(lon * M_PI / 180.0);
    double sa = sin(lat * M_PI / 180.0);
    double ca = cos(lat * M_PI / 180.0);

    // ENU
    enuMat[0] = -so;
    enuMat[1] = co;
    enuMat[2] = 0.0;

    enuMat[3] = -sa * co;
    enuMat[4] = -sa * so;
    enuMat[5] = ca;

    enuMat[6] = ca * co;
    enuMat[7] = ca * so;
    enuMat[8] = sa;

    // NED
//    enuMat[0] = -sa * co;
//    enuMat[1] = -sa * so;
//    enuMat[2] = ca;

//    enuMat[3] = -so;
//    enuMat[4] = co;
//    enuMat[5] = 0.0;

//    enuMat[6] = -ca * co;
//    enuMat[7] = -ca * so;
//    enuMat[8] = -sa;
}

void llhToEnu(const double *iLlh, const double *llh, double *xyz)
{
    double ix, iy, iz;
    llhToXyz(iLlh[0], iLlh[1], iLlh[2], &ix, &iy, &iz);

    double x, y, z;
    llhToXyz(llh[0], llh[1], llh[2], &x, &y, &z);

    double enuMat[9];
    createEnuMatrix(iLlh[0], iLlh[1], enuMat);

    double dx = x - ix;
    double dy = y - iy;
    double dz = z - iz;

    xyz[0] = enuMat[0] * dx + enuMat[1] * dy + enuMat[2] * dz;
    xyz[1] = enuMat[3] * dx + enuMat[4] * dy + enuMat[5] * dz;
    xyz[2] = enuMat[6] * dx + enuMat[7] * dy + enuMat[8] * dz;
}

void enuToLlh(const double *iLlh, const double *xyz, double *llh)
{
    double ix, iy, iz;
    llhToXyz(iLlh[0], iLlh[1], iLlh[2], &ix, &iy, &iz);

    double enuMat[9];
    createEnuMatrix(iLlh[0], iLlh[1], enuMat);

    double x = enuMat[0] * xyz[0] + enuMat[3] * xyz[1] + enuMat[6] * xyz[2] + ix;
    double y = enuMat[1] * xyz[0] + enuMat[4] * xyz[1] + enuMat[7] * xyz[2] + iy;
    double z = enuMat[2] * xyz[0] + enuMat[5] * xyz[1] + enuMat[8] * xyz[2] + iz;

    xyzToLlh(x, y, z, &llh[0], &llh[1], &llh[2]);
}

double logn(double base, double number)
{
    return log(number) / log(base);
}

bool truncate_number(double *number, double min, double max)
{
    bool did_trunc = 0;

    if (*number > max) {
        *number = max;
        did_trunc = 1;
    } else if (*number < min) {
        *number = min;
        did_trunc = 1;
    }

    return did_trunc;
}

bool truncate_number_abs(double *number, double max)
{
    bool did_trunc = 0;

    if (*number > max) {
        *number = max;
        did_trunc = 1;
    } else if (*number < -max) {
        *number = -max;
        did_trunc = 1;
    }

    return did_trunc;
}

void norm_angle(double *angle)
{
    while (*angle < -180.0) {
        *angle += 360.0;
    }

    while (*angle >  180.0) {
        *angle -= 360.0;
    }
}

void norm_angle_rad(double *angle)
{
    while (*angle < -M_PI) {
        *angle += 2.0 * M_PI;
    }

    while (*angle >  M_PI) {
        *angle -= 2.0 * M_PI;
    }
}

}
