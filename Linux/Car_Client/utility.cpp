/*
    Copyright 2016 - 2018 Benjamin Vedder	benjamin@vedder.se

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
#include <QDateTime>

namespace {
inline double roundDouble(double x) {
    return x < 0.0 ? ceil(x - 0.5) : floor(x + 0.5);
}
const unsigned short crc16_tab[] = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
                                     0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
                                     0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
                                     0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
                                     0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
                                     0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
                                     0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
                                     0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
                                     0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
                                     0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
                                     0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
                                     0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
                                     0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
                                     0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
                                     0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
                                     0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
                                     0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
                                     0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
                                     0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
                                     0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
                                     0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
                                     0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
                                     0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
                                     0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
                                     0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
                                     0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
                                     0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
                                     0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
                                     0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };
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

unsigned short crc16(const unsigned char *buf, unsigned int len)
{
    unsigned int i;
    unsigned short cksum = 0;
    for (i = 0; i < len; i++) {
        cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
    }
    return cksum;
}

int getTimeUtcToday()
{
    QDateTime date = QDateTime::currentDateTime();
    QTime current = QTime::currentTime().addSecs(-date.offsetFromUtc());
    return current.msecsSinceStartOfDay();
}

void stepTowards(double *value, double goal, double step)
{
    if (*value < goal) {
        if ((*value + step) < goal) {
            *value += step;
        } else {
            *value = goal;
        }
    } else if (*value > goal) {
        if ((*value - step) > goal) {
            *value -= step;
        } else {
            *value = goal;
        }
    }
}

double sign(double x)
{
    return x >= 0 ? 1.0 : -1.0;
}

int truncateNumber(double *number, double min, double max)
{
    int did_trunc = 0;

    if (*number > max) {
        *number = max;
        did_trunc = 1;
    } else if (*number < min) {
        *number = min;
        did_trunc = 1;
    }

    return did_trunc;
}

void normAngle(double *angle)
{
    while (*angle < -180.0) {
        *angle += 360.0;
    }

    while (*angle >  180.0) {
        *angle -= 360.0;
    }
}

}
