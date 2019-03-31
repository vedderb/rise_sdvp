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

#include "vbytearrayle.h"
#include <cmath>
#include <stdint.h>

// NOTE: This version of VByteArray is Little Endian...

namespace {
inline double roundDouble(double x) {
    return x < 0.0 ? ceil(x - 0.5) : floor(x + 0.5);
}
}

VByteArrayLe::VByteArrayLe()
{

}

VByteArrayLe::VByteArrayLe(const QByteArray &data) : QByteArray(data)
{

}

void VByteArrayLe::vbAppendInt32(qint32 number)
{
    QByteArray data;
    data.prepend((char)((number >> 24) & 0xFF));
    data.prepend((char)((number >> 16) & 0xFF));
    data.prepend((char)((number >> 8) & 0xFF));
    data.prepend((char)(number & 0xFF));
    append(data);
}

void VByteArrayLe::vbAppendUint32(quint32 number)
{
    QByteArray data;
    data.prepend((char)((number >> 24) & 0xFF));
    data.prepend((char)((number >> 16) & 0xFF));
    data.prepend((char)((number >> 8) & 0xFF));
    data.prepend((char)(number & 0xFF));
    append(data);
}

void VByteArrayLe::vbAppendInt16(qint16 number)
{
    QByteArray data;
    data.prepend((char)((number >> 8) & 0xFF));
    data.prepend((char)(number & 0xFF));
    append(data);
}

void VByteArrayLe::vbAppendUint16(quint16 number)
{
    QByteArray data;
    data.prepend((char)((number >> 8) & 0xFF));
    data.prepend((char)(number & 0xFF));
    append(data);
}

void VByteArrayLe::vbAppendInt8(qint8 number)
{
    append((char)number);
}

void VByteArrayLe::vbAppendUint8(quint8 number)
{
    append((char)number);
}

void VByteArrayLe::vbAppendDouble32(double number, double scale)
{
    vbAppendInt32((qint32)roundDouble(number * scale));
}

void VByteArrayLe::vbAppendDouble16(double number, double scale)
{
    vbAppendInt16((qint16)roundDouble(number * scale));
}

void VByteArrayLe::vbAppendDouble32Auto(double number)
{
    int e = 0;
    float fr = frexpf(number, &e);
    float fr_abs = fabsf(fr);
    uint32_t fr_s = 0;

    if (fr_abs >= 0.5) {
        fr_s = (uint32_t)((fr_abs - 0.5f) * 2.0f * 8388608.0f);
        e += 126;
    }

    uint32_t res = ((e & 0xFF) << 23) | (fr_s & 0x7FFFFF);
    if (fr < 0) {
        res |= 1 << 31;
    }

    vbAppendUint32(res);
}

void VByteArrayLe::vbAppendString(QString str)
{
    append(str.toLocal8Bit());
    append((char)0);
}

void VByteArrayLe::vbAppendUint48(quint64 number)
{
    QByteArray data;
    data.prepend((char)((number >> 40) & 0xFF));
    data.prepend((char)((number >> 32) & 0xFF));
    data.prepend((char)((number >> 24) & 0xFF));
    data.prepend((char)((number >> 16) & 0xFF));
    data.prepend((char)((number >> 8) & 0xFF));
    data.prepend((char)(number & 0xFF));
    append(data);
}

qint32 VByteArrayLe::vbPopFrontInt32()
{
    if (size() < 4) {
        return 0;
    }

    qint32 res =	((quint8) at(3)) << 24 |
                    ((quint8) at(2)) << 16 |
                    ((quint8) at(1)) << 8 |
                    ((quint8) at(0));

    remove(0, 4);
    return res;
}

quint32 VByteArrayLe::vbPopFrontUint32()
{
    if (size() < 4) {
        return 0;
    }

    quint32 res =	((quint8) at(3)) << 24 |
                    ((quint8) at(2)) << 16 |
                    ((quint8) at(1)) << 8 |
                    ((quint8) at(0));

    remove(0, 4);
    return res;
}

qint16 VByteArrayLe::vbPopFrontInt16()
{
    if (size() < 2) {
        return 0;
    }

    qint16 res =	((quint8) at(1)) << 8 |
                    ((quint8) at(0));

    remove(0, 2);
    return res;
}

quint16 VByteArrayLe::vbPopFrontUint16()
{
    if (size() < 2) {
        return 0;
    }

    quint16 res =	((quint8) at(1)) << 8 |
                    ((quint8) at(0));

    remove(0, 2);
    return res;
}

qint8 VByteArrayLe::vbPopFrontInt8()
{
    if (size() < 1) {
        return 0;
    }

    qint8 res = (quint8)at(0);

    remove(0, 1);
    return res;
}

quint8 VByteArrayLe::vbPopFrontUint8()
{
    if (size() < 1) {
        return 0;
    }

    quint8 res = (quint8)at(0);

    remove(0, 1);
    return res;
}

double VByteArrayLe::vbPopFrontDouble48(double scale)
{
    return (double)vbPopFrontInt48() / scale;
}


double VByteArrayLe::vbPopFrontDouble32(double scale)
{
    return (double)vbPopFrontInt32() / scale;
}

double VByteArrayLe::vbPopFrontDouble16(double scale)
{
    return (double)vbPopFrontInt16() / scale;
}

double VByteArrayLe::vbPopFrontDouble32Auto()
{
    uint32_t res = vbPopFrontUint32();

    int e = (res >> 23) & 0xFF;
    int fr = res & 0x7FFFFF;
    bool negative = res & (1 << 31);

    float f = 0.0;
    if (e != 0 || fr != 0) {
        f = (float)fr / (8388608.0 * 2.0) + 0.5;
        e -= 126;
    }

    if (negative) {
        f = -f;
    }

    return ldexpf(f, e);
}

QString VByteArrayLe::vbPopFrontString()
{
    if (size() < 1) {
        return QString();
    }

    QString str(data());
    remove(0, str.size() + 1);
    return str;
}

quint64 VByteArrayLe::vbPopFrontUint48()
{
    if (size() < 6) {
        return 0;
    }

    quint64 res =	(quint64)((quint8)at(5)) << 40 |
                    (quint64)((quint8)at(4)) << 32 |
                    (quint64)((quint8)at(3)) << 24 |
                    (quint64)((quint8)at(2)) << 16 |
                    (quint64)((quint8)at(1)) << 8 |
                    (quint64)((quint8)at(0));

    remove(0, 6);
    return res;
}

qint64 VByteArrayLe::vbPopFrontInt48()
{
   return (qint64)vbPopFrontUint48();
}
