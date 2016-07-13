/*
    Copyright 2012 Benjamin Vedder	benjamin@vedder.se

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

#include "locpoint.h"
#include <cmath>

LocPoint::LocPoint(double x, double y, double alpha, double speed, double radius, double sigma, QColor color) :
    mX(x), mY(y), mAlpha(alpha), mSpeed(speed), mRadius(radius), mSigma(sigma), mColor(color)
{

}

LocPoint::LocPoint(const LocPoint &point)
{
    *this = point;
}

double LocPoint::getX() const
{
    return mX;
}

double LocPoint::getY() const
{
    return mY;
}

double LocPoint::getAlpha() const
{
    return mAlpha;
}

double LocPoint::getSpeed() const
{
    return mSpeed;
}

QPointF LocPoint::getPoint() const
{
    return QPointF(mX, mY);
}

QPointF LocPoint::getPointMm() const
{
    return QPointF(mX * 1000.0, mY * 1000.0);
}

double LocPoint::getRadius() const
{
    return mRadius;
}

double LocPoint::getSigma() const
{
    return mSigma;
}

void LocPoint::setX(double x)
{
    mX = x;
}

void LocPoint::setY(double y)
{
    mY = y;
}

void LocPoint::setXY(double x, double y)
{
    mX = x;
    mY = y;
}

QString LocPoint::getInfo() const
{
    return mInfo;
}

QColor LocPoint::getColor() const
{
    return mColor;
}

LocPoint &LocPoint::operator =(const LocPoint &point)
{
    mX = point.mX;
    mY = point.mY;
    mAlpha = point.mAlpha;
    mSpeed = point.mSpeed;
    mRadius = point.mRadius;
    mSigma = point.mSigma;
    mInfo = point.mInfo;
    mColor = point.mColor;
    return *this;
}

bool LocPoint::operator ==(const LocPoint &point)
{
    if (    mX == point.mX &&
            mY == point.mY &&
            mAlpha == point.mAlpha &&
            mSpeed == point.mSpeed &&
            mRadius == point.mRadius &&
            mSigma == point.mSigma &&
            mInfo == point.mInfo &&
            mColor == point.mColor) {
        return true;
    } else {
        return false;
    }
}

bool LocPoint::operator !=(const LocPoint &point)
{
    return !(operator==(point));
}

void LocPoint::setInfo(const QString &info)
{
    mInfo = info;
}

void LocPoint::setAlpha(double alpha)
{
    mAlpha = alpha;
}

void LocPoint::setSpeed(double speed)
{
    mSpeed = speed;
}

void LocPoint::setRadius(double radius)
{
    mRadius = radius;
}

void LocPoint::setSigma(double sigma)
{
    mSigma = sigma;
}

void LocPoint::setColor(const QColor &color)
{
    mColor = color;
}

double LocPoint::getDistanceTo(const LocPoint &point) const
{
    return sqrt((point.mX - mX) * (point.mX - mX) + (point.mY - mY) * (point.mY - mY));
}
