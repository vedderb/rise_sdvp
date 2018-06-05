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

#include "carinfo.h"

CarInfo::CarInfo(int id, Qt::GlobalColor color)
{
    mId = id;
    mColor = color;
    mName = "";
    mName.sprintf("Car %d", mId);
    mTime = 0;
}

int CarInfo::getId()
{
    return mId;
}

void CarInfo::setId(int id, bool changeName)
{
    mId = id;

    if (changeName) {
        mName = "";
        mName.sprintf("Car %d", mId);
    }
}

QString CarInfo::getName() const
{
    return mName;
}

void CarInfo::setName(QString name)
{
    mName = name;
}

void CarInfo::setLocation(LocPoint &point)
{
    mLocation = point;
}

LocPoint CarInfo::getLocationGps() const
{
    return mLocationGps;
}

void CarInfo::setLocationGps(LocPoint &point)
{
    mLocationGps = point;
}

Qt::GlobalColor CarInfo::getColor() const
{
    return mColor;
}

void CarInfo::setColor(Qt::GlobalColor color)
{
    mColor = color;
}

LocPoint CarInfo::getApGoal() const
{
    return mApGoal;
}

void CarInfo::setApGoal(const LocPoint &apGoal)
{
    mApGoal = apGoal;
}

qint32 CarInfo::getTime() const
{
    return mTime;
}

void CarInfo::setTime(const qint32 &time)
{
    mTime = time;
}

LocPoint CarInfo::getLocationUwb() const
{
    return mLocationUwb;
}

void CarInfo::setLocationUwb(const LocPoint &locationUwb)
{
    mLocationUwb = locationUwb;
}

LocPoint CarInfo::getLocation() const
{
    return mLocation;
}
