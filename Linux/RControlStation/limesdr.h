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

#ifndef LIMESDR_H
#define LIMESDR_H

#include <QObject>
#include <QThread>
#include <stdarg.h>
#include "gpsgen.h"

class LimeSDR : public QThread
{
    Q_OBJECT
public:
    explicit LimeSDR(QObject *parent = nullptr);
    void setPos(double lat, double lon, double height, double speed);
    void setPosBase(double lat, double lon, double height);
    void setBaseEnabled(bool en);

    void run() override;

signals:
    void statusUpdate(QString str);

public slots:
    void stop();

private:
    bool mStop;
    GpsGen mGps;
    GpsGen mGpsBase;

    double mBaseLat;
    double mBaseLon;
    double mBaseHeight;
    bool mBaseEn;

    void myPrint(const char* format, ...);

};

#endif // LIMESDR_H
