#ifndef INTERSECTIONTEST_H
#define INTERSECTIONTEST_H

#include <QDialog>
#include "datatypes.h"
#include "carinterface.h"
/*
    Copyright 2017 Benjamin Vedder	benjamin@vedder.se

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

#include "mapwidget.h"
#include "packetinterface.h"

namespace Ui {
class IntersectionTest;
}

class IntersectionTest : public QDialog
{
    Q_OBJECT

public:
    explicit IntersectionTest(QWidget *parent = 0);
    ~IntersectionTest();
    void setCars(QList<CarInterface*> *cars);
    void setMap(MapWidget *map);
    void setPacketInterface(PacketInterface *packet);

public slots:
    void nComRx(const ncom_data &data);

private slots:
    void on_runButton_clicked();
    void on_stopButton_clicked();

private:
    Ui::IntersectionTest *ui;
    QList<CarInterface*> *mCars;
    MapWidget *mMap;
    PacketInterface *mPacketInterface;
    ncom_data mRtRangeData;
    int mRtRangeInt;
    bool mRunning;

};

#endif // INTERSECTIONTEST_H
