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

#include "intersectiontest.h"
#include "ui_intersectiontest.h"
#include "utility.h"

#include <QMessageBox>
#include <cmath>

IntersectionTest::IntersectionTest(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::IntersectionTest)
{
    ui->setupUi(this);
    mCars = 0;
    mMap = 0;
    mPacketInterface = 0;
    mRtRangeInt = 0;
    mRunning = false;
}

IntersectionTest::~IntersectionTest()
{
    delete ui;
}

void IntersectionTest::setCars(QList<CarInterface *> *cars)
{
    mCars = cars;
}

void IntersectionTest::setMap(MapWidget *map)
{
    mMap = map;
}

void IntersectionTest::setPacketInterface(PacketInterface *packet)
{
    mPacketInterface = packet;
}

void IntersectionTest::nComRx(const ncom_data &data)
{
    mRtRangeData = data;

    mRtRangeInt++;
    if (mRtRangeInt >= 5 && mRunning) {
        mRtRangeInt = 0;

        if (mMap && mPacketInterface) {
            LocPoint sync;
            bool syncFound = false;
            if (mMap->getRoute(ui->routeSyncBox->value()).size() > 0) {
                sync = mMap->getRoute(ui->routeSyncBox->value()).at(0);
                syncFound = true;
            }

            if (syncFound) {
                double v = sqrt(mRtRangeData.velE * mRtRangeData.velE +
                                mRtRangeData.velN * mRtRangeData.velN);
                double a_vel = atan2(mRtRangeData.velN, mRtRangeData.velE);
                double xDiff = sync.getX() - mRtRangeData.mapX;
                double yDiff = sync.getY() - mRtRangeData.mapY;
                double dSync = sqrt(xDiff * xDiff + yDiff * yDiff);
                double a_car = atan2(yDiff, xDiff);
                double a_diff = utility::angle_difference_rad(a_car, a_vel);
                double vComp = v * cos(a_diff);
                double tSync = dSync / vComp;
                utility::truncate_number_abs(&tSync, 1200);

                QString str;
                str.sprintf("velE:  %.2f\n"
                            "velN:  %.2f\n"
                            "v:     %.2f\n"
                            "a_diff:%.2f\n"
                            "a_vel: %.2f\n"
                            "a_car: %.2f\n"
                            "vComp: %.2f\n"
                            "tSync: %.2f",

                            mRtRangeData.velE,
                            mRtRangeData.velN,
                            v,
                            a_diff * 180.0 / M_PI,
                            a_vel * 180.0 / M_PI,
                            a_car * 180.0 / M_PI,
                            vComp,
                            tSync);
                ui->terminalEdit->clear();
                ui->terminalEdit->appendPlainText(str);

                if (tSync > 1.0) {
                    if (ui->carABox->value() >= 0) {
                        mPacketInterface->setSyncPoint(ui->carABox->value(), ui->syncABox->value(),
                                                       (int)(tSync * 1000.0), 1000.0, false);
                    }

                    if (ui->carBBox->value() >= 0) {
                        mPacketInterface->setSyncPoint(ui->carBBox->value(), ui->syncBBox->value(),
                                                       (int)(tSync * 1000.0), 1000.0, false);
                    }
                }
            }
        }
    }
}

void IntersectionTest::on_runButton_clicked()
{
    ui->settingsWidget->setEnabled(false);
    bool ok = true;

    if (mCars && mCars->isEmpty()) {
        return;
    }

    if (mCars && mPacketInterface) {
        for (CarInterface *car: *mCars) {
            if (car->getId() == ui->carABox->value()) {
                car->disableKbBox();
            } else if (car->getId() == ui->carBBox->value()) {
                car->disableKbBox();
            }
        }

        for (CarInterface *car: *mCars) {
            int carId = ui->carABox->value();

            if (carId == ui->carABox->value()) {
                QList<LocPoint> route = mMap->getRoute(ui->routeABox->value());

                if (!mPacketInterface->clearRoute(carId)) {
                    ok = false;
                }

                if (!utility::uploadRouteHelper(mPacketInterface, carId, route)) {
                    ok = false;
                }
            } else if (carId == ui->carBBox->value()) {
                car->emergencyStop();

                QList<LocPoint> route = mMap->getRoute(ui->routeBBox->value());

                if (!mPacketInterface->clearRoute(carId)) {
                    ok = false;
                }

                if (!utility::uploadRouteHelper(mPacketInterface, carId, route)) {
                    ok = false;
                }
            }
        }

        if (!ok) {
            QMessageBox::warning(this, "Start Intersection Test",
                                 "Uploading routes failed");
        }

        if (ok) {
            for (CarInterface *car: *mCars) {
                if (car->getId() == ui->carABox->value()) {
                    car->setAp(true);
                } else if (car->getId() == ui->carBBox->value()) {
                    car->setAp(true);
                }
            }
        }
    }

    mRunning = true;
    ui->runningLabel->setText("Running");
}

void IntersectionTest::on_stopButton_clicked()
{
    mRunning = false;
    ui->runningLabel->setText("Stopped");
    ui->settingsWidget->setEnabled(true);

    if (mCars && mCars->isEmpty()) {
        return;
    }

    if (mCars) {
        for (CarInterface *car: *mCars) {
            if (car->getId() == ui->carABox->value()) {
                car->emergencyStop();
            } else if (car->getId() == ui->carBBox->value()) {
                car->emergencyStop();
            }
        }
    }
}
