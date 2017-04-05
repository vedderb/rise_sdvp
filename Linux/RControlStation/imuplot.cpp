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

#include "imuplot.h"
#include "ui_imuplot.h"

ImuPlot::ImuPlot(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ImuPlot)
{
    ui->setupUi(this);
    layout()->setContentsMargins(0, 0, 0, 0);

    mMaxSampleSize = 1000;
    mAccelXData.resize(mMaxSampleSize);
    mAccelYData.resize(mMaxSampleSize);
    mAccelZData.resize(mMaxSampleSize);
    mGyroXData.resize(mMaxSampleSize);
    mGyroYData.resize(mMaxSampleSize);
    mGyroZData.resize(mMaxSampleSize);
    mMagXData.resize(mMaxSampleSize);
    mMagYData.resize(mMaxSampleSize);
    mMagZData.resize(mMaxSampleSize);
    mAccelGyroMagXAxis.resize(mMaxSampleSize);
    for(int i = 0;i < mAccelGyroMagXAxis.size();i++) {
        mAccelGyroMagXAxis[mAccelGyroMagXAxis.size() - i - 1] = (40.0 / 1000.0 * i);
    }

    ui->accelPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->gyroPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->magPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    ui->accelPlot->clearGraphs();
    ui->accelPlot->addGraph();
    ui->accelPlot->xAxis->setRangeReversed(true);
    ui->accelPlot->graph()->setPen(QPen(Qt::black));
    ui->accelPlot->graph()->setData(mAccelGyroMagXAxis, mAccelXData);
    ui->accelPlot->graph()->setName(tr("X"));
    ui->accelPlot->addGraph();
    ui->accelPlot->graph()->setPen(QPen(Qt::green));
    ui->accelPlot->graph()->setData(mAccelGyroMagXAxis, mAccelYData);
    ui->accelPlot->graph()->setName(tr("Y"));
    ui->accelPlot->addGraph();
    ui->accelPlot->graph()->setPen(QPen(Qt::blue));
    ui->accelPlot->graph()->setData(mAccelGyroMagXAxis, mAccelZData);
    ui->accelPlot->graph()->setName(tr("Z"));
    ui->accelPlot->rescaleAxes();
    ui->accelPlot->xAxis->setLabel("Seconds");
    ui->accelPlot->yAxis->setLabel("G");
    ui->accelPlot->legend->setVisible(true);
    ui->accelPlot->replot();

    ui->gyroPlot->clearGraphs();
    ui->gyroPlot->addGraph();
    ui->gyroPlot->xAxis->setRangeReversed(true);
    ui->gyroPlot->graph()->setPen(QPen(Qt::black));
    ui->gyroPlot->graph()->setData(mAccelGyroMagXAxis, mGyroXData);
    ui->gyroPlot->graph()->setName(tr("X"));
    ui->gyroPlot->addGraph();
    ui->gyroPlot->graph()->setPen(QPen(Qt::green));
    ui->gyroPlot->graph()->setData(mAccelGyroMagXAxis, mGyroYData);
    ui->gyroPlot->graph()->setName(tr("Y"));
    ui->gyroPlot->addGraph();
    ui->gyroPlot->graph()->setPen(QPen(Qt::blue));
    ui->gyroPlot->graph()->setData(mAccelGyroMagXAxis, mGyroZData);
    ui->gyroPlot->graph()->setName(tr("Z"));
    ui->gyroPlot->rescaleAxes();
    ui->gyroPlot->xAxis->setLabel("Seconds");
    ui->gyroPlot->yAxis->setLabel("deg/s");
    ui->gyroPlot->legend->setVisible(true);
    ui->gyroPlot->replot();

    ui->magPlot->clearGraphs();
    ui->magPlot->addGraph();
    ui->magPlot->xAxis->setRangeReversed(true);
    ui->magPlot->graph()->setPen(QPen(Qt::black));
    ui->magPlot->graph()->setData(mAccelGyroMagXAxis, mMagXData);
    ui->magPlot->graph()->setName(tr("X"));
    ui->magPlot->addGraph();
    ui->magPlot->graph()->setPen(QPen(Qt::green));
    ui->magPlot->graph()->setData(mAccelGyroMagXAxis, mMagYData);
    ui->magPlot->graph()->setName(tr("Y"));
    ui->magPlot->addGraph();
    ui->magPlot->graph()->setPen(QPen(Qt::blue));
    ui->magPlot->graph()->setData(mAccelGyroMagXAxis, mMagZData);
    ui->magPlot->graph()->setName(tr("Z"));
    ui->magPlot->rescaleAxes();
    ui->magPlot->xAxis->setLabel("Seconds");
    ui->magPlot->yAxis->setLabel("uT");
    ui->magPlot->legend->setVisible(true);
    ui->magPlot->replot();

    mTimer = new QTimer(this);
    mTimer->start(20);
    mImuReplot = false;

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
}

ImuPlot::~ImuPlot()
{
    delete ui;
}

void ImuPlot::addSample(double *accel, double *gyro, double *mag)
{
    mAccelXData.append(accel[0]);
    mAccelXData.remove(0, 1);
    mAccelYData.append(accel[1]);
    mAccelYData.remove(0, 1);
    mAccelZData.append(accel[2]);
    mAccelZData.remove(0, 1);

    mGyroXData.append(gyro[0] * 180.0 / M_PI);
    mGyroXData.remove(0, 1);
    mGyroYData.append(gyro[1] * 180.0 / M_PI);
    mGyroYData.remove(0, 1);
    mGyroZData.append(gyro[2] * 180.0 / M_PI);
    mGyroZData.remove(0, 1);

    mMagXData.append(mag[0]);
    mMagXData.remove(0, 1);
    mMagYData.append(mag[1]);
    mMagYData.remove(0, 1);
    mMagZData.append(mag[2]);
    mMagZData.remove(0, 1);

    mImuReplot = true;
}

void ImuPlot::timerSlot()
{
    if (mImuReplot) {
        ui->accelPlot->graph(0)->setData(mAccelGyroMagXAxis, mAccelXData);
        ui->accelPlot->graph(1)->setData(mAccelGyroMagXAxis, mAccelYData);
        ui->accelPlot->graph(2)->setData(mAccelGyroMagXAxis, mAccelZData);
        ui->accelPlot->rescaleAxes();
        ui->accelPlot->replot();

        ui->gyroPlot->graph(0)->setData(mAccelGyroMagXAxis, mGyroXData);
        ui->gyroPlot->graph(1)->setData(mAccelGyroMagXAxis, mGyroYData);
        ui->gyroPlot->graph(2)->setData(mAccelGyroMagXAxis, mGyroZData);
        ui->gyroPlot->rescaleAxes();
        ui->gyroPlot->replot();

        ui->magPlot->graph(0)->setData(mAccelGyroMagXAxis, mMagXData);
        ui->magPlot->graph(1)->setData(mAccelGyroMagXAxis, mMagYData);
        ui->magPlot->graph(2)->setData(mAccelGyroMagXAxis, mMagZData);
        ui->magPlot->rescaleAxes();
        ui->magPlot->replot();

        mImuReplot = false;
    }
}
