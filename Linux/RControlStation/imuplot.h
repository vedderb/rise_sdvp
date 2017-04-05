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

#ifndef IMUPLOT_H
#define IMUPLOT_H

#include <QWidget>

namespace Ui {
class ImuPlot;
}

class ImuPlot : public QWidget
{
    Q_OBJECT

public:
    explicit ImuPlot(QWidget *parent = 0);
    ~ImuPlot();

    void addSample(double *accel, double *gyro, double *mag);

private slots:
    void timerSlot();

private:
    Ui::ImuPlot *ui;
    QTimer *mTimer;
    bool mImuReplot;

    QVector<double> mAccelXData;
    QVector<double> mAccelYData;
    QVector<double> mAccelZData;
    QVector<double> mGyroXData;
    QVector<double> mGyroYData;
    QVector<double> mGyroZData;
    QVector<double> mMagXData;
    QVector<double> mMagYData;
    QVector<double> mMagZData;
    QVector<double> mAccelGyroMagXAxis;
    int mMaxSampleSize;

};

#endif // IMUPLOT_H
