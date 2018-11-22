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

#ifndef MOTORSIM_H
#define MOTORSIM_H

#include <QObject>
#include <QTimer>

class MotorSim : public QObject
{
    Q_OBJECT
public:
    typedef enum {
        MOTOR_CONTROL_DUTY = 0,
        MOTOR_CONTROL_CURRENT,
        MOTOR_CONTROL_CURRENT_BRAKE,
        MOTOR_CONTROL_RPM,
        MOTOR_CONTROL_POS
    } motor_control_mode;

    explicit MotorSim(QObject *parent = nullptr);
    void setControl(MotorSim::motor_control_mode mode, double value);

    int tacho() const;
    int tachoAbs() const;
    double rpm() const;
    double duty() const;
    int poles() const;

signals:

public slots:

private slots:
    void timerSlot();

private:
    QTimer *mTimer;

    motor_control_mode mMode;
    double mModeValue;
    double mTimeout;
    double mTachoF;
    double mTachoAbsF;
    double mRpm;
    double mDuty;

};

#endif // MOTORSIM_H
