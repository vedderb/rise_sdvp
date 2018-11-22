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

#include "motorsim.h"
#include "utility.h"
#include <cmath>
#include <QDebug>

namespace
{
const double input_voltage = 39.0;
const double motor_kv = 520.0;
const double motor_poles = 4.0;
const double erpm_per_sec = 25000;
const double max_current = 80.0;
const double timeout = 2.0;
}

MotorSim::MotorSim(QObject *parent) : QObject(parent)
{
    mTimer = new QTimer(this);
    mTimer->start(10);

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
}

void MotorSim::setControl(MotorSim::motor_control_mode mode, double value)
{
    mMode = mode;
    mModeValue = value;
    mTimeout = 0.0;
}

double MotorSim::duty() const
{
    return mDuty;
}

int MotorSim::poles() const
{
    return motor_poles;
}

double MotorSim::rpm() const
{
    return mRpm;
}

int MotorSim::tacho() const
{
    return mTachoF;
}

int MotorSim::tachoAbs() const
{
    return mTachoAbsF;
}

void MotorSim::timerSlot()
{
    double dt = (double)mTimer->interval() / 1000.0;
    const double rpm_max = input_voltage * motor_kv * (motor_poles / 2.0);

    switch (mMode) {
    case MOTOR_CONTROL_DUTY: {
        double rpm = mModeValue * rpm_max;
        utility::stepTowards(&mRpm, rpm, erpm_per_sec * dt);
    } break;

    case MOTOR_CONTROL_CURRENT: {
        utility::stepTowards(&mRpm, utility::sign(mModeValue) * rpm_max,
                              erpm_per_sec * dt * (fabs(mModeValue) / max_current));
    } break;

    case MOTOR_CONTROL_CURRENT_BRAKE: {
        utility::stepTowards(&mRpm, 0.0,
                              erpm_per_sec * dt * (fabs(mModeValue) / max_current));
    } break;

    case MOTOR_CONTROL_RPM: {
        utility::stepTowards(&mRpm, mModeValue, erpm_per_sec * dt);
    } break;

    case MOTOR_CONTROL_POS: {
        // TODO
    } break;

    default:
        break;
    }

    utility::truncateNumber(&mRpm, -rpm_max, rpm_max);

    // Friction
    if (mMode != MOTOR_CONTROL_RPM) {
        mRpm *= pow(0.9, dt);
        utility::stepTowards(&mRpm, 0.0, erpm_per_sec * dt * 0.02);
    }

    // Update values
    mTachoF += mRpm / 60.0 * dt * 6.0;
    mTachoAbsF += fabs(mRpm) / 60.0 * dt * 6.0;
    mDuty = mRpm / rpm_max;

    // Timeout
    mTimeout += dt;
    if (mTimeout > timeout) {
        mMode = MOTOR_CONTROL_CURRENT_BRAKE;
        mModeValue = 10.0;
    }
}
