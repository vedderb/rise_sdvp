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

#ifndef CARSIM_H
#define CARSIM_H

#include <QObject>
#include <QTimer>
#include "datatypes.h"
#include "vbytearray.h"
#include "motorsim.h"
#include "autopilot.h"

class CarSim : public QObject
{
    Q_OBJECT
public:
    explicit CarSim(QObject *parent = nullptr);

    quint8 id() const;
    void setId(const quint8 &id);

signals:
    void dataToSend(QByteArray data);

public slots:
    void processData(QByteArray data);
    void setMotorSpeed(double speed);
    void setMotorCurrentBrake(double current);
    void setSteeringTurnRad(double turnRad);
    void setSteering(double steering);

private slots:
    void timerSlot();

private:
    static const quint8 FW_VERSION_MAJOR = 10;
    static const quint8 FW_VERSION_MINOR = 2;

    typedef struct {
        double px;
        double py;
        double speed;
        double roll;
        double pitch;
        double yaw;
        double roll_rate;
        double pitch_rate;
        double yaw_rate;
        double accel_x;
        double accel_y;
        double accel_z;
        double steering;
        int motor_tacho;
    } CAR_SIM_STATE;

    QTimer *mTimer;
    MotorSim *mMotor;
    Autopilot *mAutoPilot;
    quint8 mId;
    int mRxTimer;
    int mRxState;
    unsigned int mPayloadLength;
    QByteArray mRxBuffer;
    unsigned char mCrcLow;
    unsigned char mCrcHigh;
    CAR_SIM_STATE mSimState;

    void processPacket(VByteArray vb);
    void sendPacket(VByteArray data);

};

#endif // CARSIM_H
