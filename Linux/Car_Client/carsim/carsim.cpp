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

#include "carsim.h"
#include "utility.h"

#include <cmath>
#include <cstring>
#include <QDebug>

CarSim::CarSim(QObject *parent) : QObject(parent)
{
    mId = 0;
    mRxState = 0;
    mRxTimer = 0;
    memset(&mSimState, 0, sizeof(CAR_SIM_STATE));
    mPayloadLength = 0;
    mCrcLow = 0;
    mCrcHigh = 0;
    mMotor = new MotorSim(this);
    mAutoPilot = new Autopilot(this);
    mTimer = new QTimer(this);
    mTimer->start(10);
    mUdpSocket = new QUdpSocket(this);
    mDynoConnected = false;

    // Default car parameters
    mCarTurnRad = 1.0;
    mGearRatio = (1.0 / 3.0) * (21.0 / 37.0);
    mWheelDiam = 0.11;
    mAxisDistance = 0.475;

    // Autopilot settings
    // TODO: Set this from a configuration message, or from CLI arguments
    mAutoPilot->setAxisDistance(mAxisDistance);
    mAutoPilot->setBaseRad(1.2);
    mAutoPilot->setModeTime(0);
    mAutoPilot->setRepeatRoutes(true);

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    connect(mAutoPilot, SIGNAL(setSpeed(double)),
            this, SLOT(setMotorSpeed(double)));
    connect(mAutoPilot, SIGNAL(setCurrentBrake(double)),
            this, SLOT(setMotorCurrentBrake(double)));
    connect(mAutoPilot, SIGNAL(setSteeringTurnRad(double)),
            this, SLOT(setSteeringTurnRad(double)));
    connect(mUdpSocket, SIGNAL(readyRead()),
            this, SLOT(readPendingDatagrams()));
}

void CarSim::processData(QByteArray data)
{
    unsigned char rx_data;
    const int rx_timeout = 50;

    for(int i = 0;i < data.length();i++) {
        rx_data = data[i];

        switch (mRxState) {
        case 0:
            if (rx_data == 2) {
                mRxState += 2;
                mRxTimer = rx_timeout;
                mPayloadLength = 0;
            } else if (rx_data == 3) {
                mRxState++;
                mRxTimer = rx_timeout;
                mPayloadLength = 0;
            } else {
                mRxState = 0;
            }
            break;

        case 1:
            mPayloadLength = (unsigned int)rx_data << 8;
            mRxState++;
            mRxTimer = rx_timeout;
            break;

        case 2:
            mPayloadLength |= (unsigned int)rx_data;
            if (mPayloadLength <= 4095 && mPayloadLength > 0) {
                mRxState++;
                mRxBuffer.clear();
                mRxTimer = rx_timeout;
            } else {
                mRxState = 0;
            }
            break;

        case 3:
            mRxBuffer.append(rx_data);
            if (mRxBuffer.size() == (int)mPayloadLength) {
                mRxState++;
            }
            mRxTimer = rx_timeout;
            break;

        case 4:
            mCrcHigh = rx_data;
            mRxState++;
            mRxTimer = rx_timeout;
            break;

        case 5:
            mCrcLow = rx_data;
            mRxState++;
            mRxTimer = rx_timeout;
            break;

        case 6:
            if (rx_data == 3) {
                if (utility::crc16((const unsigned char*)mRxBuffer.constData(), mPayloadLength) ==
                        ((unsigned short)mCrcHigh << 8 | (unsigned short)mCrcLow)) {
                    processPacket(mRxBuffer);
                }
            }

            mRxState = 0;
            break;

        default:
            mRxState = 0;
            break;
        }
    }
}

void CarSim::setMotorSpeed(double speed)
{
    double rpm = speed /
            (mGearRatio *  (2.0 / (double)mMotor->poles()) *
             (1.0 / 60.0) * mWheelDiam * M_PI);

    mMotor->setControl(MotorSim::MOTOR_CONTROL_RPM, rpm);
}

void CarSim::setMotorCurrentBrake(double current)
{
    mMotor->setControl(MotorSim::MOTOR_CONTROL_CURRENT_BRAKE, current);
}

void CarSim::setSteeringTurnRad(double turnRad)
{
    mSimState.steering = -mCarTurnRad / turnRad;
    utility::truncateNumber(&mSimState.steering,
                            -mAutoPilot->autopilot_get_steering_scale(),
                            mAutoPilot->autopilot_get_steering_scale());
}

void CarSim::setSteering(double steering)
{
    mSimState.steering = steering;
    utility::truncateNumber(&mSimState.steering, -1.0, 1.0);
}

Autopilot *CarSim::autopilot()
{
    return mAutoPilot;
}

void CarSim::timerSlot()
{
    if (mRxTimer) {
        mRxTimer--;
    } else {
        mRxState = 0;
    }

    if (!mDynoConnected) {
        // Estimate position from motor RPM and steering angle
        int tacho = mMotor->tacho();
        double distance = (tacho - mSimState.motor_tacho) * mGearRatio
                * (2.0 / (double)mMotor->poles()) * (1.0 / 6.0)
                * mWheelDiam * M_PI;
        mSimState.motor_tacho = tacho;

        // Initial distance is unknown
        if (distance > 2.0) {
            distance = 0;
        }

        double speed = mMotor->rpm() * mGearRatio
                * (double)(2.0 / mMotor->poles()) * (1.0 / 60.0)
                * mWheelDiam * M_PI;
        updateState(distance, speed);
    }
}

void CarSim::readPendingDatagrams()
{
    while (mUdpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(mUdpSocket->pendingDatagramSize());

        mUdpSocket->readDatagram(datagram.data(), datagram.size());

        datagram.remove(0, 1);
        datagram.remove(datagram.size() - 1, 1);
        QString str(datagram);
        QStringList elements = str.split(" ");

        while (!elements.isEmpty()) {
            if (elements.first().toLower() == "avfi") {
                break;
            } else {
                elements.removeFirst();
            }
        }

        if (elements.size() >= 11) {
//            double energy = elements.at(2).toDouble();
//            double targetForce = elements.at(3).toDouble();
            double distance = elements.at(8).toDouble();
//            double acceleration = elements.at(9).toDouble();
            double speed = elements.at(10).toDouble() / 3.6;

            double distance_diff = distance - mSimState.motor_tacho;
            mSimState.motor_tacho = distance;

            // Initial distance is unknown
            if (distance_diff > 20.0) {
                distance_diff = 0;
            }

            updateState(distance_diff, speed);
        }
    }
}

double CarSim::axisDistance() const
{
    return mAxisDistance;
}

void CarSim::setAxisDistance(double axisDistance)
{
    mAxisDistance = axisDistance;
    mAutoPilot->setAxisDistance(mAxisDistance);
}

double CarSim::wheelDiam() const
{
    return mWheelDiam;
}

void CarSim::setWheelDiam(double wheelDiam)
{
    mWheelDiam = wheelDiam;
}

double CarSim::gearRatio() const
{
    return mGearRatio;
}

void CarSim::setGearRatio(double gearRatio)
{
    mGearRatio = gearRatio;
}

double CarSim::carTurnRad() const
{
    return mCarTurnRad;
}

void CarSim::setCarTurnRad(double carTurnRad)
{
    mCarTurnRad = carTurnRad;
}

void CarSim::processPacket(VByteArray vb)
{
    quint8 id = vb.vbPopFrontUint8();
    CMD_PACKET packet_id = (CMD_PACKET)vb.vbPopFrontUint8();

    if (id == mId || id == ID_ALL || id == ID_CAR_CLIENT) {
        int id_ret = mId;

        if (id == ID_CAR_CLIENT) {
            id_ret = ID_CAR_CLIENT;
        }

        switch (packet_id) {
        case CMD_TERMINAL_CMD: {
            // For the scala tests to work
            QString cmd = vb.vbPopFrontString();
            if (cmd.startsWith("fi_")) {
                commPrintf(cmd + " : OK\n");
            } else if (cmd.startsWith("pos_uwb_reset_pos")) {
                commPrintf("UWB Pos reset\n");
            }
        } break;

        case CMD_SET_POS:
        case CMD_SET_POS_ACK: {
            mSimState.px = vb.vbPopFrontDouble32(1e4);
            mSimState.py = vb.vbPopFrontDouble32(1e4);
            mSimState.yaw = vb.vbPopFrontDouble32(1e6);

            if (packet_id == CMD_SET_POS_ACK) {
                VByteArray ack;
                ack.vbAppendUint8(id_ret);
                ack.vbAppendUint8(packet_id);
                sendPacket(ack);
            }
        } break;

        case CMD_SET_ENU_REF: {
            double lat, lon, height;
            lat = vb.vbPopFrontDouble64(1e16);
            lon = vb.vbPopFrontDouble64(1e16);
            height = vb.vbPopFrontDouble32(1e3);

            // TODO: Use values
            (void)lat;
            (void)lon;
            (void)height;

            // Send ack
            VByteArray ack;
            ack.vbAppendUint8(id_ret);
            ack.vbAppendUint8(packet_id);
            sendPacket(ack);
        } break;

        case CMD_GET_ENU_REF: {
            double llh[3];

            // TODO: Get values
            llh[0] = 0.0;
            llh[1] = 0.0;
            llh[2] = 0.0;

            VByteArray ret;
            ret.vbAppendUint8(id_ret);
            ret.vbAppendUint8(packet_id);
            ret.vbAppendDouble64(llh[0], 1e16);
            ret.vbAppendDouble64(llh[1], 1e16);
            ret.vbAppendDouble32(llh[2], 1e3);
            sendPacket(ret);
        } break;

        case CMD_GET_STATE: {
            ROUTE_POINT rp_goal;
            mAutoPilot->autopilot_get_goal_now(&rp_goal);

            VByteArray ret;
            ret.vbAppendUint8(id_ret);
            ret.vbAppendUint8(packet_id);
            ret.vbAppendUint8(FW_VERSION_MAJOR);
            ret.vbAppendUint8(FW_VERSION_MINOR);
            ret.vbAppendDouble32(mSimState.roll, 1e6);
            ret.vbAppendDouble32(mSimState.pitch, 1e6);
            ret.vbAppendDouble32(mSimState.yaw, 1e6);
            ret.vbAppendDouble32(mSimState.accel_x, 1e6);
            ret.vbAppendDouble32(mSimState.accel_y, 1e6);
            ret.vbAppendDouble32(mSimState.accel_z, 1e6);
            ret.vbAppendDouble32(mSimState.roll_rate, 1e6);
            ret.vbAppendDouble32(mSimState.pitch_rate, 1e6);
            ret.vbAppendDouble32(mSimState.yaw_rate, 1e6);
            ret.vbAppendDouble32(0.0, 1e6); // Mag x
            ret.vbAppendDouble32(0.0, 1e6); // Mag y
            ret.vbAppendDouble32(0.0, 1e6); // Mag z
            ret.vbAppendDouble32(mSimState.px, 1e4);
            ret.vbAppendDouble32(mSimState.py, 1e4);
            ret.vbAppendDouble32(mSimState.speed, 1e6);
            ret.vbAppendDouble32(12.1, 1e6); // v_in
            ret.vbAppendDouble32(24.0, 1e6); // temp mos
            ret.vbAppendUint8(FAULT_CODE_NONE); // MC Fault code
            ret.vbAppendDouble32(0.0, 1e4); // PX GPS
            ret.vbAppendDouble32(0.0, 1e4); // PY GPS
            ret.vbAppendDouble32(rp_goal.px, 1e4);
            ret.vbAppendDouble32(rp_goal.py, 1e4);
            ret.vbAppendDouble32(mAutoPilot->autopilot_get_rad_now(), 1e6);
            ret.vbAppendInt32(utility::getTimeUtcToday());
            ret.vbAppendInt16(mAutoPilot->autopilot_get_route_left());
            ret.vbAppendDouble32(mSimState.px, 1e4); // UWB px
            ret.vbAppendDouble32(mSimState.py, 1e4); // UWB PY
            sendPacket(ret);
        } break;

        case CMD_RC_CONTROL: {
            RC_MODE mode;
            double throttle, steering;
            mode = (RC_MODE)vb.vbPopFrontUint8();
            throttle = vb.vbPopFrontDouble32(1e4);
            steering = vb.vbPopFrontDouble32(1e6);

            steering *= mAutoPilot->autopilot_get_steering_scale();
            mAutoPilot->autopilot_set_active(false);
            setSteering(steering);

            switch (mode) {
            case RC_MODE_CURRENT:
                mMotor->setControl(MotorSim::MOTOR_CONTROL_CURRENT, throttle);
                break;

            case RC_MODE_DUTY:
                utility::truncateNumber(&throttle, -1.0, 1.0);
                mMotor->setControl(MotorSim::MOTOR_CONTROL_DUTY, throttle);
                break;

            case RC_MODE_PID: // In m/s
                setMotorSpeed(throttle);
                break;

            case RC_MODE_CURRENT_BRAKE:
                mMotor->setControl(MotorSim::MOTOR_CONTROL_CURRENT_BRAKE, throttle);
                break;

            default:
                break;
            }
        } break;

        case CMD_AP_ADD_POINTS: {
            bool first = true;

            while (!vb.isEmpty()) {
                ROUTE_POINT p;
                p.px = vb.vbPopFrontDouble32(1e4);
                p.py = vb.vbPopFrontDouble32(1e4);
                p.speed = vb.vbPopFrontDouble32(1e6);
                p.time = vb.vbPopFrontInt32();
                bool res = mAutoPilot->autopilot_add_point(&p, first);
                first = false;

                if (!res) {
                    break;
                }
            }

            // Send ack
            VByteArray ack;
            ack.vbAppendUint8(id_ret);
            ack.vbAppendUint8(packet_id);
            sendPacket(ack);
        } break;

        case CMD_AP_REMOVE_LAST_POINT: {
            mAutoPilot->autopilot_remove_last_point();

            // Send ack
            VByteArray ack;
            ack.vbAppendUint8(id_ret);
            ack.vbAppendUint8(packet_id);
            sendPacket(ack);
        } break;

        case CMD_AP_CLEAR_POINTS: {
            mAutoPilot->autopilot_clear_route();

            // Send ack
            VByteArray ack;
            ack.vbAppendUint8(id_ret);
            ack.vbAppendUint8(packet_id);
            sendPacket(ack);
        } break;

        case CMD_AP_GET_ROUTE_PART: {
            int first = vb.vbPopFrontInt32();
            int num = vb.vbPopFrontInt8();

            if (num > 20) {
                break;
            }

            VByteArray res;
            res.vbAppendUint8(id_ret);
            res.vbAppendUint8(packet_id);

            int route_len = mAutoPilot->autopilot_get_route_len();
            res.vbAppendInt32(route_len);

            for (int i = first;i < (first + num);i++) {
                ROUTE_POINT rp = mAutoPilot->autopilot_get_route_point(i);
                res.vbAppendDouble32Auto(rp.px);
                res.vbAppendDouble32Auto(rp.py);
                res.vbAppendDouble32Auto(rp.speed);
                res.vbAppendInt32(rp.time);
            }

            sendPacket(res);
        } break;

        case CMD_AP_SET_ACTIVE: {
            mAutoPilot->autopilot_set_active(vb.vbPopFrontInt8());

            // Send ack
            VByteArray ack;
            ack.vbAppendUint8(id_ret);
            ack.vbAppendUint8(packet_id);
            sendPacket(ack);
        } break;

        case CMD_AP_REPLACE_ROUTE: {
            int first = true;

            while (!vb.isEmpty()) {
                ROUTE_POINT p;
                p.px = vb.vbPopFrontDouble32(1e4);
                p.py = vb.vbPopFrontDouble32(1e4);
                p.speed = vb.vbPopFrontDouble32(1e6);
                p.time = vb.vbPopFrontInt32();

                if (first) {
                    first = !mAutoPilot->autopilot_replace_route(&p);
                } else {
                    mAutoPilot->autopilot_add_point(&p, false);
                }
            }

            // Send ack
            VByteArray ack;
            ack.vbAppendUint8(id_ret);
            ack.vbAppendUint8(packet_id);
            sendPacket(ack);
        } break;

        case CMD_AP_SYNC_POINT: {
            int32_t point = vb.vbPopFrontInt32();
            int32_t time = vb.vbPopFrontInt32();
            int32_t min_diff = vb.vbPopFrontInt32();

            mAutoPilot->autopilot_sync_point(point, time, min_diff);

            // Send ack
            VByteArray ack;
            ack.vbAppendUint8(id_ret);
            ack.vbAppendUint8(packet_id);
            sendPacket(ack);
        } break;

        default:
            break;
        }
    }
}

void CarSim::sendPacket(VByteArray data)
{
    VByteArray vb;
    if (data.size() <= 256) {
        vb.vbAppendUint8(2);
        vb.vbAppendUint8(data.size());
    } else {
        vb.vbAppendUint8(3);
        vb.vbAppendUint8(data.size() >> 8);
        vb.vbAppendUint8(data.size() & 0xFF);
    }

    vb.append(data);

    unsigned short crc = utility::crc16((const unsigned char*)data.constData(), data.size());
    vb.append(crc >> 8);
    vb.append(crc);
    vb.append(3);

    emit dataToSend(vb);
}

void CarSim::updateState(double distance, double speed)
{
    double angle_rad = -mSimState.yaw * M_PI / 180.0;

    if (fabs(mSimState.steering) > 1e-6) {
        double turn_rad_rear = mCarTurnRad * -1.0 / mSimState.steering;
        double turn_rad_front = sqrt(mAxisDistance * mAxisDistance + turn_rad_rear * turn_rad_rear);

        if (turn_rad_rear < 0.0) {
            turn_rad_front = -turn_rad_front;
        }

        double angle_diff = (distance * 2.0) / (turn_rad_rear + turn_rad_front);

        mSimState.px += turn_rad_rear * (sin(angle_rad + angle_diff) - sinf(angle_rad));
        mSimState.py += turn_rad_rear * (cos(angle_rad - angle_diff) - cosf(angle_rad));

        angle_rad += (distance * 2.0) / (turn_rad_rear + turn_rad_front);
        mSimState.yaw = -angle_rad * 180.0 / M_PI;
        utility::normAngle(&mSimState.yaw);
    } else {
        mSimState.px += cos(angle_rad) * distance;
        mSimState.py += sin(angle_rad) * distance;
    }

    mSimState.speed = speed;

    mAutoPilot->updatePositionSpeed(mSimState.px, mSimState.py, mSimState.yaw, mSimState.speed);
}

void CarSim::commPrintf(QString str)
{
    VByteArray pkt;
    pkt.vbAppendUint8(mId);
    pkt.vbAppendUint8(CMD_PRINTF);
    pkt.append(str.toLocal8Bit().data());
    sendPacket(pkt);
}

quint8 CarSim::id() const
{
    return mId;
}

void CarSim::setId(const quint8 &id)
{
    mId = id;
}

void CarSim::listenDyno()
{
    mUdpSocket->close();
    if (mUdpSocket->bind(QHostAddress::Any, 7000)) {
        mDynoConnected = true;
    }
}

void CarSim::stopListenDyno()
{
    mUdpSocket->close();
    mDynoConnected = false;
}

