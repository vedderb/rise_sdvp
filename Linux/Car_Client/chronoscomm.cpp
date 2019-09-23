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

#include "chronoscomm.h"
#include <QDateTime>
#include <cmath>

ChronosComm::ChronosComm(QObject *parent) : QObject(parent)
{
    mTcpServer = new TcpServerSimple(this);
    mUdpSocket = new QUdpSocket(this);
    mTcpSocket = new QTcpSocket(this);

    mTcpSocket->setSocketOption(QAbstractSocket::LowDelayOption, true);
    mUdpSocket->setSocketOption(QAbstractSocket::LowDelayOption, true);

    mUdpHostAddress = QHostAddress("0.0.0.0");
    mUdpPort = 0;
    mTransmitterId = 0;
    mChronosSeqNum = 0;
    mCommMode = COMM_MODE_UNDEFINED;

    connect(mTcpServer, SIGNAL(dataRx(QByteArray)),
            this, SLOT(tcpRx(QByteArray)));
    connect(mTcpServer, SIGNAL(connectionChanged(bool,QString)),
            this, SLOT(tcpConnectionChanged(bool,QString)));
    connect(mUdpSocket, SIGNAL(readyRead()),
            this, SLOT(readPendingDatagrams()));

    connect(mTcpSocket, SIGNAL(readyRead()), this, SLOT(tcpInputDataAvailable()));
    connect(mTcpSocket, SIGNAL(disconnected()),
            this, SLOT(tcpInputDisconnected()));
    connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(tcpInputError(QAbstractSocket::SocketError)));
}

bool ChronosComm::startObject(QHostAddress addr)
{
    closeConnection();

    bool res = mTcpServer->startServer(53241, addr);
    qDebug() << "Starting TCP server at" << addr.toString();
    if (!res) {
      qWarning() << "Starting TCP server at" << addr.toString() << "failed:" << mTcpServer->errorString();
    }

    if (res) {
        res = mUdpSocket->bind(addr, 53240);
    }

    if (!res) {
        qWarning() << "Starting UDP server failed:" << mUdpSocket->errorString();
    }

    if (res) {
        qDebug() << "Started CHRONOS object";
    } else {
        qWarning() << "Unable to start chronos object";
    }

    if (res) {
        mCommMode = COMM_MODE_OBJECT;
    }

    return res;
}

bool ChronosComm::startSupervisor(QHostAddress addr)
{
    closeConnection();

    bool res = mTcpServer->startServer(53010, addr);

    if (res) {
        qDebug() << "Started CHRONOS supervisor";
    } else {
        qWarning() << "Unable to start chronos supervisor";
    }

    if (res) {
        mCommMode = COMM_MODE_SUPERVISOR;
    }

    return res;
}

bool ChronosComm::connectAsServer(QString address)
{
    closeConnection();

    mTcpSocket->connectToHost(address, 53241);
    bool res = mTcpSocket->waitForConnected(2000);

    if (!res) {
        qWarning() << "Connecting TCP failed:" << mTcpSocket->errorString();
    }

    if (res) {
        mUdpHostAddress = mTcpSocket->peerAddress();
        res = mUdpSocket->bind(QHostAddress::Any, 12349);
    }

    if (!res) {
        qWarning() << "Starting UDP server failed:" << mUdpSocket->errorString();
    }

    if (res) {
        mCommMode = COMM_MODE_SERVER;
    }

    return res;
}

void ChronosComm::closeConnection()
{
    mTcpServer->stopServer();
    mTcpSocket->close();
    mUdpSocket->close();
    mTcpState = 0;
    mCommMode = COMM_MODE_UNDEFINED;
}

COMM_MODE ChronosComm::getCommMode()
{
    return mCommMode;
}

void ChronosComm::sendTraj(chronos_traj traj)
{
    qDebug() << "Sending TRAJ";

    QVector<chronos_traj_pt> points = traj.traj_pts;

    VByteArrayLe vb;
    vb.vbAppendUint16(ISO_VALUE_ID_TRAJECTORY_ID);
    vb.vbAppendUint16(2);
    vb.vbAppendUint16(traj.traj_id);
    vb.vbAppendUint16(ISO_VALUE_ID_TRAJECTORY_NAME);
    vb.vbAppendUint16(64);

    traj.traj_name.truncate(63);

    for (char c: traj.traj_name.toUtf8()){
        vb.vbAppendUint8((uint8_t)c);
    }

    for (int i = traj.traj_name.size();i < 64;i++) {
        vb.vbAppendUint8(0);
    }

    vb.vbAppendUint16(ISO_VALUE_ID_TRAJECTORY_VERSION);
    vb.vbAppendUint16(2);
    vb.vbAppendUint16(traj.traj_ver);

    vb.vbAppendUint16(AUX_VALUE_ID_OBJECT_ID);
    vb.vbAppendUint16(4);
    vb.vbAppendUint32(traj.object_id);

//    char buf[256];
//    snprintf(buf, 256, "%x", traj.object_id);
//    qDebug() << "TRAJ out IP: " << buf; // << traj.object_id;

    //qDebug() << "Appending number of points: " << points.size();
    for (chronos_traj_pt pt: points) {
        vb.vbAppendUint16(ISO_VALUE_ID_REL_TIME);
        vb.vbAppendUint16(4);
        vb.vbAppendUint32(pt.tRel); // TODO: Multiply with 4?
        vb.vbAppendUint16(ISO_VALUE_ID_X_POS);
        vb.vbAppendUint16(4);
        vb.vbAppendDouble32(pt.x, 1e3);
        vb.vbAppendUint16(ISO_VALUE_ID_Y_POS);
        vb.vbAppendUint16(4);
        vb.vbAppendDouble32(pt.y, 1e3);
        vb.vbAppendUint16(ISO_VALUE_ID_Z_POS);
        vb.vbAppendUint16(4);
        vb.vbAppendDouble32(pt.z, 1e3); // 32
        vb.vbAppendUint16(ISO_VALUE_ID_HEADING);
        vb.vbAppendUint16(2);
        vb.vbAppendUint16(1e2 * pt.heading * 180.0 / M_PI);
        vb.vbAppendUint16(ISO_VALUE_ID_LONG_SPEED);
        vb.vbAppendUint16(2);
        vb.vbAppendDouble16(pt.long_speed, 1e2);
        vb.vbAppendUint16(ISO_VALUE_ID_LAT_SPEED);
        vb.vbAppendUint16(2);
        vb.vbAppendDouble16(pt.lat_speed, 1e2);
        vb.vbAppendUint16(ISO_VALUE_ID_LONG_ACC);
        vb.vbAppendUint16(2);
        vb.vbAppendDouble16(pt.long_accel, 1e3);
        vb.vbAppendUint16(ISO_VALUE_ID_LAT_ACC);
        vb.vbAppendUint16(2);
        vb.vbAppendDouble16(pt.lat_accel, 1e3);
        vb.vbAppendUint16(ISO_VALUE_ID_CURVATURE); // 64
        vb.vbAppendUint16(4);
        vb.vbAppendDouble32(pt.curvature, 3e4); // 70
    }

    mkChronosHeader(vb,
                    mTransmitterId,
                    mChronosSeqNum++,
                    false,
                    PROTOCOL_VERSION,
                    ISO_MSG_TRAJ);

    appendChronosChecksum(vb);

    sendData(vb, false);
}

void ChronosComm::sendHeab(chronos_heab heab)
{
    VByteArrayLe vb;
    vb.vbAppendUint16(ISO_VALUE_ID_HEAB_STRUCT);
    vb.vbAppendUint16(5); // sizeof(chronos_heab)
    vb.vbAppendUint32(heab.gps_ms_of_week * 4);
    vb.vbAppendUint8(heab.status);

    mkChronosHeader(vb,
                    mTransmitterId,
                    mChronosSeqNum++,
                    false,
                    PROTOCOL_VERSION,
                    ISO_MSG_HEAB);

    appendChronosChecksum(vb);

    sendData(vb, true);
}

void ChronosComm::sendOsem(chronos_osem osem)
{
    VByteArrayLe vb;

    vb.vbAppendUint16(ISO_VALUE_ID_LAT);
    vb.vbAppendUint16(6);
    vb.vbAppendUint48((quint64)(osem.lat * 1e10));
    vb.vbAppendUint16(ISO_VALUE_ID_LON);
    vb.vbAppendUint16(6);
    vb.vbAppendUint48((quint64)(osem.lon * 1e10));
    vb.vbAppendUint16(ISO_VALUE_ID_ALT);
    vb.vbAppendUint16(4);
    vb.vbAppendUint32((quint32)(osem.alt * 1e2));
    vb.vbAppendUint16(ISO_VALUE_ID_GPS_SEC_OF_WEEK);
    vb.vbAppendUint16(4);
    vb.vbAppendUint32(osem.gps_ms_of_week * 4);
    vb.vbAppendUint16(ISO_VALUE_ID_GPS_WEEK);
    vb.vbAppendUint16(2);
    vb.vbAppendUint16(osem.gps_week);

    mkChronosHeader(vb,
                    mTransmitterId,
                    mChronosSeqNum++,
                    false,
                    PROTOCOL_VERSION,
                    ISO_MSG_OSEM);

    appendChronosChecksum(vb);

    sendData(vb, false);
}

void ChronosComm::sendOstm(chronos_ostm ostm)
{
    VByteArrayLe vb;
    vb.vbAppendUint16(ISO_VALUE_ID_STATE_CHANGE_REQ);
    vb.vbAppendUint16(1);
    vb.vbAppendUint8((uint8_t)ostm.armed);

    mkChronosHeader(vb,
                    mTransmitterId,
                    mChronosSeqNum++,
                    false,
                    PROTOCOL_VERSION,
                    ISO_MSG_OSTM);

    appendChronosChecksum(vb);

    sendData(vb, false);
}

void ChronosComm::sendStrt(chronos_strt strt)
{
    VByteArrayLe vb;
    vb.vbAppendUint16(ISO_VALUE_ID_GPS_SEC_OF_WEEK);
    vb.vbAppendUint16(4);
    vb.vbAppendUint32(strt.gps_ms_of_week * 4);
    vb.vbAppendUint16(ISO_VALUE_ID_GPS_WEEK);
    vb.vbAppendUint16(2);
    vb.vbAppendUint32(strt.gps_week);

    mkChronosHeader(vb,
                    mTransmitterId,
                    mChronosSeqNum++,
                    false,
                    PROTOCOL_VERSION,
                    ISO_MSG_STRT);

    appendChronosChecksum(vb);

    sendData(vb, false);
}

void ChronosComm::sendMonr(chronos_monr monr)
{
    VByteArrayLe vb;
    vb.vbAppendUint16(ISO_VALUE_ID_MONR_STRUCT);
    vb.vbAppendUint16(sizeof(chronos_monr));
    vb.vbAppendUint32(monr.gps_ms_of_week * 4);
    vb.vbAppendDouble32(monr.x,1e3);
    vb.vbAppendDouble32(monr.y,1e3);
    vb.vbAppendDouble32(monr.z,1e3);
    vb.vbAppendUint16((quint16)(monr.heading * 1e2));
    vb.vbAppendDouble16(monr.lon_speed,1e2);
    vb.vbAppendDouble16(monr.lat_speed,1e2);
    vb.vbAppendDouble16(monr.lon_acc,1e3);
    vb.vbAppendDouble16(monr.lat_acc,1e3);
    vb.vbAppendUint8(monr.direction);
    vb.vbAppendUint8(monr.status);
    vb.vbAppendUint8(monr.rdyToArm);
    vb.vbAppendUint8(monr.error);

    mkChronosHeader(vb,
                    mTransmitterId,
                    mChronosSeqNum++,
                    false,
                    PROTOCOL_VERSION,
                    ISO_MSG_MONR);

    appendChronosChecksum(vb);

    sendData(vb, true);
}

void ChronosComm::sendInitSup(chronos_init_sup init_sup)
{
    VByteArrayLe vb;
    vb.vbAppendUint16(ISO_VALUE_ID_INIT_SUP_STATUS);
    vb.vbAppendUint16(1);
    vb.vbAppendUint8(init_sup.status);

    mkChronosHeader(vb,
                    mTransmitterId,
                    mChronosSeqNum++,
                    false,
                    PROTOCOL_VERSION,
                    ISO_MSG_INIT_SUP);

    appendChronosChecksum(vb);

    sendData(vb, false);
}

quint8 ChronosComm::transmitterId() const
{
    return mTransmitterId;
}

void ChronosComm::setTransmitterId(const quint8 &transmitterId)
{
    mTransmitterId = transmitterId;
}

quint32 ChronosComm::gpsMsOfWeek()
{
    // Note 18 leap seconds is hard-coded
    return (QDateTime::currentMSecsSinceEpoch() - 315964800LL * 1000LL + 18LL * 1000LL) %
            (24LL * 60LL * 60LL * 7LL * 1000LL);
}

quint32 ChronosComm::gpsWeek()
{
    return (QDateTime::currentMSecsSinceEpoch() - 315964800LL * 1000LL + 18LL * 1000LL) /
            (24LL * 60LL * 60LL * 7LL * 1000LL);
}

quint32 ChronosComm::gpsMsOfWeekToUtcToday(quint64 time)
{
    return ((time + 315964800LL * 1000LL - 18LL * 1000LL) % (24*60*60*1000));
}

void ChronosComm::tcpRx(QByteArray data)
{
    uint8_t sender_id = 0;
    for (char c: data) {
        switch (mTcpState) {
        case 0: // first byte of sync word
            if (!(c == ISO_PART_SYNC_WORD)) {
                qDebug() << "Expected sync word byte 0";
                mTcpState = 0;
                break;
            }
            mTcpState++;
            break;
        case 1: // second byte of sync word
            if (!(c == ISO_PART_SYNC_WORD)) {
                qDebug() << "Expected sync word byte 1";
                mTcpState = 0;
                break;
            }
            mTcpState++;
            break;
        case 2: // Transmitter ID
            sender_id = (uint8_t)c;
            mTcpState++;
            break;
        case 3: // Sequence number
            // ignore for now
            mTcpState++;
            break;
        case 4: // Protocol Version and ACK requenst
            // ignore for now
            mTcpState++;
            break;
        case 5: // Message ID byte 0
            mTcpType = 0;
            mTcpType = (quint8)c;
            mTcpState++;
            break;
        case 6: // Message ID byte 1
            mTcpType |= ((quint8)c) << 8;
            mTcpLen = 0;
            mTcpData.clear();
            mTcpChecksum = 0;
            mTcpState++;
            break;
        case 7: // Message len
            mTcpLen = (quint8)c;
            mTcpState++;
            break;
        case 8:
            mTcpLen |= ((quint8)c) << 8;
            mTcpState++;
            break;
        case 9:
            mTcpLen |= ((quint8)c) << 16;
            mTcpState++;
            break;
        case 10:
            mTcpLen |= ((quint8)c) << 24;
            mTcpState++;
            if (mTcpLen == 0) {
                // Go directly to the checksum state
                mTcpState++;
            }
            break;
        case 11:
            mTcpData.append(c);
            if (mTcpData.size() >= (int)mTcpLen) {
                mTcpState++;
            }
            break;
        case 12: // checksum
            mTcpChecksum = (uint8_t)c;
            mTcpState++;
            break;
        case 13: // checksum
            mTcpChecksum |= ((uint8_t)c) << 8;
            mTcpState = 0;

            if (mTcpChecksum == 0) {
                decodeMsg(mTcpType, mTcpLen, mTcpData, sender_id);
            } else {
                qWarning() << "Invalid checksum";
            }
            break;
        default:
            break;
        }
    }
}

void ChronosComm::tcpConnectionChanged(bool connected, QString address)
{
    emit connectionChanged(connected, address);
}

void ChronosComm::readPendingDatagrams()
{
    while (mUdpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(mUdpSocket->pendingDatagramSize());

        mUdpSocket->readDatagram(datagram.data(), datagram.size(),
                                 &mUdpHostAddress, &mUdpPort);

        VByteArrayLe vb(datagram);

        /*quint16 sync_word   = */ vb.vbPopFrontUint16();
        uint8_t sender_id = vb.vbPopFrontUint8();
        /*quint8  seq_num     = */ vb.vbPopFrontUint8();
        /*quint8  prot_ver    = */ vb.vbPopFrontUint8();  // includes ack bit
        quint16 message_id  = vb.vbPopFrontUint16();
        quint32 message_len = vb.vbPopFrontUint32();
        // TODO: ACK bit.

        if (vb.size() < 2) {
            qDebug() << "Invalid UDP message";
            return;
        }

        quint16 checksum = ((quint8)vb.at(vb.size() - 1) << 8) |
                (quint8)vb.at(vb.size() - 2);

        vb.remove(vb.size() - 2, 2);

        if (checksum == 0) {
            decodeMsg(message_id, message_len, vb, sender_id);
        } else {
            qDebug() << "Checksum Error";
        }
    }
}

void ChronosComm::tcpInputConnected()
{
    qDebug() << "Chronos TCP Connected";
}

void ChronosComm::tcpInputDisconnected()
{
    qDebug() << "Chronos TCP Disconnected";
}

void ChronosComm::tcpInputDataAvailable()
{
    while (mTcpSocket->bytesAvailable() > 0) {
        tcpRx(mTcpSocket->readAll());
    }
}

void ChronosComm::tcpInputError(QAbstractSocket::SocketError socketError)
{
    (void)socketError;

    QString errorStr = mTcpSocket->errorString();
    qWarning() << "Chronos TCP Error:" << errorStr;
    mTcpSocket->close();
}

void ChronosComm::mkChronosHeader(VByteArrayLe &vb, quint8 transmitter_id, quint8 sequence_num,
                                  bool ack_req, quint8 protocol_ver, quint16 message_id)
{   
    // a bit unsure of is the ack req should go to leftmost or rightmost bit.
    quint8 augmented_protocol_ver = protocol_ver << 1;
    if (ack_req) {
        augmented_protocol_ver |= 1;
    }

    VByteArrayLe vb2;
    vb2.vbAppendUint16(ISO_SYNC_WORD);
    vb2.vbAppendUint8(transmitter_id);
    vb2.vbAppendUint8(sequence_num);
    vb2.vbAppendUint8(augmented_protocol_ver);
    vb2.vbAppendUint16(message_id);
    vb2.vbAppendUint32(vb.size());
    vb.prepend(vb2);
}

void ChronosComm::appendChronosChecksum(VByteArrayLe &vb)
{
    vb.vbAppendUint16(0);
}

bool ChronosComm::decodeMsg(quint16 type, quint32 len, QByteArray payload, uint8_t sender_id)
{
    (void)len;

    VByteArrayLe vb(payload);

    switch (type) {
    case ISO_MSG_INIT_SUP: {
        chronos_init_sup init_sup;
        init_sup.status = 0;

        while (!vb.isEmpty()) {
            quint16 value_id  = vb.vbPopFrontUint16();
            quint16 value_len = vb.vbPopFrontUint16();

            switch (value_id) {
            case ISO_VALUE_ID_INIT_SUP_STATUS:
                init_sup.status = vb.vbPopFrontUint8();
                break;
            default:
                vb.remove(0, value_len);
                break;
            }
        }

        emit insupRx(init_sup);
    } break;

    case ISO_MSG_TRAJ: {
        qDebug() << "decoding TRAJ";

        chronos_traj traj;
        QVector<chronos_traj_pt> path;
        QVector<int> ids;
        chronos_traj_pt pt;
        memset(&pt, 0, sizeof(pt));

        while (!vb.isEmpty()) {
            quint16 value_id = vb.vbPopFrontUint16();
            quint16 value_len = vb.vbPopFrontUint16();

            // Repeated value, probably at next point
            if (ids.indexOf(value_id) >= 0) {
                path.append(pt);
                memset(&pt, 0, sizeof(pt));
                ids.clear();
            }

            switch (value_id) {
            case ISO_VALUE_ID_REL_TIME:
                pt.tRel = vb.vbPopFrontUint32(); // TODO: Divide by 4?
                ids.append(value_id);
                break;
            case ISO_VALUE_ID_X_POS:
                pt.x = vb.vbPopFrontDouble32(1e3);
                ids.append(value_id);
                break;
            case ISO_VALUE_ID_Y_POS:
                pt.y = vb.vbPopFrontDouble32(1e3);
                ids.append(value_id);
                break;
            case ISO_VALUE_ID_Z_POS:
                pt.z = vb.vbPopFrontDouble32(1e3);
                ids.append(value_id);
                break;
            case ISO_VALUE_ID_HEADING:
                pt.heading = vb.vbPopFrontDouble16(1e1);
                ids.append(value_id);
                break;
            case ISO_VALUE_ID_LONG_SPEED:
                pt.long_speed = vb.vbPopFrontDouble16(1e2);
                ids.append(value_id);
                break;
            case ISO_VALUE_ID_LAT_SPEED:
                pt.lat_speed = vb.vbPopFrontDouble16(1e2);
                ids.append(value_id);
                break;
            case ISO_VALUE_ID_LONG_ACC:
                pt.long_accel = vb.vbPopFrontDouble16(1e3);
                ids.append(value_id);
                break;
            case ISO_VALUE_ID_LAT_ACC:
                pt.lat_accel = vb.vbPopFrontDouble16(1e3);
                ids.append(value_id);
                break;
            case ISO_VALUE_ID_CURVATURE:
                pt.curvature = vb.vbPopFrontDouble32(3e4);
                ids.append(value_id);
                break;
            case AUX_VALUE_ID_OBJECT_ID:
                traj.object_id = vb.vbPopFrontUint32();
                break;
            case ISO_VALUE_ID_TRAJECTORY_ID:
                traj.traj_id = vb.vbPopFrontUint16();
                break;
            case ISO_VALUE_ID_TRAJECTORY_NAME:
                traj.traj_name = vb.left(64);
                vb.remove(0, 64);
            break;
            case ISO_VALUE_ID_TRAJECTORY_VERSION:
                traj.traj_ver = vb.vbPopFrontUint16();
                break;
            default:
                vb.remove(0, value_len);
                break;
            }
        }

        // Add last point
        if (!ids.isEmpty()) {
            path.append(pt);
        }

        traj.traj_pts = path;
        emit trajRx(traj);
    } break;

    case ISO_MSG_HEAB: {
        chronos_heab heab;
        while (!vb.isEmpty()) {
            quint16 value_id = vb.vbPopFrontUint16();
            quint16 value_len = vb.vbPopFrontUint16();
            switch(value_id) {
            case ISO_VALUE_ID_HEAB_STRUCT:
                heab.gps_ms_of_week = vb.vbPopFrontUint32() / 4;
                heab.status   = vb.vbPopFrontUint8();
                emit heabRx(heab);
                break;
            default:
                qDebug() << "HEAB: Unknown value id:" << value_id;
                vb.remove(0, value_len);
                break;
            }
        }
    } break;

    case ISO_MSG_OSEM: {
        chronos_osem osem;

        while (!vb.isEmpty()) {
            quint16 value_id = vb.vbPopFrontUint16();
            quint16 value_len = vb.vbPopFrontUint16();
            switch (value_id) {
            case ISO_VALUE_ID_LAT:
                osem.lat = vb.vbPopFrontDouble48(1e10);
                break;
            case ISO_VALUE_ID_LON:
                osem.lon = vb.vbPopFrontDouble48(1e10);
                break;
            case ISO_VALUE_ID_ALT:
                osem.alt = vb.vbPopFrontDouble32(1e2);
                break;
            case ISO_VALUE_ID_DateISO8601:
                vb.vbPopFrontUint32(); // pop and throw away
                break;
            case ISO_VALUE_ID_GPS_WEEK:
                osem.gps_week = vb.vbPopFrontUint16();
                break;
            case ISO_VALUE_ID_GPS_SEC_OF_WEEK:
                osem.gps_ms_of_week = vb.vbPopFrontUint32() / 4;
                break;
            default:
                qDebug() << "OSEM: Unknown value id:" << value_id;
                vb.remove(0, value_len);
                break;
            }
        }

        emit osemRx(osem);
    } break;

    case ISO_MSG_OSTM: {
        chronos_ostm ostm;

        while (!vb.isEmpty()) {
            quint16 value_id    = vb.vbPopFrontUint16();
            quint16 value_len = vb.vbPopFrontUint16();
            switch(value_id) {
            case ISO_VALUE_ID_STATE_CHANGE_REQ:
                ostm.armed = vb.vbPopFrontUint8();
                break;

            default:
                qDebug() << "OSTM: Unknown value id";
                vb.remove(0, value_len);
                break;
            }
        }

        emit ostmRx(ostm);
    } break;

    case ISO_MSG_STRT: {
        chronos_strt strt;
        strt.gps_ms_of_week = gpsMsOfWeek();
        strt.gps_week = gpsWeek();

        while (!vb.isEmpty()) {
            quint16 value_id = vb.vbPopFrontUint16();
            quint16 value_len = vb.vbPopFrontUint16();
            switch(value_id) {
            case ISO_VALUE_ID_GPS_SEC_OF_WEEK:
                strt.gps_ms_of_week = vb.vbPopFrontUint32() / 4;
                break;
            case ISO_VALUE_ID_GPS_WEEK:
                strt.gps_week = vb.vbPopFrontUint16();
                break;
            default:
                qDebug() << "STRT: Unknown value id" << value_id;
                vb.remove(0, value_len);
                break;
            }
        }

        qDebug() << "STRT: " << strt.gps_week << strt.gps_ms_of_week;
        emit strtRx(strt);
    } break;

    case ISO_MSG_MONR: {
        chronos_monr monr;
        VByteArrayLe vb(payload);
        while (!vb.isEmpty()) {
            quint16 value_id = vb.vbPopFrontUint16();
            quint16 value_len = vb.vbPopFrontUint16();
            switch(value_id) {
            case ISO_VALUE_ID_MONR_STRUCT:
                monr.gps_ms_of_week = vb.vbPopFrontUint32() / 4;
                monr.x = vb.vbPopFrontDouble32(1e3);
                monr.y = vb.vbPopFrontDouble32(1e3);
                monr.z = vb.vbPopFrontDouble32(1e3);
                monr.heading = ((double)vb.vbPopFrontUint16()) / 1e2;
                monr.lon_speed = vb.vbPopFrontDouble16(1e2);
                monr.lat_speed = vb.vbPopFrontDouble16(1e2);
                monr.lon_acc = vb.vbPopFrontDouble16(1e2);
                monr.lat_acc = vb.vbPopFrontDouble16(1e2);
                monr.direction = vb.vbPopFrontUint8();
                monr.status = vb.vbPopFrontUint8();
                monr.rdyToArm = vb.vbPopFrontUint8();
                monr.error = vb.vbPopFrontUint8();
                monr.sender_id = sender_id;
                emit monrRx(monr);
            default:
                qDebug() << "STRT: Unknown value id" << value_id;
                vb.remove(0, value_len);
                break;
            }
        }
    } break;

    default:
        break;
    }

    return true;
}

void ChronosComm::sendData(QByteArray data, bool isUdp)
{
    if (isUdp && mCommMode != COMM_MODE_SUPERVISOR) {
        mUdpSocket->writeDatagram(data, mUdpHostAddress, mUdpPort ? mUdpPort : 53240);
    } else {
        if (mTcpSocket->state() == QTcpSocket::ConnectedState) {
            mTcpSocket->write(data);
        } else {
            mTcpServer->sendData(data);
        }
    }
}
