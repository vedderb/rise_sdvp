#include "chronos.h"
#include "utility.h"

#include <QDebug>
#include <cmath>
#include <QDateTime>

Chronos::Chronos(QObject *parent) : QObject(parent)
{
    mPacket = 0;
    mTcpServer = new TcpServerSimple(this);
    mUdpSocket = new QUdpSocket(this);
    mStartTimer = new QTimer(this);
    mIsArmed = false;
    mIsStarted = false;
    mSenderId = 1; // TODO: Who assigns these ids ?
    mSequenceNum = 0;

    mUdpHostAddress = QHostAddress("0.0.0.0");
    mUdpPort = 0;

    mTcpState = 0;
    mTcpType = 0;
    mTcpLen = 0;

    mHeabPollCnt = 0;

    connect(mTcpServer, SIGNAL(dataRx(QByteArray)),
            this, SLOT(tcpRx(QByteArray)));
    connect(mTcpServer, SIGNAL(connectionChanged(bool)),
            this, SLOT(tcpConnectionChanged(bool)));
    connect(mUdpSocket, SIGNAL(readyRead()),
            this, SLOT(readPendingDatagrams()));
    connect(mStartTimer, SIGNAL(timeout()),
            this, SLOT(startTimerSlot()));
}

bool Chronos::startServer(PacketInterface *packet)
{
    mPacket = packet;

    bool res = mTcpServer->startServer(53241);

    if (!res) {
        qWarning() << "Starting TCP server failed:" << mTcpServer->errorString();
    }

    if (res) {
        mUdpSocket->close();
        res = mUdpSocket->bind(QHostAddress::Any, 53240);
    }

    if (!res) {
        qWarning() << "Starting UDP server failed:" << mUdpSocket->errorString();
    }

    qDebug() << "Started CHRONOS server";

    if (res && mPacket) {
        connect(mPacket, SIGNAL(stateReceived(quint8,CAR_STATE)),
                this, SLOT(stateReceived(quint8,CAR_STATE)));
    }

    return res;
}

void Chronos::startTimerSlot()
{
    qDebug() << "Starting car";
    mIsStarted = true;

    if (mPacket) {
        mPacket->setApActive(255, true);
    }
}

void Chronos::tcpRx(QByteArray data)
{

    qDebug() << "TCP RX";

    for (char c: data) {
        switch (mTcpState) {
        case 0: // first byte of sync word
            if (!(c == ISO_PART_SYNC_WORD)) {

                // Error: deal with it
                qDebug() << "Expected sync word byte 0";
                break;
            }
            mTcpState++;
            break;
        case 1: // second byte of sync word
            if (!(c == ISO_PART_SYNC_WORD)) {
                qDebug() << "Expected sync word byte 1";
                break;
            }
            mTcpState++;
            break;
        case 2: // Transmitter ID
            // Ignore for now
            mTcpState++;
            break;
        case 3: // Sequence number
            // ignore for now. later detect broken sequences
            mTcpState++;
            break;
        case 4: // Protocol Version and ACK requenst
            // ignore for now. later deal with ack request.
            mTcpState++;
            break;
        case 5: //Message ID byte 0
            mTcpType = 0;
            mTcpType = ((quint8)c) << 8;
            mTcpState++;
            break;
        case 6: //Message ID byte 1
            mTcpType |= (quint8)c;
            mTcpLen = 0;
            mTcpData.clear();
            mTcpChecksum = 0;
            mTcpState++;
            break;
        case 7: // Message len
            mTcpLen = ((quint8)c) << 24;
            mTcpState++;
            break;

        case 8:
            mTcpLen |= ((quint8)c) << 16;
            mTcpState++;
            break;

        case 9:
            mTcpLen |= ((quint8)c) << 8;
            mTcpState++;
            break;

        case 10:
            mTcpLen |= (quint8)c;
            mTcpState++;
            break;

        case 11:
            //qDebug() << "decode case";
            mTcpData.append(c);
            if (mTcpData.size() >= (int)mTcpLen) {
                mTcpState++;
            }
            break;
        case 12: // checksum
            qDebug() << "checksum byte = " << (quint8)c;
            mTcpChecksum = ((uint8_t)c) << 8;
            mTcpState++;
            break;
        case 13: // checksum
            qDebug() << "checksum byte = " << (quint8)c;
            mTcpChecksum |= (uint8_t)c;
            mTcpState = 0;

            if (mTcpChecksum == 0) {
                decodeMsg(mTcpType, mTcpLen, mTcpData);
            }

            break;


        default:
            break;
        }
    }
    qDebug() << "TCP RX" << mTcpState;
}


void Chronos::tcpConnectionChanged(bool connected)
{
    if (connected) {
        qDebug() << "Chronos TCP connection accepted";
    } else {
        qDebug() << "Chronos TCP disconnected";
        mIsArmed = false;
    }
}

/*
void Chronos::readPendingDatagrams()
{
    while (mUdpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(mUdpSocket->pendingDatagramSize());

        mUdpSocket->readDatagram(datagram.data(), datagram.size(),
                                &mUdpHostAddress, &mUdpPort);

        VByteArray vb(datagram);
        quint8 type = vb.vbPopFrontUint8();
        quint16 len = vb.vbPopFrontUint32();
        decodeMsg(type, len, vb);
    }
}
*/

void Chronos::readPendingDatagrams()
{
    while (mUdpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(mUdpSocket->pendingDatagramSize());

        mUdpSocket->readDatagram(datagram.data(), datagram.size(),
                                 &mUdpHostAddress, &mUdpPort);

        VByteArray vb(datagram);

        /*quint16 sync_word   = */ vb.vbPopFrontUint16();
        /*quint8  sender_id   = */ vb.vbPopFrontUint8();
        /*quint8  seq_num     = */ vb.vbPopFrontUint8();
        /*quint8  prot_ver    = */ vb.vbPopFrontUint8();  // includes ack bit
        quint16 message_id  = vb.vbPopFrontUint16();
        quint32 message_len = vb.vbPopFrontUint32();
        // TODO: ACK bit.

        /*
        qDebug() << "UDP CHRONOS HEADER: " << sync_word
                 << sender_id << seq_num << prot_ver
                 << message_id << message_len;
        */

        quint16 checksum = ((quint8)vb.at(vb.size() - 2) << 8) |
                (quint8)vb.at(vb.size() - 1);

        vb.remove(vb.size() -2, 2);

        if (checksum == 0) {
            decodeMsg(message_id, message_len, vb);
        } else {
            qDebug() << "Checksum Error";
        }
    }
}

void Chronos::stateReceived(quint8 id, CAR_STATE state)
{
    (void)id;
    (void)state;

    chronos_monr monr;

    double yaw = state.yaw + 90.0;
    while (yaw < 0) {
        yaw += 360.0;
    }
    while (yaw > 360.0) {
        yaw -= 360.0;
    }

    // monr has 13 fields now.
    monr.gps_time = chronosTimeNow(); // Time has to be fixed
    monr.x = state.px;
    monr.y = state.py;
    monr.z = 0;
    monr.heading = yaw;
    monr.lon_speed = state.speed;
    monr.lat_speed = 0;
    monr.lon_acc = state.accel[1];  // TODO: hmm
    monr.lat_acc = state.accel[0];
    monr.direction = state.speed >= 0 ? 0 : 1;  // 0 means forward now.
    monr.rdyToArm = 1;
    monr.status = mIsArmed ? (mIsStarted ? 4 : 2) : 0;
    // if armed and started -> running (4)
    // if armed and !started -> armed (2)
    // otherwise 0
    // (Simplified)
    monr.error = 0; // TODO: check for errors.

    sendMonr(monr);
}

bool Chronos::decodeMsg(quint16 type, quint32 len, QByteArray payload)
{
    //(void)type;
    (void)len;
    //(void)payload;

    quint16 value_id;
    quint16 content_len;
    (void) content_len;

    VByteArray vb(payload);

    switch (type) {
    case ISO_MSG_DOTM: {
        qDebug() << "decoding DOTM";
        QVector<chronos_dopm_pt> path;

        while (vb.size() > 28) { // only footer left (2 bytes)
            chronos_dopm_pt pt;

            vb.vbPopFrontUint16(); // pop value_id and throw
            vb.vbPopFrontUint16(); // pop content length and throw

            pt.tRel = vb.vbPopFrontUint32();

            vb.vbPopFrontUint16(); // pop value_id and throw
            vb.vbPopFrontUint16(); // pop content length and throw

            pt.x = vb.vbPopFrontDouble32(1e3);

            vb.vbPopFrontUint16(); // pop value_id and throw
            vb.vbPopFrontUint16(); // pop content length and throw

            pt.y = vb.vbPopFrontDouble32(1e3);

            vb.vbPopFrontUint16(); // pop value_id and throw
            vb.vbPopFrontUint16(); // pop content length and throw

            pt.z = vb.vbPopFrontDouble32(1e3);

            vb.vbPopFrontUint16(); // pop value_id and throw
            vb.vbPopFrontUint16(); // pop content length and throw

            pt.heading = vb.vbPopFrontDouble16(1e1);

            vb.vbPopFrontUint16(); // pop value_id and throw
            vb.vbPopFrontUint16(); // pop content length and throw

            pt.speed = vb.vbPopFrontDouble16(1e2); // LONG SPEED

            vb.vbPopFrontUint16(); // pop value_id and throw
            vb.vbPopFrontUint16(); // pop content length and throw

            /*Throwing*/ vb.vbPopFrontInt16(); // LAT SPEED

            vb.vbPopFrontUint16(); // pop value_id and throw
            vb.vbPopFrontUint16(); // pop content length and throw

            pt.accel = vb.vbPopFrontInt16(); // LONG ACC

            vb.vbPopFrontUint16(); // pop value_id and throw
            vb.vbPopFrontUint16(); // pop content length and throw

            /*Throwing*/ vb.vbPopFrontUint16(); // LAT ACC

            pt.mode = 0;
            pt.curvature = 0;
            path.append(pt);
        }
        // Subsample points
        QVector<chronos_dopm_pt> path_reduced;

        if (path.size() > 0) {
            path_reduced.append(path.first());
            for (chronos_dopm_pt pt: path) {
                chronos_dopm_pt pt_last = path_reduced.last();

                if (sqrt((pt.x - pt_last.x) * (pt.x - pt_last.x) +
                         (pt.y - pt_last.y) * (pt.y - pt_last.y) +
                         (pt.z - pt_last.z) * (pt.z - pt_last.z)) > 1.0) {
                    path_reduced.append(pt);
                }
            }

            // Add end point
            chronos_dopm_pt pt = path.last();
            chronos_dopm_pt pt_last = path_reduced.last();
            if (sqrt((pt.x - pt_last.x) * (pt.x - pt_last.x) +
                     (pt.y - pt_last.y) * (pt.y - pt_last.y) +
                     (pt.z - pt_last.z) * (pt.z - pt_last.z)) > 0.01) {
                path_reduced.append(pt);
            }
        }

        processDopm(path_reduced);
    } break;

    case ISO_MSG_HEAB: {
        chronos_heab heab;
        while (vb.size() > 2) {  // only footer left
            value_id    = vb.vbPopFrontUint16();
            content_len = vb.vbPopFrontUint16();
            switch(value_id) {
            case 0x0090:   // heab struct id 0x0090 TODO: Add definition
                heab.gps_time = vb.vbPopFrontUint32();
                heab.status   = vb.vbPopFrontUint8();
                break;
            default:
                qDebug() << "HEAB: Error unknown value id";
                while (vb.size() > 2) vb.vbPopFrontUint8(); // trying to "fix" the rec buffer
                break;
            }
        }
        processHeab(heab);
    } break;

    case ISO_MSG_OSEM: {
        chronos_osem osem;

        while (vb.size() > 2) { // only footer left
            value_id    = vb.vbPopFrontUint16();
            content_len = vb.vbPopFrontUint16();
            switch (value_id) {
            case ISO_VALUE_ID_LAT:
                osem.lat = vb.vbPopFrontDouble48(1e7);
                break;
            case ISO_VALUE_ID_LON:
                osem.lon = vb.vbPopFrontDouble48(1e7);
                break;
            case ISO_VALUE_ID_ALT:
                osem.alt = vb.vbPopFrontDouble32(1e2);
                break;
            case ISO_VALUE_ID_DateISO8601:
                vb.vbPopFrontUint32(); // pop and throw away
                break;
            case ISO_VALUE_ID_GPS_WEEK:
                vb.vbPopFrontUint16(); // pop and throw away
                break;
            case ISO_VALUE_ID_GPS_SEC_OF_WEEK:
                vb.vbPopFrontUint32(); // pop and throw away
                break;
            case 0x0070: // ISO_VALUE_MAX_WAY_DEVIATION
                vb.vbPopFrontUint16(); // pop and throw away
                break;
            case 0x0072: // ISO_VALUE_MAX_LAT_DEVIATION
                vb.vbPopFrontUint16(); // pop and throw away
                break;
            case 0x0074: // ISO_VALUE_MIN_POS_ACCURACY
                vb.vbPopFrontUint16(); // pop and throw away
                break;
            default:
                qDebug() << "OSEM: Error unknown value id";
                while (vb.size() > 2) vb.vbPopFrontUint8(); // trying to "fix" the rec buffer
                break;
            }
        }
        processOsem(osem);
    } break;

    case ISO_MSG_OSTM: {
        chronos_ostm ostm;

        while (vb.size() > 2) {
            value_id    = vb.vbPopFrontUint16();
            content_len = vb.vbPopFrontUint16();
            switch(value_id) {
            case ISO_VALUE_ID_STATE_CHANGE_REQ:
                ostm.armed = vb.vbPopFrontUint8();
                break;

            default:
                qDebug() << "OSTM: Error unknown value id";
                while (vb.size() > 2) vb.vbPopFrontUint8();
                break;
            }
        }
        processOstm(ostm);
    } break;

    case ISO_MSG_STRT: {
        chronos_strt strt;

        while (vb.size() > 2) {
            value_id    = vb.vbPopFrontUint16();
            content_len = vb.vbPopFrontUint16();
            switch(value_id) {
            case ISO_VALUE_ID_GPS_SEC_OF_WEEK:
                strt.start_time = vb.vbPopFrontUint32();
                break;
            case ISO_VALUE_ID_DELAYED_START:
                strt.delay_time = vb.vbPopFrontUint32();
                break;

            default:
                qDebug() << "STRT: Error unknown value id";
                while (vb.size() > 2) vb.vbPopFrontUint8();
                break;
            }
        }
        processStrt(strt);
    } break;

    default:
        break;

    }

    return true;
}

void Chronos::processDopm(QVector<chronos_dopm_pt> path)
{
    qDebug() << "DOPM RX";

    if (mIsArmed) {
        //qDebug() << "Ignored because car is armed";
        qDebug() << "Car is armed but does NOT ignore the DOPM";
        //return;
    }

    if (mPacket) {
//        mPacket->clearRoute(255);

        mRouteLast.clear();
        QList<LocPoint> points;
        bool first = true;
        for (chronos_dopm_pt pt: path) {
            LocPoint lpt;
            lpt.setXY(pt.x, pt.y);
            lpt.setSpeed(pt.speed);
            lpt.setTime(pt.tRel);

            points.append(lpt);
            mRouteLast.append(lpt);

            if (points.size() >= 20) {
                if (first) {
                    mPacket->replaceRoute(255, points);
                    first = false;
                } else {
                    mPacket->setRoutePoints(255, points);
                }

                points.clear();
            }
        }

        if (points.size() > 0) {
            if (first) {
                mPacket->replaceRoute(255, points);
            } else {
                mPacket->setRoutePoints(255, points);
            }
        }
    }
}

void Chronos::processOsem(chronos_osem osem)
{
    qDebug() << "OSEM RX";

    if (mIsArmed) {
        qDebug() << "Ignored because car is armed";
        return;
    }

    mLlhRef[0] = osem.lat;
    mLlhRef[1] = osem.lon;
    mLlhRef[2] = osem.alt;

    // TODO: Rotate route with heading

    if (mPacket) {
        mPacket->setEnuRef(255, mLlhRef);
        mPacket->clearRoute(255);
    }
}

void Chronos::processOstm(chronos_ostm ostm)
{
    qDebug() << "OSTM RX";

    if (ostm.armed == 0x2) {
        mIsArmed = true;
    } else if (ostm.armed == 0x3) {
        mIsArmed = false;
    }

    qDebug() << "Armed:" << mIsArmed;
}

void Chronos::processStrt(chronos_strt strt)
{
    qDebug() << "STRT RX";

    if (!mIsArmed) {
        qDebug() << "Ignored because car is not armed";
        return;
    }

    quint64 cTime = chronosTimeNow();

    qDebug() << strt.start_time << cTime << ((int)strt.start_time - (int)cTime);

    //mStartTimer->setSingleShot(true);
    //mStartTimer->start();
    startTimerSlot();

    qDebug() << "Starting car";

    /*
    if ((strt.ts <= cTime) || (strt.ts - cTime) < 10) {
        startTimerSlot();
    } else {
        mStartTimer->setSingleShot(true);
        mStartTimer->start(strt.ts - chronosTimeNow());
        qDebug() << "Starting car in" << strt.ts - cTime << "ms";
    }
    */
}

void Chronos::processHeab(chronos_heab heab)
{
    //qDebug() << "HEAB RX";
    (void)heab;

    if (mPacket) {
        if (heab.status == 1) {
            mHeabPollCnt++;

            if (mHeabPollCnt >= 4) {
                mHeabPollCnt = 0;
                mPacket->getState(255);
            }
        } else {
            mPacket->setApActive(255, false);
        }
    }
}

void Chronos::processSypm(chronos_sypm sypm)
{
    qDebug() << "SYPM RX:" << sypm.sync_point << sypm.stop_time;
    mSypmLast = sypm;

    // Make sure that the car stops until MTSP is received.
    int closest_sync = 0;
    for (int i = 0;i < mRouteLast.size();i++) {
        if (fabs(mRouteLast.at(i).getTime() - mSypmLast.sync_point) <
                fabs(mRouteLast.at(closest_sync).getTime() - mSypmLast.sync_point)) {
            closest_sync = i;
        }
    }

    if (mPacket) {
        mPacket->setSyncPoint(255, closest_sync, 1000 * 60 * 60, 1000);
        qDebug() << "Car will not start until MTSP is received";
    }
}

void Chronos::processMtsp(chronos_mtsp mtsp)
{
    qDebug() << "MTSP RX";
    (void)mtsp;

    // Find points
    int closest_sync = 0;
    for (int i = 0;i < mRouteLast.size();i++) {
        if (fabs(mRouteLast.at(i).getTime() - mSypmLast.sync_point) <
                fabs(mRouteLast.at(closest_sync).getTime() - mSypmLast.sync_point)) {
            closest_sync = i;
        }
    }

    if (mPacket) {
        mPacket->setSyncPoint(255, closest_sync, mtsp.time_est - chronosTimeNow(),
                              mSypmLast.sync_point - mSypmLast.stop_time);

        qDebug() << closest_sync << mtsp.time_est - chronosTimeNow() <<
                    mSypmLast.sync_point - mSypmLast.stop_time;
    }
}

void Chronos::appendChronosChecksum(VByteArray &vb)
{
    vb.vbAppendUint16(0);
}

VByteArray mkChronosHeader(quint8 transmitter_id,
                           quint8 sequence_num,    // per object sequence num
                           bool ack_req,
                           quint8 protocol_ver,    // 7 bits
                           quint16 message_id,
                           quint32 message_len){

    // a bit unsure of is the ack req should go to leftmost or rightmost bit.
    quint8 augmented_protocol_ver = protocol_ver << 1;
    if (ack_req) augmented_protocol_ver |= 1;

    VByteArray vb;
    vb.vbAppendUint16(ISO_SYNC_WORD);
    vb.vbAppendUint8(transmitter_id);
    vb.vbAppendUint8(sequence_num);
    vb.vbAppendUint8(augmented_protocol_ver);
    vb.vbAppendUint16(message_id);
    vb.vbAppendUint32(message_len);

    return vb;
}

bool Chronos::sendMonr(chronos_monr monr)
{
    if (QString::compare(mUdpHostAddress.toString(), "0.0.0.0") == 0) {
        return false;
    }

    quint32 gps_time = 0;

    VByteArray vb = mkChronosHeader(mSenderId,
                                    mSequenceNum++,
                                    false,
                                    PROTOCOL_VERSION,
                                    ISO_MSG_MONR,
                                    34);

    //vb.vbAppendInt8(CHRONOS_MSG_MONR);
    //vb.vbAppendInt32(24);

    vb.vbAppendUint16(0x80); // VALUE ID MONR STRUCT
    vb.vbAppendUint16(30); // 30 bytes

    // The data is not in "key-value" pairs ....
    vb.vbAppendUint32(gps_time);
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
    vb.vbAppendUint8(monr.error);     // 30 bytes.

    appendChronosChecksum(vb);

    mUdpSocket->writeDatagram(vb, mUdpHostAddress, mUdpPort);

    return true;
}

quint64 Chronos::chronosTimeNow()
{
    QDateTime date = QDateTime::currentDateTime();
    return date.currentMSecsSinceEpoch() - 1072915200000 + 5000;
}

quint32 Chronos::chronosTimeToUtcToday(quint64 time)
{
    return (time % (24*60*60*1000)) - 5000;
}
