/*
    Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#ifndef PACKETINTERFACE_H
#define PACKETINTERFACE_H

#include <QObject>
#include <QTimer>
#include <QVector>
#include <QUdpSocket>
#include "datatypes.h"
#include "locpoint.h"

class PacketInterface : public QObject
{
    Q_OBJECT
public:
    explicit PacketInterface(QObject *parent = 0);
    ~PacketInterface();

    bool sendPacket(const unsigned char *data, unsigned int len_packet);
    bool sendPacket(QByteArray data);
    bool sendPacketAck(const unsigned char *data, unsigned int len_packet,
                       int retries, int timeoutMs = 200);
    void processData(QByteArray &data);
    void startUdpConnection(QHostAddress ip, int port);
    void stopUdpConnection();
    bool isUdpConnected();
    void getState(quint8 id);
    bool setRoutePoints(quint8 id, QList<LocPoint> points, int retries = 10);
    bool clearRoute(quint8 id, int retries = 10);
    bool setApActive(quint8 id, bool active, int retries = 10);

signals:
    void dataToSend(QByteArray &data);
    void printReceived(quint8 id, QString str);
    void stateReceived(quint8 id, CAR_STATE state);
    void vescFwdReceived(quint8 id, QByteArray data);
    void ackReceived(quint8 id, CMD_PACKET cmd, QString msg);
    void rtcmUsbReceived(quint8 id, QByteArray data);
    void nmeaRadioReceived(quint8 id, QByteArray data);
    
public slots:
    void timerSlot();
    void readPendingDatagrams();
    void sendTerminalCmd(quint8 id, QString cmd);
    void forwardVesc(quint8 id, QByteArray data);
    void setRcControlCurrent(quint8 id, double duty, double steering);
    void setRcControlDuty(quint8 id, double duty, double steering);
    void setPos(quint8 id, double x, double y, double angle);
    void setServoDirect(quint8 id, double value);
    void sendRtcmUsb(quint8 id, QByteArray rtcm_msg);
    void sendNmeaRadio(quint8 id, QByteArray nmea_msg);

private:
    unsigned short crc16(const unsigned char *buf, unsigned int len);
    void processPacket(const unsigned char *data, int len);

    QTimer *mTimer;
    quint8 *mSendBuffer;
    QUdpSocket *mUdpSocket;
    QHostAddress mHostAddress;
    int mUdpPort;

    // Packet state machine variables
    static const unsigned int mMaxBufferLen = 4096;
    int mRxTimer;
    int mRxState;
    unsigned int mPayloadLength;
    unsigned char mRxBuffer[mMaxBufferLen];
    unsigned int mRxDataPtr;
    unsigned char mCrcLow;
    unsigned char mCrcHigh;
    
};

#endif // PACKETINTERFACE_H
