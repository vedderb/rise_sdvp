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

class PacketInterface : public QObject
{
    Q_OBJECT
public:
    explicit PacketInterface(QObject *parent = 0);
    ~PacketInterface();

    bool sendPacket(const unsigned char *data, unsigned int len_packet);
    bool sendPacket(QByteArray data);
    void processData(QByteArray &data);
    void startUdpConnection(QHostAddress ip, int port);
    void stopUdpConnection();
    bool isUdpConnected();
    void getImu(quint8 car);

signals:
    void dataToSend(QByteArray &data);
    void printReceived(int id, QString str);
    void imuReceived(int id, IMU_INFO imu);
    
public slots:
    void timerSlot();
    void readPendingDatagrams();

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
