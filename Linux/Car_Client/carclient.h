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

#ifndef CARCLIENT_H
#define CARCLIENT_H

#include <QObject>
#include <QSerialPort>
#include <QTcpSocket>
#include <QTimer>
#include <QUdpSocket>
#include "packetinterface.h"
#include "tcpbroadcast.h"

class CarClient : public QObject
{
    Q_OBJECT
public:
    typedef struct {
        bool serialConnect;
        QString serialPort;
        int serialBaud;
        bool nmeaConnect;
        QString nmeaServer;
        int nmeaPort;
    } settings_t;

    explicit CarClient(QObject *parent = 0);
    void connectSerial(QString port, int baudrate = 115200);
    void startRtcmServer(int port = 8200);
    void connectNmea(QString server, int port = 2948);
    void startUdpServer(int port = 8300);

signals:

public slots:
    void serialDataAvailable();
    void serialPortError(QSerialPort::SerialPortError error);
    void packetDataToSend(QByteArray &data);
    void tcpDataAvailable();
    void tcpConnected();
    void tcpDisconnected();
    void rtcmUsbRx(quint8 id, QByteArray data);
    void reconnectTimerSlot();
    void readPendingDatagrams();
    void carPacketRx(const QByteArray &data);

private:
    PacketInterface *mPacketInterface;
    TcpBroadcast *mRtcmBroadcaster;
    QSerialPort *mSerialPort;
    QTcpSocket *mTcpSocket;
    int mCarId;
    QTimer *mReconnectTimer;
    settings_t mSettings;
    bool mTcpConnected;
    QUdpSocket *mUdpSocket;
    QHostAddress mHostAddress;
    int mUdpPort;

};

#endif // CARCLIENT_H
