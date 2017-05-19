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
#include <QFile>
#include <QProcess>
#include "packetinterface.h"
#include "tcpbroadcast.h"
#include "serialport.h"
#include "ublox.h"

class CarClient : public QObject
{
    Q_OBJECT
public:
    static CarClient* currentMsgHandler;
    static rtcm3_state rtcmState;

    typedef struct {
        bool serialConnect;
        QString serialPort;
        int serialBaud;
        bool serialRtcmConnect;
        QString serialRtcmPort;
        int serialRtcmBaud;
        bool nmeaConnect;
        QString nmeaServer;
        int nmeaPort;
    } settings_t;

    explicit CarClient(QObject *parent = 0);
    ~CarClient();
    void connectSerial(QString port, int baudrate = 115200);
    void connectSerialRtcm(QString port, int baudrate = 9600);
    void startRtcmServer(int port = 8200);
    void startUbxServer(int port = 8210);
    void connectNmea(QString server, int port = 2948);
    void startUdpServer(int port = 8300);
    bool enableLogging(QString directory);
    void logStop();
    void rtcmRx(QByteArray data, int type);
    void restartRtklib();

signals:

public slots:
    void serialDataAvailable();
    void serialPortError(int error);
    void serialRtcmDataAvailable();
    void serialRtcmPortError(QSerialPort::SerialPortError error);
    void packetDataToSend(QByteArray &data);
    void tcpDataAvailable();
    void tcpConnected();
    void tcpDisconnected();
    void rtcmUsbRx(quint8 id, QByteArray data);
    void reconnectTimerSlot();
    void logFlushTimerSlot();
    void readPendingDatagrams();
    void carPacketRx(quint8 id, CMD_PACKET cmd, const QByteArray &data);
    void logLineUsbReceived(quint8 id, QString str);
    void systemTimeReceived(quint8 id, qint32 sec, qint32 usec);
    void rebootSystemReceived(quint8 id, bool powerOff);
    void ubxRx(const QByteArray &data);
    void rxRawx(ubx_rxm_rawx rawx);

private:
    PacketInterface *mPacketInterface;
    TcpBroadcast *mRtcmBroadcaster;
    TcpBroadcast *mUbxBroadcaster;
    SerialPort *mSerialPort;
    QSerialPort *mSerialPortRtcm;
    QTcpSocket *mTcpSocket;
    int mCarId;
    QTimer *mReconnectTimer;
    QTimer *mLogFlushTimer;
    settings_t mSettings;
    bool mTcpConnected;
    QUdpSocket *mUdpSocket;
    QHostAddress mHostAddress;
    int mUdpPort;
    QFile mLog;
    Ublox *mUblox;

    void printTerminal(QString str);
    bool waitProcess(QProcess &process, int timeoutMs = 300000);

};

#endif // CARCLIENT_H
