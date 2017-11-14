/*
    Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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

#ifndef RTCMCLIENT_H
#define RTCMCLIENT_H

#include <QObject>
#include <QTcpSocket>
#include <QSerialPort>
#include "datatypes.h"

class RtcmClient : public QObject
{
    Q_OBJECT
public:
    static RtcmClient* currentMsgHandler;
    static bool gpsOnly;
    static rtcm3_state rtcmState;

    explicit RtcmClient(QObject *parent = 0);
    bool connectNtrip(QString server, QString stream, QString user = "", QString pass = "", int port = 80);
    bool connectTcp(QString server, int port = 80);
    bool connectSerial(QString port, int baudrate = 115200);
    bool isTcpConnected();
    bool isSerialConnected();
    void disconnectTcpNtrip();
    void disconnectSerial();
    void setGpsOnly(bool isGpsOnly);

    void emitRtcmReceived(QByteArray data, int type, bool sync = false);
    void emitRefPosReceived(double lat, double lon, double height, double antenna_height);

    static QByteArray encodeBasePos(double lat, double lon, double height, double antenna_height = 0);

signals:
    void rtcmReceived(QByteArray data, int type, bool sync = false);
    void refPosReceived(double lat, double lon, double height, double antenna_height);

private slots:
    void tcpInputConnected();
    void tcpInputDisconnected();
    void tcpInputDataAvailable();
    void tcpInputError(QAbstractSocket::SocketError socketError);
    void serialDataAvailable();
    void serialPortError(QSerialPort::SerialPortError error);

private:
    QString mNtripUser;
    QString mNtripPassword;
    QString mNtripServer;
    QString mNtripStream;
    QTcpSocket *mTcpSocket;
    QSerialPort *mSerialPort;

};

#endif // RTCMCLIENT_H
