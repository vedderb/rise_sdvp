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

#include "rtcmclient.h"
#include "rtcm3_simple.h"
#include <QDebug>
#include <QMessageBox>

namespace {
void rtcm_rx(uint8_t *data, int len, int type) {
    if (RtcmClient::currentMsgHandler && type != 1002 && type != 1004 && type != 1010 && type != 1012) {
        QByteArray rtcm_data((const char*)data, len);
        RtcmClient::currentMsgHandler->emitRtcmReceived(rtcm_data, type);
    }
}

void rtcm_rx_1006(rtcm_ref_sta_pos_t *pos) {
    if (RtcmClient::currentMsgHandler) {
        RtcmClient::currentMsgHandler->emitRefPosReceived(
                    pos->lat, pos->lon, pos->height, pos->ant_height);
    }
}

void rtcm_rx_obs(rtcm_obs_header_t *header, rtcm_obs_t *obs, int obs_num) {
    // Don't send empty observations.
    if (RtcmClient::currentMsgHandler && obs_num > 0) {
        if (header->type == 1002 || header->type == 1004) {
            // Re-encode to 1002 since we don't care about L2 for now.
            static uint8_t data[2048];
            int len;
            int type = 1002;

            // Set sync to 0 since no more observations are coming. Otherwise
            // RTKLIB or the ublox will wait for other observations that are
            // thrown away and RTK won't work.
            if (RtcmClient::gpsOnly) {
                header->sync = 0;
            }

            rtcm3_encode_1002(header, obs, obs_num, data, &len);

            QByteArray rtcm_data((const char*)data, len);
            RtcmClient::currentMsgHandler->emitRtcmReceived(rtcm_data, type, header->sync);
        } if ((header->type == 1010 || header->type == 1012) && !RtcmClient::gpsOnly) {
            // Re-encode to 1010 since we don't care about L2 for now.
            static uint8_t data[2048];
            int len;
            int type = 1010;

            rtcm3_encode_1010(header, obs, obs_num, data, &len);

            QByteArray rtcm_data((const char*)data, len);
            RtcmClient::currentMsgHandler->emitRtcmReceived(rtcm_data, type, header->sync);
        }
    }

    if (obs_num == 0) {
        qDebug() << "Empty observation received";
    }
}
}

// Static member initialization
RtcmClient *RtcmClient::currentMsgHandler = 0;
bool RtcmClient::gpsOnly = false;
rtcm3_state RtcmClient::rtcmState;

RtcmClient::RtcmClient(QObject *parent) : QObject(parent)
{
    mTcpSocket = new QTcpSocket(this);
    mSerialPort = new QSerialPort(this);

    qRegisterMetaType<rtcm_obs_header_t>("rtcm_obs_header_t");
    qRegisterMetaType<rtcm_obs_t>("rtcm_obs_gps_t");

    currentMsgHandler = this;
    rtcm3_init_state(&rtcmState);
    rtcm3_set_rx_callback(rtcm_rx, &rtcmState);
    rtcm3_set_rx_callback_1005_1006(rtcm_rx_1006, &rtcmState);
    rtcm3_set_rx_callback_obs(rtcm_rx_obs, &rtcmState);

    connect(mTcpSocket, SIGNAL(readyRead()), this, SLOT(tcpInputDataAvailable()));
    connect(mTcpSocket, SIGNAL(connected()), this, SLOT(tcpInputConnected()));
    connect(mTcpSocket, SIGNAL(disconnected()),
            this, SLOT(tcpInputDisconnected()));
    connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(tcpInputError(QAbstractSocket::SocketError)));
    connect(mSerialPort, SIGNAL(readyRead()), this, SLOT(serialDataAvailable()));
    connect(mSerialPort, SIGNAL(error(QSerialPort::SerialPortError)),
            this, SLOT(serialPortError(QSerialPort::SerialPortError)));
}

bool RtcmClient::connectNtrip(QString server, QString stream, QString user, QString pass, int port)
{
    mNtripUser = user;
    mNtripPassword = pass;
    mNtripStream = stream;
    mNtripServer = server;

    mTcpSocket->abort();
    mTcpSocket->connectToHost(server, port);

    return true;
}

bool RtcmClient::connectTcp(QString server, int port)
{
    mNtripUser = "";
    mNtripPassword = "";
    mNtripStream = "";
    mNtripServer = "";

    mTcpSocket->abort();
    mTcpSocket->connectToHost(server, port);

    return true;
}

bool RtcmClient::connectSerial(QString port, int baudrate)
{
    if(mSerialPort->isOpen()) {
        mSerialPort->close();
    }

    mSerialPort->setPortName(port);
    mSerialPort->open(QIODevice::ReadWrite);

    if(!mSerialPort->isOpen()) {
        return false;
    }

    mSerialPort->setBaudRate(baudrate);
    mSerialPort->setDataBits(QSerialPort::Data8);
    mSerialPort->setParity(QSerialPort::NoParity);
    mSerialPort->setStopBits(QSerialPort::OneStop);
    mSerialPort->setFlowControl(QSerialPort::NoFlowControl);

    return true;
}

bool RtcmClient::isTcpConnected()
{
    // If no stream is selected we have simply connected to a TCP server.
    return mTcpSocket->isOpen();
}

bool RtcmClient::isSerialConnected()
{
    return mSerialPort->isOpen();
}

void RtcmClient::disconnectTcpNtrip()
{
    mTcpSocket->close();
}

void RtcmClient::disconnectSerial()
{
    mSerialPort->close();
}

void RtcmClient::setGpsOnly(bool isGpsOnly)
{
    gpsOnly = isGpsOnly;
}

void RtcmClient::emitRtcmReceived(QByteArray data, int type, bool sync)
{
    emit rtcmReceived(data, type, sync);
}

void RtcmClient::emitRefPosReceived(double lat, double lon, double height, double antenna_height)
{
    emit refPosReceived(lat, lon, height, antenna_height);
}

QByteArray RtcmClient::encodeBasePos(double lat, double lon, double height, double antenna_height)
{
    rtcm_ref_sta_pos_t pos;
    int len;

    pos.staid = 0;
    pos.lat = lat;
    pos.lon = lon;
    pos.height = height;
    pos.ant_height = antenna_height;

    quint8 buffer[40];
    rtcm3_encode_1006(pos, buffer, &len);

    QByteArray rtcm_data((const char*)buffer, len);
    return rtcm_data;
}

void RtcmClient::tcpInputConnected()
{
    qDebug() << "RTCM TCP connected";

    // If no stream is selected we have simply connected to a TCP server.
    if (mNtripStream.size() > 0) {
        QString msg;
        msg += "GET /" + mNtripStream + " HTTP/1.1\r\n";
        msg += "User-Agent: NTRIP " + mNtripServer + "\r\n";

        if (mNtripUser.size() > 0 || mNtripPassword.size() > 0) {
            QString authStr = mNtripUser + ":" + mNtripPassword;
            QByteArray auth;
            auth.append(authStr);
            msg += "Authorization: Basic " + auth.toBase64() + "\r\n";
        }

        msg += "Accept: */*\r\nConnection: close\r\n";
        msg += "\r\n";

        mTcpSocket->write(msg.toLocal8Bit());
    }
}

void RtcmClient::tcpInputDisconnected()
{
    qDebug() << "RTCM TCP disconnected";
}

void RtcmClient::tcpInputDataAvailable()
{
    QByteArray data =  mTcpSocket->readAll();

    for (int i = 0;i < data.size();i++) {
        int ret = rtcm3_input_data(data.at(i), &rtcmState);
        if (ret == -1 || ret == -2) {
            //qWarning() << "RTCM decode error:" <<  ret;
        }
    }
}

void RtcmClient::tcpInputError(QAbstractSocket::SocketError socketError)
{
    (void)socketError;

    QString errorStr = mTcpSocket->errorString();
    qWarning() << "RTCM TcpError:" << errorStr;
    QMessageBox::warning(0, "TCP Error", errorStr);

    mTcpSocket->close();
}

void RtcmClient::serialDataAvailable()
{
    while (mSerialPort->bytesAvailable() > 0) {
        QByteArray data = mSerialPort->readAll();

        for (int i = 0;i < data.size();i++) {
            int ret = rtcm3_input_data(data.at(i), &rtcmState);
            if (ret == -1 || ret == -2) {
                //qWarning() << "RTCM decode error:" <<  ret;
            }
        }
    }
}

void RtcmClient::serialPortError(QSerialPort::SerialPortError error)
{
    QString message;
    switch (error) {
    case QSerialPort::NoError:
        break;
    case QSerialPort::DeviceNotFoundError:
        message = tr("Device not found");
        break;
    case QSerialPort::OpenError:
        message = tr("Can't open device");
        break;
    case QSerialPort::NotOpenError:
        message = tr("Not open error");
        break;
    case QSerialPort::ResourceError:
        message = tr("Port disconnected");
        break;
    case QSerialPort::PermissionError:
        message = tr("Permission error");
        break;
    case QSerialPort::UnknownError:
        message = tr("Unknown error");
        break;
    default:
        message = "Error number: " + QString::number(error);
        break;
    }

    if(!message.isEmpty()) {
        qDebug() << "Serial error:" << message;

        if(mSerialPort->isOpen()) {
            mSerialPort->close();
        }
    }
}

