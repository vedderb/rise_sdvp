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

#include "carclient.h"
#include <QDebug>
#include <QDateTime>
#include <QDir>
#include <sys/time.h>

CarClient::CarClient(QObject *parent) : QObject(parent)
{
    mSettings.nmeaConnect = false;
    mSettings.serialConnect = false;

    mSerialPort = new QSerialPort(this);
    mPacketInterface = new PacketInterface(this);
    mRtcmBroadcaster = new TcpBroadcast(this);
    mTcpSocket = new QTcpSocket(this);
    mCarId = 255;
    mReconnectTimer = new QTimer(this);
    mReconnectTimer->start(2000);
    mTcpConnected = false;
    mLogFlushTimer = new QTimer(this);
    mLogFlushTimer->start(2000);

    mHostAddress = QHostAddress("0.0.0.0");
    mUdpPort = 0;
    mUdpSocket = new QUdpSocket(this);

    connect(mSerialPort, SIGNAL(readyRead()),
            this, SLOT(serialDataAvailable()));
    connect(mSerialPort, SIGNAL(error(QSerialPort::SerialPortError)),
            this, SLOT(serialPortError(QSerialPort::SerialPortError)));
    connect(mTcpSocket, SIGNAL(readyRead()), this, SLOT(tcpDataAvailable()));
    connect(mTcpSocket, SIGNAL(connected()), this, SLOT(tcpConnected()));
    connect(mTcpSocket, SIGNAL(disconnected()), this, SLOT(tcpDisconnected()));
    connect(mPacketInterface, SIGNAL(rtcmUsbReceived(quint8,QByteArray)),
            this, SLOT(rtcmUsbRx(quint8,QByteArray)));
    connect(mPacketInterface, SIGNAL(dataToSend(QByteArray&)),
            this, SLOT(packetDataToSend(QByteArray&)));
    connect(mReconnectTimer, SIGNAL(timeout()),
            this, SLOT(reconnectTimerSlot()));
    connect(mUdpSocket, SIGNAL(readyRead()),
            this, SLOT(readPendingDatagrams()));
    connect(mPacketInterface, SIGNAL(packetReceived(quint8,CMD_PACKET,QByteArray)),
            this, SLOT(carPacketRx(quint8,CMD_PACKET,QByteArray)));
    connect(mPacketInterface, SIGNAL(logLineUsbReceived(quint8,QString)),
            this, SLOT(logLineUsbReceived(quint8,QString)));
    connect(mLogFlushTimer, SIGNAL(timeout()),
            this, SLOT(logFlushTimerSlot()));
    connect(mPacketInterface, SIGNAL(systemTimeReceived(quint8,qint32,qint32)),
            this, SLOT(systemTimeReceived(quint8,qint32,qint32)));
}

CarClient::~CarClient()
{
    logStop();
}

void CarClient::connectSerial(QString port, int baudrate)
{
    if(mSerialPort->isOpen()) {
        mSerialPort->close();
    }

    mSerialPort->setPortName(port);
    mSerialPort->open(QIODevice::ReadWrite);

    mSettings.serialConnect = true;
    mSettings.serialPort = port;
    mSettings.serialBaud = baudrate;

    if(!mSerialPort->isOpen()) {
        return;
    }

    qDebug() << "Serial port connected";

    mSerialPort->setBaudRate(baudrate);
    mSerialPort->setDataBits(QSerialPort::Data8);
    mSerialPort->setParity(QSerialPort::NoParity);
    mSerialPort->setStopBits(QSerialPort::OneStop);
    mSerialPort->setFlowControl(QSerialPort::NoFlowControl);

    mPacketInterface->stopUdpConnection();
}

void CarClient::startRtcmServer(int port)
{
    mRtcmBroadcaster->startTcpServer(port);
}

void CarClient::connectNmea(QString server, int port)
{
    mTcpSocket->close();
    mTcpSocket->connectToHost(server, port);

    mSettings.nmeaConnect = true;
    mSettings.nmeaServer = server;
    mSettings.nmeaPort = port;
}

void CarClient::startUdpServer(int port)
{
    mUdpPort = port + 1;
    mUdpSocket->close();
    mUdpSocket->bind(QHostAddress::Any, port);
}

bool CarClient::enableLogging(QString directory)
{
    if (mLog.isOpen()) {
        mLog.close();
    }

    QString name = QDateTime::currentDateTime().
            toString("LOG_yyyy-MM-dd_hh.mm.ss.log");

    QDir dir;
    dir.mkpath(directory);

    mLog.setFileName(directory + "/" + name);
    return mLog.open(QIODevice::ReadWrite | QIODevice::Truncate);
}

void CarClient::logStop()
{
    if (mLog.isOpen()) {
        qDebug() << "Closing log:" << mLog.fileName();
        mLog.close();
    } else {
        qDebug() << "Log not open";
    }
}

void CarClient::serialDataAvailable()
{
    while (mSerialPort->bytesAvailable() > 0) {
        QByteArray data = mSerialPort->readAll();
        mPacketInterface->processData(data);
    }
}

void CarClient::serialPortError(QSerialPort::SerialPortError error)
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

void CarClient::packetDataToSend(QByteArray &data)
{
    if (mSerialPort->isOpen()) {
        mSerialPort->write(data);
    }
}

void CarClient::tcpDataAvailable()
{
    while (!mTcpSocket->atEnd()) {
        QByteArray data = mTcpSocket->readAll();
        // TODO: Collect data and split at newline. (seems ok)
        mPacketInterface->sendNmeaRadio(mCarId, data);
    }
}

void CarClient::tcpConnected()
{
    qDebug() << "NMEA TCP Connected";
    mTcpConnected = true;
}

void CarClient::tcpDisconnected()
{
    qDebug() << "NMEA TCP Disconnected";
    mTcpConnected = false;
}

void CarClient::rtcmUsbRx(quint8 id, QByteArray data)
{
    mCarId = id;
    mRtcmBroadcaster->broadcastData(data);
}

void CarClient::reconnectTimerSlot()
{
    // Try to reconnect if the connections are lost
    if (mSettings.serialConnect && !mSerialPort->isOpen()) {
        qDebug() << "Trying to reconnect serial...";
        connectSerial(mSettings.serialPort, mSettings.serialBaud);
    }

    if (mSettings.nmeaConnect && !mTcpConnected) {
        qDebug() << "Trying to reconnect nmea tcp...";
        connectNmea(mSettings.nmeaServer, mSettings.nmeaPort);
    }
}

void CarClient::logFlushTimerSlot()
{
    if (mLog.isOpen()) {
        mLog.flush();
    }
}

void CarClient::readPendingDatagrams()
{
    while (mUdpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(mUdpSocket->pendingDatagramSize());
        quint16 senderPort;

        mUdpSocket->readDatagram(datagram.data(), datagram.size(),
                                &mHostAddress, &senderPort);

        mPacketInterface->sendPacket(datagram);
    }
}

void CarClient::carPacketRx(quint8 id, CMD_PACKET cmd, const QByteArray &data)
{
    (void)id;

    if (QString::compare(mHostAddress.toString(), "0.0.0.0") != 0) {
        if (cmd != CMD_LOG_LINE_USB) {
            mUdpSocket->writeDatagram(data, mHostAddress, mUdpPort);
        }
    }
}

void CarClient::logLineUsbReceived(quint8 id, QString str)
{
    (void)id;

    if (mLog.isOpen()) {
        mLog.write(str.toLocal8Bit());
    }
}

void CarClient::systemTimeReceived(quint8 id, qint32 sec, qint32 usec)
{
    (void)id;

    struct timeval now;
    int rc;

    now.tv_sec = sec;
    now.tv_usec = usec;
    rc = settimeofday(&now, NULL);

    if(rc == 0) {
        qDebug() << "Sucessfully set system time";
    } else {
        qDebug() << "Setting system time failed";
    }

}
