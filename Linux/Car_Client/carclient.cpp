/*
    Copyright 2016-2017 Benjamin Vedder	benjamin@vedder.se

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
#include <sys/reboot.h>
#include <unistd.h>
#include <QEventLoop>
#include "rtcm3_simple.h"
#include "ublox.h"

namespace {
void rtcm_rx(uint8_t *data, int len, int type) {
    if (CarClient::currentMsgHandler) {
        QByteArray rtcm_data((const char*)data, len);
        CarClient::currentMsgHandler->rtcmRx(rtcm_data, type);
    }
}
}

// Static member initialization
CarClient *CarClient::currentMsgHandler = 0;
rtcm3_state CarClient::rtcmState;

CarClient::CarClient(QObject *parent) : QObject(parent)
{
    mSettings.nmeaConnect = false;
    mSettings.serialConnect = false;
    mSettings.serialRtcmConnect = false;

    mSerialPort = new SerialPort(this);
    mSerialPortRtcm = new QSerialPort(this);
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

    currentMsgHandler = this;
    rtcm3_init_state(&rtcmState);
    rtcm3_set_rx_callback(rtcm_rx, &rtcmState);

    connect(mSerialPort, SIGNAL(serial_data_available()),
            this, SLOT(serialDataAvailable()));
    connect(mSerialPort, SIGNAL(serial_port_error(int)),
            this, SLOT(serialPortError(int)));
    connect(mSerialPortRtcm, SIGNAL(readyRead()),
            this, SLOT(serialRtcmDataAvailable()));
    connect(mSerialPortRtcm, SIGNAL(error(QSerialPort::SerialPortError)),
            this, SLOT(serialRtcmPortError(QSerialPort::SerialPortError)));
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
    connect(mPacketInterface, SIGNAL(rebootSystemReceived(quint8,bool)),
            this, SLOT(rebootSystemReceived(quint8,bool)));
}

CarClient::~CarClient()
{
    logStop();
}

void CarClient::connectSerial(QString port, int baudrate)
{
    if(mSerialPort->isOpen()) {
        mSerialPort->closePort();
    }

    mSerialPort->openPort(port, baudrate);

    mSettings.serialConnect = true;
    mSettings.serialPort = port;
    mSettings.serialBaud = baudrate;

    if(!mSerialPort->isOpen()) {
        return;
    }

    qDebug() << "Serial port connected";

    mPacketInterface->stopUdpConnection();
}

void CarClient::connectSerialRtcm(QString port, int baudrate)
{
    if(mSerialPortRtcm->isOpen()) {
        mSerialPortRtcm->close();
    }

    mSerialPortRtcm->setPortName(port);
    mSerialPortRtcm->open(QIODevice::ReadWrite);

    mSettings.serialRtcmConnect = true;
    mSettings.serialRtcmPort = port;
    mSettings.serialRtcmBaud = baudrate;

    if(!mSerialPortRtcm->isOpen()) {
        return;
    }

    qDebug() << "Serial port RTCM connected";

    mSerialPortRtcm->setBaudRate(baudrate);
    mSerialPortRtcm->setDataBits(QSerialPort::Data8);
    mSerialPortRtcm->setParity(QSerialPort::NoParity);
    mSerialPortRtcm->setStopBits(QSerialPort::OneStop);
    mSerialPortRtcm->setFlowControl(QSerialPort::NoFlowControl);
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

void CarClient::rtcmRx(QByteArray data, int type)
{
    (void)type;

    QString str;

    // Print the packet in the car terminal. NOTE: This is for debugging
    for (int i = 0;i < data.size();i++) {
        str.append(QString().sprintf("%02X ", data.at(i)));
        if (i >= 50) {
            break;
        }
    }

    printTerminal(str);
}

void CarClient::restartRtklib()
{
    QFile ublox("/dev/ublox");
    if (!ublox.exists()) {
        return;
    }

    // Set up ublox before starting RTKLib
    Ublox ubx;
    if (ubx.connectSerial(ublox.fileName())) {
        // Serial port baud rate
        // if it is too low the buffer will overfill and it won't work properly.
        ubx_cfg_prt_uart uart;
        uart.baudrate = 115200;
        uart.in_ubx = true;
        uart.in_nmea = true;
        uart.in_rtcm2 = false;
        uart.in_rtcm3 = true;
        uart.out_ubx = true;
        uart.out_nmea = true;
        uart.out_rtcm3 = true;
        ubx.ubxCfgPrtUart(&uart);

        // Set configuration
        // Switch on RAWX messages, set rate to 5 Hz and time reference to UTC
        ubx.ubxCfgRate(200, 1, 0);
        ubx.ubxCfgMsg(UBX_CLASS_RXM, UBX_RXM_RAWX, 1); // Every second
        ubx.ubxCfgMsg(UBX_CLASS_RXM, UBX_RXM_SFRBX, 1); // Every second

        // Automotive dynamic model
        ubx_cfg_nav5 nav5;
        memset(&nav5, 0, sizeof(ubx_cfg_nav5));
        nav5.apply_dyn = true;
        nav5.dyn_model = 4;
        ubx.ubxCfgNav5(&nav5);

        ubx.disconnectSerial();
    }

    QString user = qgetenv("USER");
    if (user.isEmpty()) {
        user = qgetenv("USERNAME");
    }

    if (user.isEmpty()) {
        QProcess process;
        process.setEnvironment(QProcess::systemEnvironment());
        process.start("whoami");
        waitProcess(process);
        user = QString(process.readAllStandardOutput());
        user.replace("\n", "");
    }

    QProcess process;
    process.setEnvironment(QProcess::systemEnvironment());
    process.start("screen", QStringList() <<
                  "-X" << "-S" << "rtklib" << "quit");
    waitProcess(process);

    QProcess process2;
    process2.setEnvironment(QProcess::systemEnvironment());
    process2.start("killall", QStringList() << "rtkrcv");
    waitProcess(process2);

    QProcess process3;
    process3.setEnvironment(QProcess::systemEnvironment());
    process3.start("screen", QStringList() <<
                   "-d" << "-m" << "-S" << "rtklib" << "bash" << "-c" <<
                   QString("cd /home/%1/rise_sdvp/Linux/RTK/rtkrcv_arm && ./start_ublox ; bash").arg(user));
    waitProcess(process3);
}

void CarClient::serialDataAvailable()
{
    while (mSerialPort->bytesAvailable() > 0) {
        QByteArray data = mSerialPort->readAll();
        mPacketInterface->processData(data);
    }
}

void CarClient::serialPortError(int error)
{
    qDebug() << "Serial error:" << error;

    if(mSerialPort->isOpen()) {
        mSerialPort->closePort();
    }
}

void CarClient::serialRtcmDataAvailable()
{
    while (mSerialPortRtcm->bytesAvailable() > 0) {
        QByteArray data = mSerialPortRtcm->readAll();
        for (int i = 0;i < data.size();i++) {
            rtcm3_input_data((uint8_t)data.at(i), &rtcmState);
        }
    }
}

void CarClient::serialRtcmPortError(QSerialPort::SerialPortError error)
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

        if(mSerialPortRtcm->isOpen()) {
            mSerialPortRtcm->close();
        }
    }
}

void CarClient::packetDataToSend(QByteArray &data)
{
    if (mSerialPort->isOpen()) {
        mSerialPort->writeData(data);
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

    if (mSettings.serialRtcmConnect && !mSerialPortRtcm->isOpen()) {
        qDebug() << "Trying to reconnect RTCM serial...";
        connectSerialRtcm(mSettings.serialRtcmPort, mSettings.serialRtcmBaud);
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
    mCarId = id;

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
        restartRtklib();
    } else {
        qDebug() << "Setting system time failed";
    }
}

void CarClient::rebootSystemReceived(quint8 id, bool powerOff)
{
    (void)id;

    logStop();
    sync();

    if (powerOff) {
        reboot(RB_POWER_OFF);
    } else {
        reboot(RB_AUTOBOOT);
    }
}

void CarClient::printTerminal(QString str)
{
    QByteArray packet;
    packet.clear();
    packet.append((quint8)mCarId);
    packet.append((char)CMD_PRINTF);
    packet.append(str.toLocal8Bit());
    carPacketRx(mCarId, CMD_PRINTF, packet);
}

bool CarClient::waitProcess(QProcess &process, int timeoutMs)
{
    bool killed = false;

    process.waitForStarted();

    QEventLoop loop;
    QTimer timeoutTimer;
    timeoutTimer.setSingleShot(true);
    timeoutTimer.start(timeoutMs);
    connect(&process, SIGNAL(finished(int)), &loop, SLOT(quit()));
    connect(&timeoutTimer, SIGNAL(timeout()), &loop, SLOT(quit()));
    loop.exec();

    if (process.state() == QProcess::Running) {
        process.kill();
        process.waitForFinished();
        killed = true;
    }

    return !killed;
}
