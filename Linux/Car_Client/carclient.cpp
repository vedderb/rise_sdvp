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
#include <QCoreApplication>
#include <QNetworkInterface>
#include <QBuffer>
#include "rtcm3_simple.h"

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
    mUbxBroadcaster = new TcpBroadcast(this);
    mLogBroadcaster = new TcpBroadcast(this);
    mUblox = new Ublox(this);
    mTcpSocket = new QTcpSocket(this);
    mTcpServer = new TcpServerSimple(this);
    mRtcmClient = new RtcmClient(this);
    mCarId = 255;
    mReconnectTimer = new QTimer(this);
    mReconnectTimer->start(2000);
    mTcpConnected = false;
    mLogFlushTimer = new QTimer(this);
    mLogFlushTimer->start(2000);
    mRtklibRunning = false;
    mBatteryCells = 10;

    mHostAddress = QHostAddress("0.0.0.0");
    mUdpPort = 0;
    mUdpSocket = new QUdpSocket(this);

    mRtcmBaseLat = 0.0;
    mRtcmBaseLon = 0.0;
    mRtcmBaseHeight = 0.0;
    mRtcmSendBase = false;

    currentMsgHandler = this;
    rtcm3_init_state(&rtcmState);
    rtcm3_set_rx_callback(rtcm_rx, &rtcmState);

    mTcpServer->setUsePacket(true);

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
    connect(mUblox, SIGNAL(ubxRx(QByteArray)), this, SLOT(ubxRx(QByteArray)));
    connect(mUblox, SIGNAL(rxRawx(ubx_rxm_rawx)), this, SLOT(rxRawx(ubx_rxm_rawx)));
    connect(mTcpServer->packet(), SIGNAL(packetReceived(QByteArray&)),
            this, SLOT(tcpRx(QByteArray&)));
    connect(mTcpServer, SIGNAL(connectionChanged(bool,QString)),
            this, SLOT(tcpConnectionChanged(bool,QString)));
    connect(mRtcmClient, SIGNAL(rtcmReceived(QByteArray,int,bool)),
            this, SLOT(rtcmReceived(QByteArray,int,bool)));
    connect(mPacketInterface, SIGNAL(logEthernetReceived(quint8,QByteArray)),
            this, SLOT(logEthernetReceived(quint8,QByteArray)));

#if HAS_CAMERA
    mCameraJpgQuality = -1;
    mCameraSkipFrames = 0;
    mCameraSkipFrameCnt = 0;
    mCameraNoAckCnt = 0;
    mCamera = new Camera(this);
    connect(mCamera->video(), SIGNAL(imageCaptured(QImage)),
            this, SLOT(cameraImageCaptured(QImage)));
#endif
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
    mPacketInterface->getState(255); // To get car ID
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

void CarClient::startUbxServer(int port)
{
    mUbxBroadcaster->startTcpServer(port);
}

void CarClient::startLogServer(int port)
{
    mLogBroadcaster->startTcpServer(port);
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

bool CarClient::startTcpServer(int port)
{
    bool res = mTcpServer->startServer(port);

    if (!res) {
        qWarning() << "Starting TCP server failed:" << mTcpServer->errorString();
    }

    return res;
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
        mRtklibRunning = false;
        return;
    }

    mRtklibRunning = true;

    mUblox->disconnectSerial();

    if (mUblox->connectSerial(ublox.fileName())) {
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
        mUblox->ubxCfgPrtUart(&uart);

        // Set configuration
        // Switch on RAWX and NMEA messages, set rate to 1 Hz and time reference to UTC
        mUblox->ubxCfgRate(200, 1, 0);
        mUblox->ubxCfgMsg(UBX_CLASS_RXM, UBX_RXM_RAWX, 1); // Every second
        mUblox->ubxCfgMsg(UBX_CLASS_RXM, UBX_RXM_SFRBX, 1); // Every second
        mUblox->ubxCfgMsg(UBX_CLASS_NMEA, UBX_NMEA_GGA, 1); // Every second

        // Automotive dynamic model
        ubx_cfg_nav5 nav5;
        memset(&nav5, 0, sizeof(ubx_cfg_nav5));
        nav5.apply_dyn = true;
        nav5.dyn_model = 4;
        mUblox->ubxCfgNav5(&nav5);

        // Time pulse configuration
        ubx_cfg_tp5 tp5;
        memset(&tp5, 0, sizeof(ubx_cfg_tp5));
        tp5.active = true;
        tp5.polarity = true;
        tp5.alignToTow = true;
        tp5.lockGnssFreq = true;
        tp5.lockedOtherSet = true;
        tp5.syncMode = false;
        tp5.isFreq = false;
        tp5.isLength = true;
        tp5.freq_period = 1000000;
        tp5.pulse_len_ratio = 0;
        tp5.freq_period_lock = 1000000;
        tp5.pulse_len_ratio_lock = 100000;
        tp5.gridUtcGnss = 0;
        tp5.user_config_delay = 0;
        tp5.rf_group_delay = 0;
        tp5.ant_cable_delay = 50;
        mUblox->ubloxCfgTp5(&tp5);
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

PacketInterface *CarClient::packetInterface()
{
    return mPacketInterface;
}

bool CarClient::isRtklibRunning()
{
    return mRtklibRunning;
}

quint8 CarClient::carId()
{
    return mCarId;
}

void CarClient::setCarId(quint8 id)
{
    mCarId = id;
}

void CarClient::connectNtrip(QString server, QString stream, QString user, QString pass, int port)
{
    mRtcmClient->connectNtrip(server, stream, user, pass, port);
}

void CarClient::setSendRtcmBasePos(bool send, double lat, double lon, double height)
{
    mRtcmSendBase = send;
    mRtcmBaseLat = lat;
    mRtcmBaseLon = lon;
    mRtcmBaseHeight = height;
}

void CarClient::rebootSystem(bool powerOff)
{
    // https://askubuntu.com/questions/159007/how-do-i-run-specific-sudo-commands-without-a-password
    QStringList args;
    QString cmd = "sudo";

    if (powerOff) {
        args << "shutdown" << "-h" << "now";
    } else {
        args << "reboot";
    }

    QProcess process;
    process.setEnvironment(QProcess::systemEnvironment());
    process.start(cmd, args);
    waitProcess(process);

    qApp->quit();
}

QVariantList CarClient::getNetworkAddresses()
{
    QVariantList res;

    for(QHostAddress a: QNetworkInterface::allAddresses()) {
        if(!a.isLoopback()) {
            if (a.protocol() == QAbstractSocket::IPv4Protocol) {
                res << a.toString();
            }
        }
    }

    return res;
}

int CarClient::getBatteryCells()
{
    return mBatteryCells;
}

void CarClient::setBatteryCells(int cells)
{
    mBatteryCells = cells;
}

void CarClient::addSimulatedCar(int id)
{
    CarSim *car = new CarSim(this);
    car->setId(id);
    connect(car, SIGNAL(dataToSend(QByteArray)), this, SLOT(processCarData(QByteArray)));
    mSimulatedCars.append(car);
}

CarSim *CarClient::getSimulatedCar(int id)
{
    CarSim *sim = 0;

    for (CarSim *s: mSimulatedCars) {
        if (s->id() == id) {
            sim = s;
            break;
        }
    }

    return sim;
}

void CarClient::serialDataAvailable()
{
    while (mSerialPort->bytesAvailable() > 0) {
        processCarData(mSerialPort->readAll());
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
    // This is a packet from RControlStation going to the car.
    // Inspect data and possibly process it here.

    bool packetConsumed = false;

    VByteArray vb(data);
    vb.remove(0, data[0]);

    quint8 id = vb.vbPopFrontUint8();
    CMD_PACKET cmd = (CMD_PACKET)vb.vbPopFrontUint8();
    vb.chop(3);

    (void)id;

    if (id == mCarId || id == 255) {
        if (cmd == CMD_CAMERA_STREAM_START) {
#if HAS_CAMERA
            int camera = vb.vbPopFrontInt16();
            mCameraJpgQuality = vb.vbPopFrontInt16();
            int width = vb.vbPopFrontInt16();
            int height = vb.vbPopFrontInt16();
            int fps = vb.vbPopFrontInt16();
            mCameraSkipFrames = vb.vbPopFrontInt16();
            mCameraNoAckCnt = 0;

            mCamera->closeCamera();

            if (camera >= 0) {
                mCamera->openCamera(camera);
                mCamera->startCameraStream(width, height, fps);
            }

        } else if (cmd == CMD_CAMERA_FRAME_ACK) {
            mCameraNoAckCnt--;
#endif
        } else if (cmd == CMD_TERMINAL_CMD) {
            QString str(vb);

            if (str == "help") {
#if HAS_CAMERA
                printTerminal("camera_info\n"
                              "  Print information about the available camera.");
#endif
            } else if (str == "camera_info") {
#if HAS_CAMERA
                bool res = true;
                if (!mCamera->isLoaded()) {
                    res = mCamera->openCamera();
                }

                if (res) {
                    printTerminal(mCamera->cameraInfo());
                } else {
                    printTerminal("No camera available.");
                }

                packetConsumed = true;
#endif
            }
        }
    }

    if (!packetConsumed) {
        if (mSerialPort->isOpen()) {
            mSerialPort->writeData(data);
        }

        for (CarSim *s: mSimulatedCars) {
            s->processData(data);
        }
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

    if ((mRtcmClient->isTcpConnected() || mRtcmClient->isSerialConnected()) && mRtcmSendBase) {
        mPacketInterface->sendRtcmUsb(255, mRtcmClient->encodeBasePos(mRtcmBaseLat,
                                                                      mRtcmBaseLon,
                                                                      mRtcmBaseHeight,
                                                                      0.0));
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
    (void)cmd;

    if (id != 254) {
        mCarId = id;

        if (QString::compare(mHostAddress.toString(), "0.0.0.0") != 0) {
            mUdpSocket->writeDatagram(data, mHostAddress, mUdpPort);
        }

        mTcpServer->packet()->sendPacket(data);
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
    (void)usec;

//    struct timeval now;
//    int rc;
//    now.tv_sec = sec;
//    now.tv_usec = usec;
//    rc = settimeofday(&now, NULL);

    if(setUnixTime(sec)) {
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

//    if (powerOff) {
//        reboot(RB_POWER_OFF);
//    } else {
//        reboot(RB_AUTOBOOT);
//    }

    rebootSystem(powerOff);
}

void CarClient::ubxRx(const QByteArray &data)
{
    mUbxBroadcaster->broadcastData(data);
}

void CarClient::rxRawx(ubx_rxm_rawx rawx)
{
    if (!rawx.leap_sec) {
        // Leap seconds are not known...
        return;
    }

    QDateTime dateGps(QDate(1980, 1, 6), QTime(0, 0, 0));
    dateGps.setOffsetFromUtc(0);
    dateGps = dateGps.addDays(rawx.week * 7);
    dateGps = dateGps.addMSecs((rawx.rcv_tow - (double)rawx.leaps) * 1000.0);

    QDateTime date = QDateTime::currentDateTime();
    qint64 diff = dateGps.toMSecsSinceEpoch() - date.toMSecsSinceEpoch();

    if (abs(diff) > 60000) {
        // Set the system time if the difference is over 10 seconds.
        qDebug() << "System time is different from GPS time. Difference: " << diff << " ms";

//        struct timeval now;
//        int rc;
//        now.tv_sec = dateGps.toTime_t();
//        now.tv_usec = dateGps.time().msec() * 1000.0;
//        rc = settimeofday(&now, NULL);

        if(setUnixTime(dateGps.toTime_t())) {
            qDebug() << "Sucessfully updated system time";
            restartRtklib();
        } else {
            qDebug() << "Setting system time failed";
        }
    }
}

void CarClient::tcpRx(QByteArray &data)
{
    mPacketInterface->sendPacket(data);
}

void CarClient::tcpConnectionChanged(bool connected, QString address)
{
    if (connected) {
        qDebug() << "TCP connection from" << address << "accepted";
    } else {
        qDebug() << "Disconnected TCP from" << address;
    }
}

void CarClient::rtcmReceived(QByteArray data, int type, bool sync)
{
    (void)type;
    (void)sync;
    mPacketInterface->sendRtcmUsb(255, data);
}

void CarClient::logEthernetReceived(quint8 id, QByteArray data)
{
    (void)id;
    mLogBroadcaster->broadcastData(data);
}

void CarClient::processCarData(QByteArray data)
{
    mPacketInterface->processData(data);
}

void CarClient::cameraImageCaptured(QImage img)
{
#if HAS_CAMERA
    if (mCameraSkipFrames > 0) {
        mCameraSkipFrameCnt++;

        if (mCameraSkipFrameCnt <= mCameraSkipFrames) {
            return;
        }
    }

    // No ack has been received for a couple of frames, meaning that
    // the connection probably is bad. Drop frame.
    if (mCameraNoAckCnt >= 3) {
        return;
    }

    mCameraSkipFrameCnt = 0;

    QByteArray data;
    data.append((quint8)mCarId);
    data.append((char)CMD_CAMERA_IMAGE);
    QBuffer buffer;
    buffer.open(QIODevice::WriteOnly);
    img.save(&buffer, "jpg", mCameraJpgQuality);
    buffer.close();
    data.append(buffer.buffer());

    if (data.size() > 100) {
        carPacketRx(mCarId, CMD_CAMERA_IMAGE, data);
        mCameraNoAckCnt++;
    }
#else
    (void)img;
#endif
}

bool CarClient::setUnixTime(qint64 t)
{
    // https://askubuntu.com/questions/159007/how-do-i-run-specific-sudo-commands-without-a-password
    QProcess process;
    process.setEnvironment(QProcess::systemEnvironment());
    process.start("sudo", QStringList() << "date" << "+%s" << "-s" << QString("@%1").arg(t));
    return waitProcess(process, 5000);
}

void CarClient::printTerminal(QString str)
{
    QByteArray packet;
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
