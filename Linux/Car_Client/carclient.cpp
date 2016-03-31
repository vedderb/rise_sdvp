#include "carclient.h"
#include <QDebug>

CarClient::CarClient(QObject *parent) : QObject(parent)
{
    mSerialPort = new QSerialPort(this);
    mPacketInterface = new PacketInterface(this);
    mRtcmBroadcaster = new TcpBroadcast(this);
    mTcpSocket = new QTcpSocket(this);
    mCarId = 255;

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
}

void CarClient::connectSerial(QString port, int baudrate)
{
    if(mSerialPort->isOpen()) {
        return;
    }

    mSerialPort->setPortName(port);
    mSerialPort->open(QIODevice::ReadWrite);

    if(!mSerialPort->isOpen()) {
        return;
    }

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
    mTcpSocket->connectToHost(server, port);
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
        message = "Serial port error: " + QString::number(error);
        break;
    }

    if(!message.isEmpty()) {
        qDebug() << message;

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
    qDebug() << "TCP Connected";
}

void CarClient::tcpDisconnected()
{
    qDebug() << "TCP Disconnected";
}

void CarClient::rtcmUsbRx(quint8 id, QByteArray data)
{
    mCarId = id;
    mRtcmBroadcaster->broadcastData(data);
}

