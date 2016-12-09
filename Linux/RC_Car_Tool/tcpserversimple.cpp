#include "tcpserversimple.h"
#include <QDebug>

TcpServerSimple::TcpServerSimple(QObject *parent) : QObject(parent)
{
    mTcpServer = new QTcpServer(this);
    mPacket = new Packet(this);
    mTcpSocket = 0;
    mUsePacket = false;

    connect(mTcpServer, SIGNAL(newConnection()), this, SLOT(newTcpConnection()));
    connect(mPacket, SIGNAL(dataToSend(QByteArray&)),
            this, SLOT(dataToSend(QByteArray&)));
}

bool TcpServerSimple::startServer(int port)
{
    if (!mTcpServer->listen(QHostAddress::Any,  port)) {
        return false;
    }

    return true;
}

void TcpServerSimple::stopServer()
{
    mTcpServer->close();

    if (mTcpSocket) {
        mTcpSocket->close();
        delete mTcpSocket;
        mTcpSocket = 0;
        emit connectionChanged(false);
    }
}

bool TcpServerSimple::sendData(const QByteArray &data)
{
    bool res = false;

    if (mTcpSocket) {
        mTcpSocket->write(data);
        res = true;
    }

    return res;
}

QString TcpServerSimple::errorString()
{
    return mTcpServer->errorString();
}

Packet *TcpServerSimple::packet()
{
    return mPacket;
}

void TcpServerSimple::newTcpConnection()
{
    QTcpSocket *socket = mTcpServer->nextPendingConnection();

    if (mTcpSocket) {
        socket->close();
        delete socket;
    } else {
        mTcpSocket = socket;

        if (mTcpSocket) {
            connect(mTcpSocket, SIGNAL(readyRead()), this, SLOT(tcpInputDataAvailable()));
            connect(mTcpSocket, SIGNAL(disconnected()),
                    this, SLOT(tcpInputDisconnected()));
            connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
                    this, SLOT(tcpInputError(QAbstractSocket::SocketError)));
            emit connectionChanged(true);
        }
    }
}

void TcpServerSimple::tcpInputDisconnected()
{
    mTcpSocket->deleteLater();
    mTcpSocket = 0;
    emit connectionChanged(false);
}

void TcpServerSimple::tcpInputDataAvailable()
{
    QByteArray data = mTcpSocket->readAll();
    emit dataRx(data);

    if (mUsePacket) {
        mPacket->processData(data);
    }
}

void TcpServerSimple::tcpInputError(QAbstractSocket::SocketError socketError)
{
    (void)socketError;
    mTcpSocket->close();
    delete mTcpSocket;
    mTcpSocket = 0;
    emit connectionChanged(false);
}

void TcpServerSimple::dataToSend(QByteArray &data)
{
    sendData(data);
}

bool TcpServerSimple::usePacket() const
{
    return mUsePacket;
}

void TcpServerSimple::setUsePacket(bool usePacket)
{
    mUsePacket = usePacket;
}
