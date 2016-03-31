#include "rtcmclient.h"
#include "rtcm3_simple.h"
#include <QDebug>

namespace {
void rtcm_rx(uint8_t *data, int len, int type) {
    if (RtcmClient::currentMsgHandler) {
        QByteArray rtcm_data((const char*)data, len);
        RtcmClient::currentMsgHandler->emitRtcmReceived(rtcm_data, type);
    }
}
}

// Static member initialization
RtcmClient *RtcmClient::currentMsgHandler = 0;

RtcmClient::RtcmClient(QObject *parent) : QObject(parent)
{
    mTcpSocket = new QTcpSocket(this);
}

bool RtcmClient::connectNtrip(QString server, QString stream, QString user, QString pass, int port)
{
    currentMsgHandler = this;

    mNtripUser = user;
    mNtripPassword = pass;
    mNtripStream = stream;
    mNtripServer = server;

    mTcpSocket->abort();
    mTcpSocket->connectToHost(server, port);

    connect(mTcpSocket, SIGNAL(readyRead()), this, SLOT(tcpInputDataAvailable()));
    connect(mTcpSocket, SIGNAL(connected()), this, SLOT(tcpInputConnected()));
    connect(mTcpSocket, SIGNAL(disconnected()),
            this, SLOT(tcpInputDisconnected()));
    connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(tcpInputError(QAbstractSocket::SocketError)));

    rtcm3_set_rx_callback(rtcm_rx);

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

bool RtcmClient::isNtripConnected()
{
    // If no stream is selected we have simply connected to a TCP server.
    if (mNtripStream.size() > 0) {
        return mTcpSocket->isOpen();
    }

    return false;
}

bool RtcmClient::isTcpConnected()
{
    // If no stream is selected we have simply connected to a TCP server.
    if (mNtripStream.size() == 0) {
        return mTcpSocket->isOpen();
    }

    return false;
}

void RtcmClient::disconnectTcpNtrip()
{
    mTcpSocket->close();
}

void RtcmClient::emitRtcmReceived(QByteArray data, int type)
{
    emit rtcmReceived(data, type);
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
        int ret = rtcm3_input_data(data.at(i));
        if (ret == -1 || ret == -2) {
            //qWarning() << "RTCM decode error:" <<  ret;
        }
    }
}

void RtcmClient::tcpInputError(QAbstractSocket::SocketError socketError)
{
    switch (socketError) {
    case QAbstractSocket::RemoteHostClosedError:
        qWarning() << "TcpError: connection Closed";
        break;
    case QAbstractSocket::HostNotFoundError:
        qWarning() << "TcpError: host not found";
        break;
    case QAbstractSocket::ConnectionRefusedError:
        qWarning() << "TcpError: connection refused";
        break;
    default:
        qWarning() << "TcpError:" << mTcpSocket->errorString();
    }
}

