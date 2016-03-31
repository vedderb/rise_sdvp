#include "tcpbroadcast.h"

TcpBroadcast::TcpBroadcast(QObject *parent) : QObject(parent)
{
    mTcpServer = new QTcpServer(this);
    connect(mTcpServer, SIGNAL(newConnection()), this, SLOT(newTcpConnection()));
}

TcpBroadcast::~TcpBroadcast()
{
    logStop();
}

bool TcpBroadcast::startTcpServer(int port)
{
    if (!mTcpServer->listen(QHostAddress::Any,  port)) {
        qWarning() << "Unable to start TCP server: " << mTcpServer->errorString();
        return false;
    }

    return true;
}

void TcpBroadcast::broadcastData(QByteArray data)
{
    QMutableListIterator<QTcpSocket*> itr(mSockets);

    if (mLog.isOpen()) {
        mLog.write(data);
    }

    while(itr.hasNext()) {
        QTcpSocket *socket = itr.next();
        if (socket->isOpen()) {
            socket->write(data);
        } else {
            socket->deleteLater();
            itr.remove();
        }
    }
}

bool TcpBroadcast::logToFile(QString file)
{
    if (mLog.isOpen()) {
        mLog.close();
    }

    mLog.setFileName(file);
    return mLog.open(QIODevice::ReadWrite | QIODevice::Truncate);
}

void TcpBroadcast::logStop()
{
    if (mLog.isOpen()) {
        qDebug() << "Closing log:" << mLog.fileName();
        mLog.close();
    } else {
        qDebug() << "TCP log not open";
    }
}

void TcpBroadcast::newTcpConnection()
{
    mSockets.append(mTcpServer->nextPendingConnection());
    qDebug() << "TCP connection accepted:" << mSockets[mSockets.size() - 1]->peerAddress();
}

