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

QString TcpBroadcast::getLastError()
{
    return mTcpServer->errorString();
}

void TcpBroadcast::stopServer()
{
    mTcpServer->close();
    QMutableListIterator<QTcpSocket*> itr(mSockets);

    while(itr.hasNext()) {
        QTcpSocket *socket = itr.next();
        socket->deleteLater();
        itr.remove();
    }
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
    }
}

void TcpBroadcast::newTcpConnection()
{
    mSockets.append(mTcpServer->nextPendingConnection());
    connect(mSockets.last(), SIGNAL(readyRead()),
            this, SLOT(readyRead()));
    qDebug() << "TCP connection accepted:" << mSockets[mSockets.size() - 1]->peerAddress();
}

void TcpBroadcast::readyRead()
{
    QMutableListIterator<QTcpSocket*> itr(mSockets);

    while(itr.hasNext()) {
        QTcpSocket *socket = itr.next();
        QByteArray data = socket->readAll();
        if (!data.isEmpty()) {
            emit dataReceived(data);
        }
    }
}

