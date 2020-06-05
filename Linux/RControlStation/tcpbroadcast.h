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

#ifndef TCPBROADCAST_H
#define TCPBROADCAST_H

#include <QObject>
#include <QUrl>
#include <QTcpServer>
#include <QTcpSocket>
#include <QList>
#include <QFile>

class TcpBroadcast : public QObject
{
    Q_OBJECT
public:
    explicit TcpBroadcast(QObject *parent = 0);
    ~TcpBroadcast();
    bool startTcpServer(int port);
    bool isRunning() {return mTcpServer->isListening();}
    QString getLastError();
    void stopServer();
    void broadcastData(QByteArray data);
    bool logToFile(QString file);
    void logStop();

signals:

public slots:


private slots:
    void newTcpConnection();

private:
    QTcpServer *mTcpServer;
    QList<QTcpSocket*> mSockets;
    QFile mLog;

};

#endif // TCPBROADCAST_H
