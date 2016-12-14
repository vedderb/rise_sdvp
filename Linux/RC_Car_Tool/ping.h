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

#ifndef PING_H
#define PING_H

#include <QObject>
#include <QThread>
#include <stdint.h>

class Ping : public QThread
{
    Q_OBJECT
public:
    Ping(QObject *parent);
    ~Ping();
    bool pingHost(QString host, int len = 64, QString msg = "");

signals:
    void pingRx(int us, QString msg);
    void pingError(QString msg, QString error);

public slots:

protected:
    void run();

private:
    int ping(QString target);
    uint16_t in_cksum(uint16_t *addr, unsigned len);

    QString mMsg;
    QString mHost;
    int mLen;

    int mSocket;
    unsigned char *mPacket;
    unsigned char *mOutpack;

};

#endif // PING_H
