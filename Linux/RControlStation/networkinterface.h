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

#ifndef NETWORKINTERFACE_H
#define NETWORKINTERFACE_H

#include <QWidget>
#include <QUdpSocket>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include "tcpserversimple.h"
#include "packetinterface.h"
#include "carinterface.h"
#include "mapwidget.h"

namespace Ui {
class NetworkInterface;
}

class NetworkInterface : public QWidget
{
    Q_OBJECT

public:
    explicit NetworkInterface(QWidget *parent = 0);
    ~NetworkInterface();
    void setMap(MapWidget *map);
    void setPacketInterface(PacketInterface *packetInterface);
    void setCars(QList<CarInterface*> *cars);

    void sendState(quint8 id, const CAR_STATE &state);
    void sendEnuRef(quint8 id, double lat, double lon, double height);
    void sendPrint(quint8 id, QString str);
    void sendRoute(quint8 id, QList<LocPoint> route);
    void sendError(const QString &txt, const QString &cmd = "");
    void sendAck(const QString &cmd);

    void setTcpEnabled(bool enabled, int port = -1);
    void setUdpEnabled(bool enabled, int port = -1);

private slots:
    void tcpDataRx(const QByteArray &data);
    void tcpConnectionChanged(bool connected);
    void udpReadReady();
    void pollTimerSlot();

    void stateReceived(quint8 id, CAR_STATE state);
    void enuRefReceived(quint8 id, double lat, double lon, double height);
    void printReceived(quint8 id, QString str);

    void on_tcpActivateBox_clicked(bool checked);
    void on_udpActivateBox_clicked(bool checked);

private:
    Ui::NetworkInterface *ui;
    QUdpSocket *mUdpSocket;
    TcpServerSimple *mTcpServer;
    QHostAddress mLastHostAddress;
    QByteArray mRxBuffer;
    MapWidget *mMap;
    PacketInterface *mPacketInterface;
    QTimer *mPollTimer;
    int mPollTimerCarId;
    QList<CarInterface*> *mCars;

    void processData(const QByteArray &data);
    void processXml(const QByteArray &xml);
    void sendData(const QByteArray &data);

};

#endif // NETWORKINTERFACE_H
