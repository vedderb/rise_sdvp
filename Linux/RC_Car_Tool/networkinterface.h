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

    void sendState(quint8 car, const CAR_STATE &state);
    void sendError(const QString &txt);

private slots:
    void tcpDataRx(const QByteArray &data);
    void tcpConnectionChanged(bool connected);
    void udpReadReady();

    void stateReceived(quint8 id, CAR_STATE state);

    void on_tcpActivateBox_toggled(bool checked);
    void on_udpActivateBox_toggled(bool checked);

private:
    Ui::NetworkInterface *ui;
    QUdpSocket *mUdpSocket;
    TcpServerSimple *mTcpServer;
    QHostAddress mLastHostAddress;
    QByteArray mRxBuffer;
    MapWidget *mMap;
    PacketInterface *mPacketInterface;

    void processData(const QByteArray &data);
    void processXml(const QByteArray &xml);
    void sendData(const QByteArray &data);

};

#endif // NETWORKINTERFACE_H
