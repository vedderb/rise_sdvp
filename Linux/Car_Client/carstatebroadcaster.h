#ifndef CARSTATEBROADCASTER_H
#define CARSTATEBROADCASTER_H

#include <QObject>
#include <QTimer>
#include "packetinterface.h"
#include "tcpbroadcast.h"

class CarStateBroadcaster : public QObject
{
    Q_OBJECT
public:
    explicit CarStateBroadcaster(QObject *parent, PacketInterface *carClientPacketInterface, quint8 idToBroadcast);
    const int mCarStateBroadcastPort = 2102;

private slots:
    void carPacketRx(quint8 id, CMD_PACKET cmd, const QByteArray &data);

private:
    PacketInterface *mPacketInterface;
    quint8 mIdToBroadcast;
    QTimer mGetStateTimer;
    TcpBroadcast mTcpBroadcast;

};

#endif // CARSTATEBROADCASTER_H
