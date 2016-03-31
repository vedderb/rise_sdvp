#ifndef CARCLIENT_H
#define CARCLIENT_H

#include <QObject>
#include <QSerialPort>
#include <QTcpSocket>
#include "packetinterface.h"
#include "tcpbroadcast.h"

class CarClient : public QObject
{
    Q_OBJECT
public:
    explicit CarClient(QObject *parent = 0);
    void connectSerial(QString port, int baudrate = 115200);
    void startRtcmServer(int port = 8200);
    void connectNmea(QString server, int port = 2948);

signals:

public slots:
    void serialDataAvailable();
    void serialPortError(QSerialPort::SerialPortError error);
    void packetDataToSend(QByteArray &data);
    void tcpDataAvailable();
    void tcpConnected();
    void tcpDisconnected();
    void rtcmUsbRx(quint8 id, QByteArray data);

private:
    PacketInterface *mPacketInterface;
    TcpBroadcast *mRtcmBroadcaster;
    QSerialPort *mSerialPort;
    QTcpSocket *mTcpSocket;
    int mCarId;

};

#endif // CARCLIENT_H
