#ifndef CARCLIENT_H
#define CARCLIENT_H

#include <QObject>
#include <QSerialPort>
#include <QTcpSocket>
#include <QTimer>
#include "packetinterface.h"
#include "tcpbroadcast.h"

class CarClient : public QObject
{
    Q_OBJECT
public:
    typedef struct {
        bool serialConnect;
        QString serialPort;
        int serialBaud;
        bool nmeaConnect;
        QString nmeaServer;
        int nmeaPort;
    } settings_t;

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
    void reconnectTimerSlot();

private:
    PacketInterface *mPacketInterface;
    TcpBroadcast *mRtcmBroadcaster;
    QSerialPort *mSerialPort;
    QTcpSocket *mTcpSocket;
    int mCarId;
    QTimer *mReconnectTimer;
    settings_t mSettings;
    bool mTcpConnected;

};

#endif // CARCLIENT_H
