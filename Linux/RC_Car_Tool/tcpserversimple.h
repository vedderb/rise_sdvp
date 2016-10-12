#ifndef TCPSERVERSIMPLE_H
#define TCPSERVERSIMPLE_H

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include "packet.h"

class TcpServerSimple : public QObject
{
    Q_OBJECT
public:
    explicit TcpServerSimple(QObject *parent = 0);
    bool startServer(int port);
    void stopServer();
    bool sendData(const QByteArray &data);
    QString errorString();
    Packet *packet();

signals:
    void dataRx(const QByteArray &data);
    void connectionChanged(bool connected);

public slots:
    void newTcpConnection();
    void tcpInputDisconnected();
    void tcpInputDataAvailable();
    void tcpInputError(QAbstractSocket::SocketError socketError);
    void dataToSend(QByteArray &data);

private:
    QTcpServer *mTcpServer;
    QTcpSocket *mTcpSocket;
    Packet *mPacket;

};

#endif // TCPSERVERSIMPLE_H
