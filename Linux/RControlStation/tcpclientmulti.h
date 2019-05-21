#ifndef TCPCLIENTMULTI_H
#define TCPCLIENTMULTI_H

#include <QObject>
#include <QTcpSocket>
#include "packetinterface.h"

class TcpClientMulti : public QObject
{
    Q_OBJECT
public:
    explicit TcpClientMulti(QObject *parent = nullptr);
    ~TcpClientMulti();

    void addConnection(QString ip, int port);
    bool isAnyConnected();
    void disconnectAll();
    void sendAll(QByteArray data);

signals:
    void stateChanged(QString msg, bool isError);
    void packetRx(QByteArray data);

public slots:

private:
    class TcpConn {
    public:
        TcpConn(QString ip, int port, TcpClientMulti *client) {
            socket.abort();
            socket.connectToHost(ip, port);

            connect(&socket, &QTcpSocket::readyRead, [this]() {
                while (socket.bytesAvailable() > 0) {
                    QByteArray data = socket.readAll();
                    packet.processData(data);
                }
            });

            connect(&socket, &QTcpSocket::connected, [this,client]() {
                emit client->stateChanged("TCP Connected", false);
            });

            connect(&socket, &QTcpSocket::disconnected, [this,client]() {
                emit client->stateChanged("TCP Disconnected", false);
            });

            connect(&socket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::error),
                    [this,client](QAbstractSocket::SocketError e) {
                (void)e;
                QString errorStr = socket.errorString();
                socket.close();
                emit client->stateChanged(QString("TCP Error: %1").arg(errorStr), true);
            });

            connect(&packet, &PacketInterface::packetReceived,
                    [client](quint8 id, CMD_PACKET cmd, const QByteArray &data) {
                (void)id;
                (void)cmd;
                emit client->packetRx(data);
            });

            connect(&packet, &PacketInterface::dataToSend, [this](QByteArray &data) {
                    socket.write(data);
            });
        }

        ~TcpConn() {
            socket.abort();
        }

        void sendData(QByteArray data) {
            socket.write(data);
        }

        bool isTcpConnected() {
            return socket.isOpen();
        }

    private:
        QTcpSocket socket;
        PacketInterface packet;
    };

    QVector<TcpConn*> mTcpConns;

};

#endif // TCPCLIENTMULTI_H
