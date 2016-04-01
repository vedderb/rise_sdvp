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
