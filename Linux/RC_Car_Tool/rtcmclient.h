#ifndef RTCMCLIENT_H
#define RTCMCLIENT_H

#include <QObject>
#include <QTcpSocket>

class RtcmClient : public QObject
{
    Q_OBJECT
public:
    static RtcmClient* currentMsgHandler;

    explicit RtcmClient(QObject *parent = 0);
    bool connectNtrip(QString server, QString stream, QString user = "", QString pass = "", int port = 80);
    bool connectTcp(QString server, int port = 80);
    bool isNtripConnected();
    bool isTcpConnected();
    void disconnectTcpNtrip();

    void emitRtcmReceived(QByteArray data, int type);

signals:
    void rtcmReceived(QByteArray data, int type);

public slots:
    void tcpInputConnected();
    void tcpInputDisconnected();
    void tcpInputDataAvailable();
    void tcpInputError(QAbstractSocket::SocketError socketError);

private:
    QString mNtripUser;
    QString mNtripPassword;
    QString mNtripServer;
    QString mNtripStream;
    QTcpSocket *mTcpSocket;

};

#endif // RTCMCLIENT_H
