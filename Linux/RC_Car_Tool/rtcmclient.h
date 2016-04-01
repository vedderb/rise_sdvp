#ifndef RTCMCLIENT_H
#define RTCMCLIENT_H

#include <QObject>
#include <QTcpSocket>
#include <QSerialPort>

class RtcmClient : public QObject
{
    Q_OBJECT
public:
    static RtcmClient* currentMsgHandler;

    explicit RtcmClient(QObject *parent = 0);
    bool connectNtrip(QString server, QString stream, QString user = "", QString pass = "", int port = 80);
    bool connectTcp(QString server, int port = 80);
    bool connectSerial(QString port, int baudrate = 115200);
    bool isTcpConnected();
    bool isSerialConnected();
    void disconnectTcpNtrip();
    void disconnectSerial();

    void emitRtcmReceived(QByteArray data, int type);

signals:
    void rtcmReceived(QByteArray data, int type);

public slots:
    void tcpInputConnected();
    void tcpInputDisconnected();
    void tcpInputDataAvailable();
    void tcpInputError(QAbstractSocket::SocketError socketError);
    void serialDataAvailable();
    void serialPortError(QSerialPort::SerialPortError error);

private:
    QString mNtripUser;
    QString mNtripPassword;
    QString mNtripServer;
    QString mNtripStream;
    QTcpSocket *mTcpSocket;
    QSerialPort *mSerialPort;

};

#endif // RTCMCLIENT_H
