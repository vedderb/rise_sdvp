#ifndef PING_H
#define PING_H

#include <QObject>
#include <QThread>
#include <stdint.h>

class Ping : public QThread
{
    Q_OBJECT
public:
    Ping(QObject *parent);
    ~Ping();
    bool pingHost(QString host, int len = 64, QString msg = "");

signals:
    void pingRx(int us, QString msg);
    void pingError(QString msg, QString error);

public slots:

protected:
    void run();

private:
    int ping(QString target);
    uint16_t in_cksum(uint16_t *addr, unsigned len);

    QString mMsg;
    QString mHost;
    int mLen;

    int mSocket;
    unsigned char *mPacket;
    unsigned char *mOutpack;

};

#endif // PING_H
