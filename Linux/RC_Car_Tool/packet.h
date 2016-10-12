#ifndef PACKET_H
#define PACKET_H

#include <QObject>
#include <QTimer>

class Packet : public QObject
{
    Q_OBJECT
public:
    explicit Packet(QObject *parent = 0);
    void sendPacket(const QByteArray &data);

    static unsigned short crc16(const unsigned char *buf, unsigned int len);

signals:
    void dataToSend(QByteArray &data);
    void packetReceived(QByteArray &packet);

public slots:
    void processData(QByteArray data);

private slots:
    void timerSlot();

private:
    QTimer *mTimer;
    int mRxTimer;
    int mRxState;
    unsigned int mPayloadLength;
    unsigned char mCrcLow;
    unsigned char mCrcHigh;
    QByteArray mRxBuffer;
    int mByteTimeout;

};

#endif // PACKET_H
