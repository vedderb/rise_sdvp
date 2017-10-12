#ifndef CHRONOS_H
#define CHRONOS_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>

#include "tcpserversimple.h"
#include "packetinterface.h"
#include "vbytearray.h"

class Chronos : public QObject
{
    Q_OBJECT
public:
    Chronos(QObject *parent = 0);
    bool startServer(PacketInterface *packet);

private slots:
    void startTimerSlot();
    void tcpRx(QByteArray data);
    void tcpConnectionChanged(bool connected);
    void readPendingDatagrams();
    void stateReceived(quint8 id, CAR_STATE state);

private:
    TcpServerSimple *mTcpServer;
    PacketInterface *mPacket;
    QUdpSocket *mUdpSocket;
    QHostAddress mUdpHostAddress;
    quint16 mUdpPort;
    QTimer *mStartTimer;
    bool mIsArmed;

    int mTcpState;
    quint8 mTcpType;
    quint32 mTcpLen;
    QByteArray mTcpData;

    int mHeabPollCnt;
    double mLlhRef[3];
    QList<LocPoint> mRouteLast;
    chronos_sypm mSypmLast;

    bool decodeMsg(quint8 type, quint32 len, QByteArray payload);

    void processDopm(QVector<chronos_dopm_pt> path);
    void processOsem(chronos_osem osem);
    void processOstm(chronos_ostm ostm);
    void processStrt(chronos_strt strt);
    void processHeab(chronos_heab heab);
    void processSypm(chronos_sypm sypm);
    void processMtsp(chronos_mtsp mtsp);

    bool sendMonr(chronos_monr monr);
    quint64 chronosTimeNow();
    quint32 chronosTimeToUtcToday(quint64 time);

};

#endif // CHRONOS_H
