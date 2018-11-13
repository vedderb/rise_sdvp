#ifndef CHRONOS_H
#define CHRONOS_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>

#include "tcpserversimple.h"
#include "packetinterface.h"
#include "vbytearray.h"

// SYNC Word
#define ISO_SYNC_WORD 0x7E7E
#define ISO_PART_SYNC_WORD 0x7E

// ISO Message Types
#define ISO_MSG_DOTM 0x0001
#define ISO_MSG_OSEM 0x0002
#define ISO_MSG_OSTM 0x0003
#define ISO_MSG_STRT 0x0004
#define ISO_MSG_HEAB 0x0005
#define ISO_MSG_MONR 0x0006

// ISO Value Types
#define ISO_VALUE_ID_LAT 0x0020
#define ISO_VALUE_ID_LON 0x0021
#define ISO_VALUE_ID_ALT 0x0022
#define ISO_VALUE_ID_DateISO8601 0x0004
#define ISO_VALUE_ID_GPS_WEEK 0x0003
#define ISO_VALUE_ID_GPS_SEC_OF_WEEK 0x0002
#define ISO_VALUE_ID_MAX_WAY_DEV 0x0070
#define ISO_VALUE_ID_MAX_LATERAL_DEV 0x0072
#define ISO_VALUE_ID_MIN_POS_ACCURACY 0x0074
#define ISO_VALUE_ID_STATE_CHANGE_REQ 0x0064
#define ISO_VALUE_ID_DELAYED_START 0x0001
#define ISO_VALUE_ID_REL_TIME 0x0001
#define ISO_VALUE_ID_X_POS 0x0010
#define ISO_VALUE_ID_Y_POS 0x0011
#define ISO_VALUE_ID_Z_POS 0x0012
#define ISO_VALUE_ID_HEADING 0x0030
#define ISO_VALUE_ID_LONG_SPEED 0x0040
#define ISO_VALUE_ID_LAT_SPEED 0x0041
#define ISO_VALUE_ID_LONG_ACC 0x0050
#define ISO_VALUE_ID_LAT_ACC 0x0051

// CHRONOS PROTOCOL VERSION
#define PROTOCOL_VERSION 0

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
    bool mIsStarted;
    quint8 mSenderId; // Seems to be of undecided use currently in Chronos
    quint8 mSequenceNum;

    int mTcpState;
    quint16 mTcpType;
    quint32 mTcpLen;
    quint16 mTcpChecksum;
    VByteArray mTcpData;

    int mHeabPollCnt;
    double mLlhRef[3];
    QList<LocPoint> mRouteLast;
    chronos_sypm mSypmLast;

    bool decodeMsg(quint16 type, quint32 len, QByteArray payload);

    void processDopm(QVector<chronos_dopm_pt> path);
    void processOsem(chronos_osem osem);
    void processOstm(chronos_ostm ostm);
    void processStrt(chronos_strt strt);
    void processHeab(chronos_heab heab);
    void processSypm(chronos_sypm sypm);
    void processMtsp(chronos_mtsp mtsp);

    void appendChronosChecksum(VByteArray &vb);

    bool sendMonr(chronos_monr monr);
    quint64 chronosTimeNow();
    quint32 chronosTimeToUtcToday(quint64 time);

};

#endif // CHRONOS_H
