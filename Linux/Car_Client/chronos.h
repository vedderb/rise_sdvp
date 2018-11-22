#ifndef CHRONOS_H
#define CHRONOS_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>

#include "tcpserversimple.h"
#include "packetinterface.h"
#include "chronoscomm.h"

class Chronos : public QObject
{
    Q_OBJECT
public:
    Chronos(QObject *parent = 0);
    bool startServer(PacketInterface *packet);

private slots:
    void startTimerSlot();
    void connectionChanged(bool connected, QString address);
    void stateReceived(quint8 id, CAR_STATE state);

    void processDotm(QVector<chronos_dotm_pt> path);
    void processOsem(chronos_osem osem);
    void processOstm(chronos_ostm ostm);
    void processStrt(chronos_strt strt);
    void processHeab(chronos_heab heab);
    void processSypm(chronos_sypm sypm);
    void processMtsp(chronos_mtsp mtsp);

private:
    PacketInterface *mPacket;
    ChronosComm *mChronos;
    QTimer *mStartTimer;
    bool mIsArmed;
    bool mIsStarted;

    int mHeabPollCnt;
    double mLlhRef[3];
    QList<LocPoint> mRouteLast;
    chronos_sypm mSypmLast;
};

#endif // CHRONOS_H
