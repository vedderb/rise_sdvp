#include "chronos.h"
#include "utility.h"

#include <QDebug>
#include <cmath>
#include <QDateTime>

Chronos::Chronos(QObject *parent) : QObject(parent)
{
    mPacket = 0;
    mChronos = new ChronosComm(this);
    mStartTimer = new QTimer(this);
    mIsArmed = false;
    mIsStarted = false;

    mHeabPollCnt = 0;

    connect(mStartTimer, SIGNAL(timeout()),
            this, SLOT(startTimerSlot()));
    connect(mChronos, SIGNAL(connectionChanged(bool,QString)),
            this, SLOT(connectionChanged(bool,QString)));
    connect(mChronos, SIGNAL(trajRx(chronos_traj)),
            this, SLOT(processTraj(chronos_traj)));
    connect(mChronos, SIGNAL(heabRx(chronos_heab)),
            this, SLOT(processHeab(chronos_heab)));
    connect(mChronos, SIGNAL(osemRx(chronos_osem)),
            this, SLOT(processOsem(chronos_osem)));
    connect(mChronos, SIGNAL(ostmRx(chronos_ostm)),
            this, SLOT(processOstm(chronos_ostm)));
    connect(mChronos, SIGNAL(strtRx(chronos_strt)),
            this, SLOT(processStrt(chronos_strt)));
}

bool Chronos::startServer(PacketInterface *packet, QHostAddress addr)
{
    mPacket = packet;

    bool res = mChronos->startObject(addr);

    if (res && mPacket) {
        connect(mPacket, SIGNAL(stateReceived(quint8,CAR_STATE)),
                this, SLOT(stateReceived(quint8,CAR_STATE)));
    }

    return res;
}

ChronosComm *Chronos::comm()
{
    return mChronos;
}

void Chronos::startTimerSlot()
{
    qDebug() << "Starting car";
    mIsStarted = true;

    if (mPacket) {
        mPacket->setApActive(255, true);
        mScenarioTimer.start();
    }
}

void Chronos::connectionChanged(bool connected, QString address)
{
    if (connected) {
        qDebug() << "Chronos TCP connection accepted from" << address;
    } else {
        qDebug() << "Chronos TCP disconnected from" << address;
        mIsArmed = false;
    }
}

void Chronos::stateReceived(quint8 id, CAR_STATE state)
{
    (void)id;
    (void)state;

    chronos_monr monr;

    double yaw = state.yaw + 90.0;
    while (yaw < 0) {
        yaw += 360.0;
    }
    while (yaw > 360.0) {
        yaw -= 360.0;
    }

    // monr has 13 fields now.
    monr.gps_ms_of_week = ChronosComm::gpsMsOfWeek();
    monr.x = state.px;
    monr.y = state.py;
    monr.z = 0;
    monr.heading = yaw;
    monr.lon_speed = state.speed;
    monr.lat_speed = 0;
    monr.lon_acc = state.accel[1];  // TODO: hmm
    monr.lat_acc = state.accel[0];
    monr.direction = state.speed >= 0 ? 0 : 1;  // 0 means forward now.
    monr.rdyToArm = 1;
    monr.status = mIsArmed ? (mIsStarted ? 4 : 2) : 0;
    // if armed and started -> running (4)
    // if armed and !started -> armed (2)
    // otherwise 0
    // (Simplified)
    monr.error = 0; // TODO: check for errors.

    mChronos->sendMonr(monr);
}

void Chronos::processTraj(chronos_traj traj)
{
    qDebug() << "TRAJ RX";

    // Subsample points
    QVector<chronos_traj_pt> path_reduced;

    if (traj.traj_pts.size() > 0) {
        path_reduced.append(traj.traj_pts.first());
        for (chronos_traj_pt pt: traj.traj_pts) {
            chronos_traj_pt pt_last = path_reduced.last();

            if (sqrt((pt.x - pt_last.x) * (pt.x - pt_last.x) +
                     (pt.y - pt_last.y) * (pt.y - pt_last.y) +
                     (pt.z - pt_last.z) * (pt.z - pt_last.z)) > 1.0) {
                path_reduced.append(pt);
            }
        }

        // Add end point
        chronos_traj_pt pt = traj.traj_pts.last();
        chronos_traj_pt pt_last = path_reduced.last();
        if (sqrt((pt.x - pt_last.x) * (pt.x - pt_last.x) +
                 (pt.y - pt_last.y) * (pt.y - pt_last.y) +
                 (pt.z - pt_last.z) * (pt.z - pt_last.z)) > 0.01) {
            path_reduced.append(pt);
        }
    }

//    qDebug() << "Last PT time" << path_reduced.last().tRel - mScenarioTimer.elapsed();

    if (mPacket) {
        mRouteLast.clear();
        QList<LocPoint> points;
        bool first = true;
        for (chronos_traj_pt pt: path_reduced) {
            LocPoint lpt;
            lpt.setXY(pt.x, pt.y);
            lpt.setSpeed(pt.long_speed);
            lpt.setTime(pt.tRel);

            points.append(lpt);
            mRouteLast.append(lpt);

            if (points.size() >= 20) {
                if (first) {
                    mPacket->replaceRoute(255, points);
                    first = false;
                } else {
                    mPacket->setRoutePoints(255, points);
                }

                points.clear();
            }
        }

        if (points.size() > 0) {
            if (first) {
                mPacket->replaceRoute(255, points);
            } else {
                mPacket->setRoutePoints(255, points);
            }
        }
    }
}

void Chronos::processOsem(chronos_osem osem)
{
    qDebug() << "OSEM RX";

    if (mIsArmed) {
        qDebug() << "Ignored because car is armed";
        return;
    }

    mLlhRef[0] = osem.lat;
    mLlhRef[1] = osem.lon;
    mLlhRef[2] = osem.alt;

    // TODO: Rotate route with heading

    if (mPacket) {
        mPacket->setEnuRef(255, mLlhRef);
        mPacket->clearRoute(255);
    }
}

void Chronos::processOstm(chronos_ostm ostm)
{
    qDebug() << "OSTM RX";

    if (ostm.armed == 0x2) {
        mIsArmed = true;
    } else if (ostm.armed == 0x3) {
        mIsArmed = false;
    }

    qDebug() << "Armed:" << mIsArmed;
}

void Chronos::processStrt(chronos_strt strt)
{
    quint64 cTime = ChronosComm::gpsMsOfWeek();

    qDebug() << "STRT RX" << cTime << strt.gps_ms_of_week;

    if (!mIsArmed) {
        qDebug() << "Ignored because car is not armed";
        return;
    }

    if ((strt.gps_ms_of_week <= cTime) || (strt.gps_ms_of_week - cTime) < 10) {
        startTimerSlot();
        qDebug() << "Starting car now";
    } else {
        mStartTimer->setSingleShot(true);
        mStartTimer->start(strt.gps_ms_of_week - cTime);
        qDebug() << "Starting car in" << strt.gps_ms_of_week - cTime << "ms";
    }
}

void Chronos::processHeab(chronos_heab heab)
{
    (void)heab;

    if (mPacket) {
        if (heab.status == 1) {
            mHeabPollCnt++;

            if (mHeabPollCnt >= 4) {
                mHeabPollCnt = 0;
                mPacket->getState(255);
            }
        } else {
            mPacket->setApActive(255, false);
        }
    }
}

void Chronos::processSypm(chronos_sypm sypm)
{
    qDebug() << "SYPM RX:" << sypm.sync_point << sypm.stop_time;
    mSypmLast = sypm;

    // Make sure that the car stops until MTSP is received.
    int closest_sync = 0;
    for (int i = 0;i < mRouteLast.size();i++) {
        if (fabs(mRouteLast.at(i).getTime() - mSypmLast.sync_point) <
                fabs(mRouteLast.at(closest_sync).getTime() - mSypmLast.sync_point)) {
            closest_sync = i;
        }
    }

    if (mPacket) {
        mPacket->setSyncPoint(255, closest_sync, 1000 * 60 * 60, 1000);
        qDebug() << "Car will not start until MTSP is received";
    }
}

// TODO: Not part of protocol any more...
void Chronos::processMtsp(chronos_mtsp mtsp)
{
    qDebug() << "MTSP RX";
    (void)mtsp;

    // Find points
    int closest_sync = 0;
    for (int i = 0;i < mRouteLast.size();i++) {
        if (fabs(mRouteLast.at(i).getTime() - mSypmLast.sync_point) <
                fabs(mRouteLast.at(closest_sync).getTime() - mSypmLast.sync_point)) {
            closest_sync = i;
        }
    }

    if (mPacket) {
        mPacket->setSyncPoint(255, closest_sync, mtsp.time_est - ChronosComm::gpsMsOfWeek(),
                              mSypmLast.sync_point - mSypmLast.stop_time);

        qDebug() << closest_sync << mtsp.time_est - ChronosComm::gpsMsOfWeek() <<
                    mSypmLast.sync_point - mSypmLast.stop_time;
    }
}
