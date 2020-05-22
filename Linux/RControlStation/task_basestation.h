#ifndef BASESTATIONTASK_H
#define BASESTATIONTASK_H

#include "task.h"
#include "ublox.h"

class Task_BaseStation : public Task
{
    Q_OBJECT
public:
    Task_BaseStation(QObject *parent = 0) : Task(parent) {}

private:
    Ublox mUblox;
    QMap<int, int> mRtcmUbx;
    void task();
};

#endif // BASESTATIONTASK_H
