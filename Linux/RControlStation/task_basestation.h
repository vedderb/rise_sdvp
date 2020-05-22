#ifndef BASESTATIONTASK_H
#define BASESTATIONTASK_H

#include "task.h"

class BaseStationTask : public Task
{
    Q_OBJECT
public:
    BaseStationTask(QObject *parent = 0) : Task(parent) {}

private:
    void task();
};

#endif // BASESTATIONTASK_H
