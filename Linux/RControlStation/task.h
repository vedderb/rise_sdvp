#ifndef TASK_H
#define TASK_H

#include <QObject>

// abstract class used to implement commandline-based modes
class Task : public QObject
{
    Q_OBJECT
public:
    Task(QObject *parent = 0) : QObject(parent) {}

private:
    virtual void task() = 0; // the actual work to be performed

public slots:
    void run()
    {
        task();
    }

signals:
    void finished();
};

#endif // TASK_H
