#ifndef GPIO_H
#define GPIO_H

#include <QObject>
#include <stdio.h>
#include <string.h>
class GPIO : public QObject
{
    Q_OBJECT
public:
    explicit GPIO(QObject *parent = 0);
    ~GPIO();

    typedef std::pair<int,bool> PinOperation_t;

    int setGPIO_Out(int pin);
    int GPIO_Write(PinOperation_t pinOp);
    int unsetGPIO(int pin);
    int checkGPIOstatus(int pin);
};

#endif // GPIO_H
