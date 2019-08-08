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

    int setGPIO_Out(int pin);
    int GPIO_Write(int pin, int value);
    int unsetGPIO(int pin);
};

#endif // GPIO_H
