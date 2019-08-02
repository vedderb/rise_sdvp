#ifndef GPIO_H
#define GPIO_H

//TODO Check which are used for other things and remove these.
#define GPIO_PINS 0, 1, 4, 7, 8, 9, 10, 11, 14, 15, 17, 18, 21, 22, 23, 24, 25
#define STR_LENGTH 64

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
