#include "task_basestation.h"
#include "basestation.h"
#include <QDebug>


void Task_BaseStation::task()
{
//    qDebug() << "hello";
//    printf("%c[1A", 033); // back one line
//    printf("test");

    // TODO: parameterize from CLI
    QString serialPort = "ttyACM0";
    int baudrate = 115200;
    int rate = 1000;
    bool surveyIn = false;
    double surveyInMinAcc = 2.0;
    double refSendLat = 0;
    double refSendLon = 0;
    double refSendH = 0;
    bool isM8p = false;
    bool isF9p = true;
    bool mBasePosSet = false;


    bool res = mUblox.connectSerial(serialPort, baudrate);
    qDebug() << "Connection to UBX: " << res;
    if (res) {
        BaseStation::configureUbx(&mUblox, rate, isF9p, isM8p, surveyIn, surveyInMinAcc, &mBasePosSet, refSendLat, refSendLon, refSendH);
    }

    emit finished();
}
