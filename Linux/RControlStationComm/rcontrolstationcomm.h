#ifndef RCONTROLSTATIONCOMM_H
#define RCONTROLSTATIONCOMM_H

#include "rcontrolstationcomm_global.h"
#include "rcontrolstationcomm_types.h"
#include <QTcpSocket>
#include <QCoreApplication>

class RCONTROLSTATIONCOMMSHARED_EXPORT RControlStationComm
{

public:
    RControlStationComm();
    ~RControlStationComm();
    bool connectTcp(QString host, int port);
    void disconnectTcp();
    void setDebugLevel(int level);
    bool getState(int car, CAR_STATE *state, int timeoutMs = 5000);

private:
    QTcpSocket *mTcpSocket;
    int mDebugLevel;
    QByteArray mRxBuffer;
    QVector<QByteArray> mXmlBuffer;

    // CoreApplication
    QCoreApplication *mApp;
    int mAppArgc;
    char const *mAppArgv[2];

    void processData(const QByteArray &data);
    void sendData(const QByteArray &data);
    QByteArray waitForXml(int timeoutMs = 5000);
    bool isTcpConnected();
    QByteArray requestAnswer(int car, QString cmd, int timeoutMs = 5000);

};

#endif // RCONTROLSTATIONCOMM_H
