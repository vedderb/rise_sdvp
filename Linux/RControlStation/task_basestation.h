#ifndef BASESTATIONTASK_H
#define BASESTATIONTASK_H

#include "task.h"
#include "ublox.h"
#include <QTimer>
#include <QEventLoop>
#include <iostream>
#include <memory>
#include <QNetworkAccessManager>

typedef struct SatInfo
{
    SatInfo() {}
    int gpsVisible, gloVisible, galVisible, bdsVisible;
    int gpsUsed, gloUsed, galUsed, bdsUsed;
} SatInfo;

class Task_BaseStation : public Task
{
    Q_OBJECT
public:
    Task_BaseStation(QObject *parent = 0, int surveyInMinDuration = 300, double surveyInMinAcc = 2.0, int tcpPort = 2101, QString ubxSerialPort = "", int ubxBaudrate = 115200) : Task(parent) {
        mSurveyInMinAcc = surveyInMinDuration;
        mSurveyInMinAcc = surveyInMinAcc;
        mTcpPort = tcpPort;
        mUbxSerialPort = ubxSerialPort;
        mUbxBaudrate = ubxBaudrate;

        connect(this, &Task_BaseStation::gotAllInitInfo, &mUBXCommWaitLoop, &QEventLoop::quit);
        connect(&mUBXCommTimer, &QTimer::timeout, [this]{mUpdateConsoleTimer.stop(); std::cout << "\nu-blox serial communication timed out. Exiting.\n"; emit finished();});

        connect(&mUblox, &Ublox::rxMonVer, this, &Task_BaseStation::rxMonVer);
        connect(&mUblox, &Ublox::rxCfgGnss, this, &Task_BaseStation::rxCfgGnss);

        connect(&mUpdateConsoleTimer, &QTimer::timeout, this, &Task_BaseStation::updateConsoleOutput);

    }

signals:
    void gotAllInitInfo();

private slots:
    void rxMonVer(QString sw, QString hw, QStringList extensions);
    void rxCfgGnss(ubx_cfg_gnss cfg);
    void rxNavSat(ubx_nav_sat satellites);
    void rxSvin(ubx_nav_svin svin);
    void rtcmRx(QByteArray data, int type);
    void updateConsoleOutput();

private:
    QTimer mUBXCommTimer;
    QEventLoop mUBXCommWaitLoop;
    QTimer mUpdateConsoleTimer;

    Ublox mUblox;
    QMap<int, int> mRtcmUbx;
    SatInfo mSatInfo;
    ubx_nav_svin mLastSvin;

    int mTcpPort;
    TcpBroadcast mTcpServer;

    std::unique_ptr<QNetworkAccessManager> mNetworkAccessManager;

    bool mBasePosSet = false;

    int mSurveyInMinDuration; // seconds
    double mSurveyInMinAcc; // meters
    double mRefSendllh[3];

    QString mUbxSerialPort;
    int mUbxBaudrate;

    QString mMonVerString;
    QString mCfgGnssString;
    QString mExtIPstring;
    QString mIntIPstring;
    bool mPrintMonVer = false;
    bool mPrintCfgGnss = false;
    QString consoleOutputString;

    int ubxCommTimeoutDuration = 2000;

    void task();
    void getExternalIp();
    void getInternalIp();
};

#endif // BASESTATIONTASK_H
