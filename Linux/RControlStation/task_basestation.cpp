#include "task_basestation.h"
#include "basestation.h"
#include "utility.h"
#include <QDebug>
#include <QJsonObject>
#include <QJsonDocument>
#include <QSerialPortInfo>
#include <QNetworkInterface>

#define RTCM_REF_MSG_DELAY_MULT 5

void Task_BaseStation::getExternalIp()
{
    QUrl url("https://api.ipify.org?format=json");
    mExtIPstring = "testing...";

    connect(mNetworkAccessManager.get(), &QNetworkAccessManager::finished, [&](QNetworkReply* reply){
        reply->deleteLater();

        if (reply->error()) {
            mExtIPstring = "uknown.";
        } else {
            QJsonObject jsonObject = QJsonDocument::fromJson(reply->readAll()).object();
            QHostAddress ip(jsonObject["ip"].toString());

            mExtIPstring = ip.toString();
        }
    });

    mNetworkAccessManager->get(QNetworkRequest(url));
}

void Task_BaseStation::getInternalIp() {
    mIntIPstring = "";
    for (QHostAddress addr : QNetworkInterface::allAddresses())
        if (addr.protocol() == QAbstractSocket::IPv4Protocol && !addr.isLoopback())
            mIntIPstring += QString(addr.toString() + " ");
}

void Task_BaseStation::task()
{
    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    QSerialPortInfo ubxSerialPort;
    if (mUbxSerialPort.isEmpty()) {
        foreach(const QSerialPortInfo &port, ports)
            if (port.manufacturer().toLower().replace("-", "").contains("ublox"))
                ubxSerialPort = port;
    } else {
        foreach(const QSerialPortInfo &port, ports)
            if (port.systemLocation() == mUbxSerialPort)
                ubxSerialPort = port;
    }

    if (ubxSerialPort.isNull()) {
        std::cerr << "Could not find u-blox serial interface. Try specifying it, see help. Exiting.\n";
        emit finished();
        return;
    }

    if (!mUblox.connectSerial(ubxSerialPort.systemLocation(), mUbxBaudrate)) {
        std::cerr << "Unable to connect serial to " << ubxSerialPort.systemLocation().toStdString() << ". Exiting.\n";
        emit finished();
        return;
    }
    qDebug() << "Serial connection to " << ubxSerialPort.systemLocation() << " established.";

    // TODO: allow override over CLI
    double refSendLat = -90;
    double refSendLon = 0;
    double refSendH = -5000;

    // Note: only F9P (no rawx processing) supported in console basestation mode, currently no plans to change that
    bool isM8p = false;
    bool isF9p = true;
    bool mBasePosSet = false;

    int rate_meas = 1000;
    ubxCommTimeoutDuration = 2*rate_meas;
    int rate_nav = 1;

    mUBXCommTimer.start(ubxCommTimeoutDuration);
    BaseStation::configureUbx(&mUblox, 921000, rate_meas, rate_nav, isF9p, isM8p, &mBasePosSet, refSendLat, refSendLon, refSendH, BaseStation::BaseStationPositionMode::SURVEY_IN, mSurveyInMinAcc, mSurveyInMinDuration);

    mUblox.ubxPoll(UBX_CLASS_MON, UBX_MON_VER);
    mUblox.ubxPoll(UBX_CLASS_CFG, UBX_CFG_GNSS);    

    // Wait for UBX messages to arrive, proceed only then
    mUBXCommWaitLoop.exec();

    if (mPrintMonVer)
    std::cout << mMonVerString.toStdString() << std::endl;
    if (mPrintCfgGnss)
        std::cout << mCfgGnssString.toStdString() << std::endl;
    std::cout << std::endl;

    connect(&mUblox, &Ublox::rxNavSat, this, &Task_BaseStation::rxNavSat);
    connect(&mUblox, &Ublox::rxSvin,   this, &Task_BaseStation::rxSvin);
    connect(&mUblox, &Ublox::rtcmRx,   this, &Task_BaseStation::rtcmRx);

    mNetworkAccessManager.reset(new QNetworkAccessManager(this));
    getExternalIp();
    getInternalIp();

    mUpdateConsoleTimer.start(500);

    mTcpServer.startTcpServer(mTcpPort);
}

void Task_BaseStation::rxMonVer(QString sw, QString hw, QStringList extensions)
{
    mUBXCommTimer.start(ubxCommTimeoutDuration);
    mMonVerString = "SW: " + sw + "\tHW: " + hw + "\tExtensions: ";
    mMonVerString += extensions.join(", ");
    if (!mCfgGnssString.isEmpty())
        emit gotAllInitInfo();
}

void Task_BaseStation::rxCfgGnss(ubx_cfg_gnss cfg)
{
    mUBXCommTimer.start(ubxCommTimeoutDuration);
    mCfgGnssString = QString("TrkChHw   : %1\t TrkChUse  : %2\t Blocks    : %3\n").
            arg(cfg.num_ch_hw).arg(cfg.num_ch_use).arg(cfg.num_blocks);

    for (int i = 0;i < cfg.num_blocks;i++) {
        mCfgGnssString += QString("GNSS ID: %1, Enabled: %2\t"
                       "MinTrkCh  : %3\t"
                       "MaxTrkCh  : %4\t"
                       "Flags     : %5").
                arg(cfg.blocks[i].gnss_id).
                arg(cfg.blocks[i].en).
                arg(cfg.blocks[i].minTrkCh).
                arg(cfg.blocks[i].maxTrkCh).
                arg(cfg.blocks[i].flags);

        if (i != cfg.num_blocks - 1) {
            mCfgGnssString += "\n";
        }
    }

    if (!mMonVerString.isEmpty())
        emit gotAllInitInfo();
}

void Task_BaseStation::rxNavSat(ubx_nav_sat satellites)
{
    mUBXCommTimer.start(ubxCommTimeoutDuration);
    memset(&mSatInfo, 0, sizeof(SatInfo));

    for (int i = 0;i < satellites.num_sv;i++) {
        ubx_nav_sat_info sat = satellites.sats[i];

        switch (sat.gnss_id) {
        case 0: mSatInfo.gpsVisible++; if (sat.used && sat.quality >= 4) mSatInfo.gpsUsed++; break;
        case 2: mSatInfo.galVisible++; if (sat.used && sat.quality >= 4) mSatInfo.galUsed++; break;
        case 3: mSatInfo.bdsVisible++; if (sat.used && sat.quality >= 4) mSatInfo.bdsUsed++; break;
        case 6: mSatInfo.gloVisible++; if (sat.used && sat.quality >= 4) mSatInfo.gloUsed++; break;
        default: break;
        }
    }
}

void Task_BaseStation::rxSvin(ubx_nav_svin svin)
{
    mUBXCommTimer.start(ubxCommTimeoutDuration);
    double llh[3];
    utility::xyzToLlh(svin.meanX, svin.meanY, svin.meanZ,
                      &llh[0], &llh[1], &llh[2]);

    memcpy(mRefSendllh, llh, sizeof(double[3]));
    mLastSvin = svin;
}

void Task_BaseStation::rtcmRx(QByteArray data, int type)
{
    mUBXCommTimer.start(ubxCommTimeoutDuration);
    // Send base station position every RTCM_REF_MSG_DELAY_MULT cycles to save bandwidth.
    static int basePosCnt = 0;
    if (type == 1006 || type == 1005) {
        basePosCnt++;
        if (basePosCnt < RTCM_REF_MSG_DELAY_MULT)
            return;
        basePosCnt = 0;
    }

    mRtcmUbx[type]++;
    if (mTcpServer.isRunning()) {
        mTcpServer.broadcastData(data);
    }
}

void Task_BaseStation::updateConsoleOutput()
{
    QString surveyInfoString = QString(
        "RTK Reference Station. Sending RTCM3 on port %1.                 \n"
        "External IP: %10  Internal IP: %11                                \n"
        "----------------------------------------------------------------------------------------------\n"
        "Position  - Lat: %2   Lon: %3   Height: %4          \n"
        "Survey-In - Valid: %8   Active: %9   Duration: %7s   Accuracy: %6m   Observations: %5         \n"
        "----------------------------------------------------------------------------------------------\n").
        arg(mTcpPort).
        arg(mRefSendllh[0], 0, 'f', 8).
        arg(mRefSendllh[1], 0, 'f', 8).
        arg(mRefSendllh[2]).
        arg(mLastSvin.obs).
        arg(mLastSvin.meanAcc).
        arg(mLastSvin.dur).
        arg(mLastSvin.valid).
        arg(mLastSvin.active).
        arg(mExtIPstring).
        arg(mIntIPstring);

    QString satInfoString = QString(
        "Satellite Info.\n"
        " GPS     -\t%2 / %1   used / visible          \n"
        " Galileo -\t%4 / %3   used / visible          \n"
        " BeiDou  -\t%6 / %5   used / visible          \n"
        " GLONASS -\t%8 / %7   used / visible          \n"
        " Total   -\t%10 / %9   used / visible         \n"
        "----------------------------------------------------------------------------------------------\n").
            arg(mSatInfo.gpsVisible, 3).
            arg(mSatInfo.gpsUsed, 3).
            arg(mSatInfo.galVisible, 3).
            arg(mSatInfo.galUsed, 3).
            arg(mSatInfo.bdsVisible, 3).
            arg(mSatInfo.bdsUsed, 3).
            arg(mSatInfo.gloVisible, 3).
            arg(mSatInfo.gloUsed, 3).
            arg(mSatInfo.gpsVisible+mSatInfo.galVisible+mSatInfo.bdsVisible+mSatInfo.gloVisible, 3).
            arg(mSatInfo.gpsUsed+mSatInfo.galUsed+mSatInfo.bdsUsed+mSatInfo.gloUsed, 3);

    QString rtcmTransmittedString = QString(
                "RTCM Transmitted:");

    for (int i = 0; i < mRtcmUbx.size(); i++) {
        if (i%5 == 0)
            rtcmTransmittedString += "\n ";
        rtcmTransmittedString += QString("%1 sent:%2   ").arg(mRtcmUbx.keys()[i]).arg(mRtcmUbx.values()[i], 6);
    }

    consoleOutputString = surveyInfoString + satInfoString + rtcmTransmittedString + "\n";

    static int lastNumLines = 0;
    if (lastNumLines != 0) // go back to beginning
        printf("%c[%dA",  033, lastNumLines+1);
    std::cout << consoleOutputString.toStdString() << std::endl;
    lastNumLines = consoleOutputString.count("\n");
}
