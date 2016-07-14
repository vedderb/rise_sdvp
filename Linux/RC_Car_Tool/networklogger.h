#ifndef NETWORKLOGGER_H
#define NETWORKLOGGER_H

#include <QWidget>
#include <QTcpSocket>
#include <QFile>
#include <QList>
#include <QDateTime>
#include "datatypes.h"
#include "ping.h"
#include "locpoint.h"
#include "mapwidget.h"

namespace Ui {
class NetworkLogger;
}

class NetworkLogger : public QWidget
{
    Q_OBJECT

public:
    explicit NetworkLogger(QWidget *parent = 0);
    ~NetworkLogger();
    void setMap(MapWidget *map);

    typedef struct {
        QDateTime date;
        double llh[3];
        int fix_type;
        int datalen;
        QString pingRes;
        double pingMs;
        bool pingOk;
    } LOGPOINT;

private slots:
    void tcpInputConnected();
    void tcpInputDisconnected();
    void tcpInputDataAvailable();
    void tcpInputError(QAbstractSocket::SocketError socketError);
    void pingRx(int time, QString msg);
    void pingError(QString msg, QString error);

    void on_nmeaServerConnectButton_clicked();
    void on_pingTestButton_clicked();
    void on_logClearButton_clicked();
    void on_logFileChooseButton_clicked();
    void on_logFileActiveBox_toggled(bool checked);
    void on_statLogOpenButton_clicked();
    void on_statLogChooseButton_clicked();

private:
    Ui::NetworkLogger *ui;
    QTcpSocket *mTcpSocket;
    bool mTcpConnected;
    QString mFixNowStr;
    QString mSatNowStr;
    Ping *mPing;
    QFile mLog;
    LocPoint mLastPoint;
    MapWidget *mMap;
    QList<LOGPOINT> mLogRt;
    QList<LOGPOINT> mLogLoaded;

};

#endif // NETWORKLOGGER_H
