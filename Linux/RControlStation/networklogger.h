/*
    Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

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
    void on_statHistRescaleButton_clicked();
    void on_statHistBinsBox_valueChanged(int arg1);

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

    void drawHistogram(int bins, const QList<LOGPOINT> &log);

};

#endif // NETWORKLOGGER_H
