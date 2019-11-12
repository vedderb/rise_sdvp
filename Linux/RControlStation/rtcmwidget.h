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

#ifndef RTCMWIDGET_H
#define RTCMWIDGET_H

#include <QWidget>
#include <QTimer>
#include "rtcmclient.h"
#include "tcpbroadcast.h"

namespace Ui {
class RtcmWidget;
}

class RtcmWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RtcmWidget(QWidget *parent = 0);
    ~RtcmWidget();
    void setRefPos(double lat, double lon, double height, double antenna_height = 0.0);

signals:
    void rtcmReceived(QByteArray data);
    void refPosGet();

private slots:
    void timerSlot();
    void rtcmRx(QByteArray data, int type, bool sync);
    void refPosRx(double lat, double lon, double height, double antenna_height);

    void on_ntripConnectButton_clicked();
    void on_ntripDisconnectButton_clicked();
    void on_resetAllCountersButton_clicked();
    void on_ntripBox_toggled(bool checked);
    void on_rtcmSerialRefreshButton_clicked();
    void on_rtcmSerialDisconnectButton_clicked();
    void on_rtcmSerialConnectButton_clicked();
    void on_refGetButton_clicked();
    void on_tcpServerBox_toggled(bool checked);
    void on_gpsOnlyBox_toggled(bool checked);

private:
    Ui::RtcmWidget *ui;
    RtcmClient *mRtcm;
    QTimer *mTimer;
    TcpBroadcast *mTcpServer;
};

#endif // RTCMWIDGET_H
