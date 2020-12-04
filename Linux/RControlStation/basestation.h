/*
    Copyright 2016 - 2018 Benjamin Vedder	benjamin@vedder.se

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

#ifndef BASESTATION_H
#define BASESTATION_H

#include <QWidget>
#include <QTcpSocket>
#include <QTimer>
#include <QMap>
#include "ublox.h"
#include "tcpbroadcast.h"
#include "mapwidget.h"

namespace Ui {
class BaseStation;
}

class BaseStation : public QWidget
{
    Q_OBJECT

public:
    explicit BaseStation(QWidget *parent = 0);
    ~BaseStation();
    void setMap(MapWidget *map);

    // TODO: move to u-blox?
    enum BaseStationPositionMode {FIXED, MOVING_BASE, SURVEY_IN};
    static void configureUbx(Ublox *ublox, unsigned int baudrate, int rate_meas, int rate_nav, bool isF9p, bool isM8p, bool *basePosSet, double refSendLat, double refSendLon, double refSendH, BaseStationPositionMode positionMode, double surveyInMinAcc, int surveyInMinDuration = 20);

signals:
    void rtcmOut(QByteArray data);

private slots:
    void timerSlot();
    void rxRawx(ubx_rxm_rawx rawx);
    void rxNavSol(ubx_nav_sol sol);
    void rxNavSat(ubx_nav_sat sat);
    void rxSvin(ubx_nav_svin svin);
    void rtcmRx(QByteArray data, int type);

    void on_ubxSerialRefreshButton_clicked();
    void on_ubxSerialDisconnectButton_clicked();
    void on_ubxSerialConnectButton_clicked();
    void on_tcpServerBox_toggled(bool checked);

    void on_readVersionButton_clicked();

    void on_gnssInfoButton_clicked();

    void on_surveyInRadioButton_toggled(bool checked);

    void on_m8Button_toggled(bool checked);

    void on_movingBaseRadioButton_toggled(bool checked);

private:
    Ui::BaseStation *ui;
    QTimer *mTimer;
    Ublox *mUblox;
    TcpBroadcast *mTcpServer;
    MapWidget *mMap;
    bool mBasePosSet;
    QMap<int, int> mRtcmUbx;

};

#endif // BASESTATION_H
