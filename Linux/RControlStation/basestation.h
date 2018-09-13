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

signals:
    void rtcmOut(QByteArray data);

private slots:
    void timerSlot();
    void rxRawx(ubx_rxm_rawx rawx);
    void rxNavSol(ubx_nav_sol sol);

    void on_ubxSerialRefreshButton_clicked();
    void on_ubxSerialDisconnectButton_clicked();
    void on_ubxSerialConnectButton_clicked();
    void on_tcpServerBox_toggled(bool checked);

private:
    Ui::BaseStation *ui;
    QTimer *mTimer;
    Ublox *mUblox;
    TcpBroadcast *mTcpServer;
    MapWidget *mMap;
    int mBasePosCnt;
    bool mBasePosSet;

};

#endif // BASESTATION_H
