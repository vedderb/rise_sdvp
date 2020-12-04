/*
    Copyright 2017 Benjamin Vedder	benjamin@vedder.se

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

#ifndef COPTERINTERFACE_H
#define COPTERINTERFACE_H

#include <QWidget>
#include "datatypes.h"
#include "mapwidget.h"
#include "packetinterface.h"

#ifdef HAS_OPENGL
#include "orientationwidget.h"
#endif

namespace Ui {
class CopterInterface;
}

class CopterInterface : public QWidget
{
    Q_OBJECT

public:
    explicit CopterInterface(QWidget *parent = 0);
    ~CopterInterface();
    void setID(int id);
    int getId();
    bool pollData();
    bool updateRouteFromMap();
    void setOrientation(double roll, double pitch, double yaw);
    void setStateData(MULTIROTOR_STATE data);
    void setMap(MapWidget *map);
    void setPacketInterface(PacketInterface *packetInterface);
    void setControlValues(double throttle, double roll, double pitch, double yaw);
    void setCtrlAp();
    void setCtrlJs();
    bool setAp(bool on);
    QPair<int,int> getFirmwareVersion();

signals:
    void terminalCmd(quint8 id, QString cmd);
    void showStatusInfo(QString str, bool isGood);

private slots:
    void timerSlot();
    void terminalPrint(quint8 id, QString str);
    void routePointSet(LocPoint pos);
    void lastRoutePointRemoved();
    void nmeaReceived(quint8 id, QByteArray nmea_msg);
    void configurationReceived(quint8 id, MAIN_CONFIG config);
    void loadMagCal();

    void on_idBox_valueChanged(int arg1);
    void on_terminalSendButton_clicked();
    void on_confWriteButton_clicked();
    void on_confReadDefaultButton_clicked();
    void on_confReadButton_clicked();
    void on_setClockButton_clicked();
    void on_setClockPiButton_clicked();
    void on_rebootPiButton_clicked();
    void on_shutdownPiButton_clicked();

private:
    Ui::CopterInterface *ui;
    MapWidget *mMap;
    PacketInterface *mPacketInterface;

#ifdef HAS_OPENGL
    OrientationWidget *mOrientationWidget;
#endif

    int mId;
    QTimer *mTimer;
    bool settingsReadDone;

    QVector<double> mAltitudeData;
    QVector<double> mAltitudeXAxis;
    QPair<int,int> mFirmwareVersion;

    void getConfGui(MAIN_CONFIG &conf);
    void setConfGui(MAIN_CONFIG &conf);
    void setFirmwareVersion(QPair<int,int> firmwareVersion);

};

#endif // COPTERINTERFACE_H
