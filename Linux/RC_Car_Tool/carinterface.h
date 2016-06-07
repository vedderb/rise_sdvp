/*
    Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#ifndef CARINTERFACE_H
#define CARINTERFACE_H

#include <QWidget>
#include <QVector>
#include <QTimer>
#include <QUdpSocket>
#include "datatypes.h"
#include "mapwidget.h"
#include "packetinterface.h"
#include "tcpbroadcast.h"

namespace Ui {
class CarInterface;
}

class CarInterface : public QWidget
{
    Q_OBJECT

public:
    explicit CarInterface(QWidget *parent = 0);
    ~CarInterface();
    void setID(int id);
    int getId();
    bool pollData();
    bool updateRouteFromMap();
    void setOrientation(double roll, double pitch, double yaw);
    void setStateData(CAR_STATE data);
    void setMap(MapWidget *map);
    void setPacketInterface(PacketInterface *packetInterface);
    void setControlValues(double throttle, double steering, double max, bool currentMode);
    void emergencyStop();
    void setCtrlAp();
    void setCtrlKb();
    bool setAp(bool on);

signals:
    void terminalCmd(quint8 id, QString cmd);
    void forwardVesc(quint8 id, QByteArray data);
    void setRcCurrent(quint8 id, double current, double steering);
    void setRcDuty(quint8 id, double duty, double steering);
    void showStatusInfo(QString str, bool isGood);
    void setServoDirect(quint8 id, double value);

private slots:
    void timerSlot();
    void udpReadReady();
    void terminalPrint(quint8 id, QString str);
    void vescFwdReceived(quint8 id, QByteArray data);
    void routePointSet(LocPoint pos);
    void lastRoutePointRemoved();
    void nmeaReceived(quint8 id, QByteArray nmea_msg);
    void configurationReceived(quint8 id, MAIN_CONFIG config);

    void on_terminalSendButton_clicked();
    void on_terminalClearButton_clicked();
    void on_idBox_valueChanged(int arg1);
    void on_magSampleClearButton_clicked();
    void on_magSampleSaveButton_clicked();
    void on_bldcToolUdpBox_toggled(bool checked);
    void on_autopilotBox_toggled(bool checked);
    void on_clearRouteButton_clicked();
    void on_servoDirectSlider_valueChanged(int value);
    void on_servoMappedSlider_valueChanged(int value);
    void on_nmeaServerActiveBox_toggled(bool checked);
    void on_confReadButton_clicked();
    void on_confReadDefaultButton_clicked();
    void on_confWriteButton_clicked();
    void on_nmeaLogChooseButton_clicked();
    void on_nmeaLogActiveBox_toggled(bool checked);
    void on_magCalChooseButton_clicked();
    void on_magCalLoadButton_clicked();

private:
    Ui::CarInterface *ui;
    QVector<double> accelXData;
    QVector<double> accelYData;
    QVector<double> accelZData;
    QVector<double> gyroXData;
    QVector<double> gyroYData;
    QVector<double> gyroZData;
    QVector<double> magXData;
    QVector<double> magYData;
    QVector<double> magZData;
    QVector<double> accelGyroMagXAxis;
    int maxSampleSize;
    MapWidget *mMap;
    PacketInterface *mPacketInterface;

    int mId;
    QVector<QVector<double> > mMagSamples;
    QTimer *mTimer;
    QUdpSocket *mUdpSocket;
    QHostAddress mLastHostAddress;
    quint16 mUdpPort;
    TcpBroadcast *mNmeaForwardServer;

    void getConfGui(MAIN_CONFIG &conf);
    void setConfGui(MAIN_CONFIG &conf);

};

#endif // CARINTERFACE_H
