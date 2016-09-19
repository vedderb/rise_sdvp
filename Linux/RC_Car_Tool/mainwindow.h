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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QList>
#include <QTimer>
#include <QSerialPort>
#include <QLabel>
#include "carinterface.h"
#include "packetinterface.h"
#include "ping.h"
#include "nmeaserver.h"

#ifdef HAS_JOYSTICK
#include "joystick.h"
#endif

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    bool eventFilter(QObject *object, QEvent *e);

private slots:
    void serialDataAvailable();
    void serialPortError(QSerialPort::SerialPortError error);
    void timerSlot();
    void showStatusInfo(QString info, bool isGood);
    void packetDataToSend(QByteArray &data);
    void stateReceived(quint8 id, CAR_STATE state);
    void mapPosSet(quint8 id, LocPoint pos);
    void ackReceived(quint8 id, CMD_PACKET cmd, QString msg);
    void rtcmReceived(QByteArray data, int type);
    void rtcmRefPosGet();
    void pingRx(int time, QString msg);
    void pingError(QString msg, QString error);
    void enuRx(quint8 id, double lat, double lon, double height);
    void nmeaGgaRx(int fields, NmeaServer::nmea_gga_info_t gga);

    void on_carAddButton_clicked();
    void on_carRemoveButton_clicked();
    void on_serialConnectButton_clicked();
    void on_serialRefreshButton_clicked();
    void on_disconnectButton_clicked();
    void on_mapRemoveTraceButton_clicked();
    void on_MapRemovePixmapsButton_clicked();
    void on_udpConnectButton_clicked();
    void on_mapZeroButton_clicked();
    void on_mapRemoveRouteButton_clicked();
    void on_mapRouteSpeedBox_valueChanged(double arg1);
    void on_jsConnectButton_clicked();
    void on_jsDisconnectButton_clicked();
    void on_mapAntialiasBox_toggled(bool checked);
    void on_carsWidget_tabCloseRequested(int index);
    void on_genCircButton_clicked();
    void on_mapSetAbsYawButton_clicked();
    void on_mapAbsYawSlider_valueChanged(int value);
    void on_mapAbsYawSlider_sliderReleased();
    void on_stopButton_clicked();
    void on_mapUploadRouteButton_clicked();
    void on_mapApButton_clicked();
    void on_mapKbButton_clicked();
    void on_mapOffButton_clicked();
    void on_mapUpdateSpeedButton_clicked();
    void on_udpPingButton_clicked();
    void on_mapOpenStreetMapBox_toggled(bool checked);
    void on_mapAntialiasOsmBox_toggled(bool checked);
    void on_mapOsmResSlider_valueChanged(int value);
    void on_mapChooseNmeaButton_clicked();
    void on_mapImportNmeaButton_clicked();
    void on_mapRemoveInfoButton_clicked();
    void on_traceInfoMinZoomBox_valueChanged(double arg1);
    void on_removeRouteExtraButton_clicked();
    void on_mapOsmClearCacheButton_clicked();
    void on_mapOsmServerOsmButton_toggled(bool checked);
    void on_mapOsmServerHiResButton_toggled(bool checked);
    void on_mapOsmServerVedderButton_toggled(bool checked);
    void on_mapOsmServerVedderHdButton_toggled(bool checked);
    void on_mapOsmMaxZoomBox_valueChanged(int arg1);
    void on_mapDrawGridBox_toggled(bool checked);
    void on_mapGetEnuButton_clicked();
    void on_mapSetEnuButton_clicked();
    void on_mapOsmStatsBox_toggled(bool checked);
    void on_removeTraceExtraButton_clicked();
    void on_mapEditHelpButton_clicked();
    void on_mapStreamNmeaConnectButton_clicked();
    void on_mapStreamNmeaDisconnectButton_clicked();
    void on_mapStreamNmeaClearTraceButton_clicked();

private:
    Ui::MainWindow *ui;
    QTimer *mTimer;
    QSerialPort *mSerialPort;
    PacketInterface *mPacketInterface;
    QList<CarInterface*> mCars;
    QLabel *mStatusLabel;
    int mStatusInfoTime;
    bool mKeyUp;
    bool mKeyDown;
    bool mKeyRight;
    bool mKeyLeft;
    double mThrottle;
    double mSteering;
    Ping *mPing;
    NmeaServer *mNmea;

#ifdef HAS_JOYSTICK
    Joystick *mJoystick;
    JS_TYPE mJsType;
#endif

};

#endif // MAINWINDOW_H
