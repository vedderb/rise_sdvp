#ifndef CARINTERFACE_H
#define CARINTERFACE_H

#include <QWidget>
#include <QVector>
#include <QTimer>
#include <QUdpSocket>
#include "datatypes.h"
#include "mapwidget.h"
#include "packetinterface.h"

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
    void setOrientation(double roll, double pitch, double yaw);
    void setStateData(CAR_STATE data);
    void setMap(MapWidget *map);
    void setPacketInterface(PacketInterface *packetInterface);
    void setKeyboardValues(double throttle, double steering);

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

    void on_terminalSendButton_clicked();
    void on_terminalClearButton_clicked();
    void on_yawOffsetSlider_valueChanged(int value);
    void on_idBox_valueChanged(int arg1);
    void on_magSampleClearButton_clicked();
    void on_magSampleSaveButton_clicked();
    void on_bldcToolUdpBox_toggled(bool checked);
    void on_autopilotBox_toggled(bool checked);
    void on_clearRouteButton_clicked();
    void on_servoDirectSlider_valueChanged(int value);
    void on_servoMappedSlider_valueChanged(int value);

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

};

#endif // CARINTERFACE_H
