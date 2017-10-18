#ifndef RTRANGE_H
#define RTRANGE_H

#include <QWidget>
#include <QUdpSocket>
#include <QTimer>
#include "mapwidget.h"
#include "datatypes.h"

namespace Ui {
class NCom;
}

class NCom : public QWidget
{
    Q_OBJECT

public:
    explicit NCom(QWidget *parent = 0);
    ~NCom();
    void setMap(MapWidget *map);
    void sendNcom(double *illh,
                  double px,
                  double py,
                  double pz,
                  double heading,
                  double vel);

signals:
    void dataRx(const ncom_data &data);

private slots:
    void timerSlot();
    void readPendingDatagrams();
    void on_connectButton_clicked();
    void on_disconnectButton_clicked();
    void on_ipAnyBox_toggled(bool checked);
    void on_mapDrawCarBox_toggled(bool checked);

private:
    Ui::NCom *ui;
    QUdpSocket *mUdpSocket;
    int mPacketCounter;
    MapWidget *mMap;
    unsigned int mMapCnt;
    ncom_data mData;
    QTimer *mTimer;
    bool mUpdateMap;

};

#endif // RTRANGE_H
