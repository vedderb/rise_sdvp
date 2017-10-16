#ifndef RTRANGE_H
#define RTRANGE_H

#include <QWidget>
#include <QUdpSocket>
#include <QTimer>
#include "mapwidget.h"
#include "datatypes.h"

namespace Ui {
class RtRange;
}

class RtRange : public QWidget
{
    Q_OBJECT

public:
    explicit RtRange(QWidget *parent = 0);
    ~RtRange();
    void setMap(MapWidget *map);
    void sendNcom(double *illh,
                  double px,
                  double py,
                  double pz,
                  double heading,
                  double vel);

signals:
    void dataRx(const rt_range_data &data);

private slots:
    void timerSlot();
    void readPendingDatagrams();
    void on_connectButton_clicked();
    void on_disconnectButton_clicked();
    void on_ipAnyBox_toggled(bool checked);
    void on_mapDrawCarBox_toggled(bool checked);

private:
    Ui::RtRange *ui;
    QUdpSocket *mUdpSocket;
    int mPacketCounter;
    MapWidget *mMap;
    unsigned int mMapCnt;
    rt_range_data mData;
    QTimer *mTimer;
    bool mUpdateMap;

};

#endif // RTRANGE_H
