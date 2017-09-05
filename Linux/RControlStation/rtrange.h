#ifndef RTRANGE_H
#define RTRANGE_H

#include <QWidget>
#include <QUdpSocket>
#include "mapwidget.h"

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

private slots:
    void readPendingDatagrams();
    void on_connectButton_clicked();
    void on_disconnectButton_clicked();

private:
    Ui::RtRange *ui;
    QUdpSocket *mUdpSocket;
    int mPacketCounter;
    MapWidget *mMap;

};

#endif // RTRANGE_H
