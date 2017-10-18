#ifndef INTERSECTIONTEST_H
#define INTERSECTIONTEST_H

#include <QDialog>
#include "datatypes.h"
#include "carinterface.h"
#include "mapwidget.h"
#include "packetinterface.h"

namespace Ui {
class IntersectionTest;
}

class IntersectionTest : public QDialog
{
    Q_OBJECT

public:
    explicit IntersectionTest(QWidget *parent = 0);
    ~IntersectionTest();
    void setCars(QList<CarInterface*> *cars);
    void setMap(MapWidget *map);
    void setPacketInterface(PacketInterface *packet);

public slots:
    void nComRx(const ncom_data &data);

private slots:
    void on_runButton_clicked();
    void on_stopButton_clicked();

private:
    Ui::IntersectionTest *ui;
    QList<CarInterface*> *mCars;
    MapWidget *mMap;
    PacketInterface *mPacketInterface;
    ncom_data mRtRangeData;
    int mRtRangeInt;
    bool mRunning;

};

#endif // INTERSECTIONTEST_H
