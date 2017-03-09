#ifndef NMEAWIDGET_H
#define NMEAWIDGET_H

#include <QWidget>
#include "tcpbroadcast.h"

namespace Ui {
class NmeaWidget;
}

class NmeaWidget : public QWidget
{
    Q_OBJECT

public:
    explicit NmeaWidget(QWidget *parent = 0);
    ~NmeaWidget();
    void inputNmea(QByteArray msg);

private slots:
    void on_nmeaLogChooseButton_clicked();
    void on_nmeaLogActiveBox_toggled(bool checked);
    void on_nmeaServerActiveBox_toggled(bool checked);

private:
    Ui::NmeaWidget *ui;
    TcpBroadcast *mNmeaForwardServer;

};

#endif // NMEAWIDGET_H
