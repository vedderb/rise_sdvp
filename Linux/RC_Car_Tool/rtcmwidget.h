#ifndef RTCMWIDGET_H
#define RTCMWIDGET_H

#include <QWidget>
#include <QTimer>
#include "rtcmclient.h"

namespace Ui {
class RtcmWidget;
}

class RtcmWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RtcmWidget(QWidget *parent = 0);
    ~RtcmWidget();

signals:
    void rtcmReceived(QByteArray data, int type);

private slots:
    void timerSlot();
    void rtcmRx(QByteArray data, int type);

    void on_ntripConnectButton_clicked();
    void on_ntripDisconnectButton_clicked();

    void on_resetAllCountersButton_clicked();

private:
    Ui::RtcmWidget *ui;
    RtcmClient *mRtcm;
    QTimer *mTimer;
};

#endif // RTCMWIDGET_H
