#include "rtcmwidget.h"
#include "ui_rtcmwidget.h"
#include <QSerialPortInfo>

RtcmWidget::RtcmWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RtcmWidget)
{
    ui->setupUi(this);
    mRtcm = new RtcmClient(this);
    mTimer = new QTimer(this);
    mTimer->start(20);

    connect(mRtcm, SIGNAL(rtcmReceived(QByteArray,int)),
            this, SLOT(rtcmRx(QByteArray,int)));
    connect(mTimer, SIGNAL(timeout()),
            this, SLOT(timerSlot()));

    on_rtcmSerialRefreshButton_clicked();
}

RtcmWidget::~RtcmWidget()
{
    delete ui;
}

void RtcmWidget::timerSlot()
{
    // Update ntrip connected label
    static bool wasNtripConnected = false;
    if (wasNtripConnected != mRtcm->isTcpConnected()) {
        wasNtripConnected = mRtcm->isTcpConnected();

        if (wasNtripConnected) {
            ui->ntripConnectedLabel->setText("Connected");
        } else {
            ui->ntripConnectedLabel->setText("Not connected");
        }
    }

    // Update serial connected label
    static bool wasSerialConnected = false;
    if (wasSerialConnected != mRtcm->isSerialConnected()) {
        wasSerialConnected = mRtcm->isSerialConnected();

        if (wasSerialConnected) {
            ui->rtcmSerialConnectedLabel->setText("Connected");
        } else {
            ui->rtcmSerialConnectedLabel->setText("Not connected");
        }
    }
}

void RtcmWidget::rtcmRx(QByteArray data, int type)
{
    emit rtcmReceived(data, type);

    switch (type) {
    case 1001: ui->rtcm1001Number->display(ui->rtcm1001Number->value() + 1); break;
    case 1002: ui->rtcm1002Number->display(ui->rtcm1002Number->value() + 1); break;
    case 1003: ui->rtcm1003Number->display(ui->rtcm1003Number->value() + 1); break;
    case 1004: ui->rtcm1004Number->display(ui->rtcm1004Number->value() + 1); break;

    case 1009: ui->rtcm1009Number->display(ui->rtcm1009Number->value() + 1); break;
    case 1010: ui->rtcm1010Number->display(ui->rtcm1010Number->value() + 1); break;
    case 1011: ui->rtcm1011Number->display(ui->rtcm1011Number->value() + 1); break;
    case 1012: ui->rtcm1012Number->display(ui->rtcm1012Number->value() + 1); break;

    case 1005: ui->rtcm1005Number->display(ui->rtcm1005Number->value() + 1); break;
    case 1006: ui->rtcm1006Number->display(ui->rtcm1006Number->value() + 1); break;

    case 1019: ui->rtcm1019Number->display(ui->rtcm1019Number->value() + 1); break;
    case 1020: ui->rtcm1020Number->display(ui->rtcm1020Number->value() + 1); break;
    default:
        break;
    }
}

void RtcmWidget::on_ntripConnectButton_clicked()
{
    if (ui->ntripBox->isChecked()) {
        mRtcm->connectNtrip(ui->ntripServerEdit->text(),
                            ui->ntripStreamEdit->text(),
                            ui->ntripUserEdit->text(),
                            ui->ntripPasswordEdit->text(),
                            ui->ntripPortBox->value());
    } else {
        mRtcm->connectTcp(ui->ntripServerEdit->text(), ui->ntripPortBox->value());
    }
}

void RtcmWidget::on_ntripDisconnectButton_clicked()
{
    mRtcm->disconnectTcpNtrip();
}

void RtcmWidget::on_resetAllCountersButton_clicked()
{
    ui->rtcm1001Number->display(0);
    ui->rtcm1002Number->display(0);
    ui->rtcm1003Number->display(0);
    ui->rtcm1004Number->display(0);

    ui->rtcm1009Number->display(0);
    ui->rtcm1010Number->display(0);
    ui->rtcm1011Number->display(0);
    ui->rtcm1012Number->display(0);

    ui->rtcm1005Number->display(0);
    ui->rtcm1006Number->display(0);

    ui->rtcm1019Number->display(0);
    ui->rtcm1020Number->display(0);
}

void RtcmWidget::on_ntripBox_toggled(bool checked)
{
    if (checked) {
        ui->ntripUserEdit->setEnabled(true);
        ui->ntripPasswordEdit->setEnabled(true);
        ui->ntripStreamEdit->setEnabled(true);
    } else {
        ui->ntripUserEdit->setEnabled(false);
        ui->ntripPasswordEdit->setEnabled(false);
        ui->ntripStreamEdit->setEnabled(false);
    }
}

void RtcmWidget::on_rtcmSerialRefreshButton_clicked()
{
    ui->rtcmSerialPortBox->clear();

    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    foreach(const QSerialPortInfo &port, ports) {
        ui->rtcmSerialPortBox->addItem(port.portName(), port.systemLocation());
    }

    ui->rtcmSerialPortBox->setCurrentIndex(0);
}

void RtcmWidget::on_rtcmSerialDisconnectButton_clicked()
{
    mRtcm->disconnectSerial();
}

void RtcmWidget::on_rtcmSerialConnectButton_clicked()
{
    mRtcm->connectSerial(ui->rtcmSerialPortBox->currentData().toString(),
                         ui->rtcmSerialBaudBox->value());
}
