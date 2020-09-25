#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QSerialPortInfo>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    mTimer = new QTimer(this);
    mTimer->start(40);
    mStatusLabel = new QLabel(this);
    ui->statusBar->addPermanentWidget(mStatusLabel);
    mStatusInfoTime = 0;
    mSerialPort = new QSerialPort(this);

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    connect(mSerialPort, SIGNAL(readyRead()), this, SLOT(serialDataAvailable()));
    connect(mSerialPort, SIGNAL(error(QSerialPort::SerialPortError)),
            this, SLOT(serialPortError(QSerialPort::SerialPortError)));

    on_serialRefreshButton_clicked();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::serialDataAvailable()
{
    while (mSerialPort->bytesAvailable() > 0) {
        QByteArray data = mSerialPort->readAll();
        // TODO!
    }
}

void MainWindow::serialPortError(QSerialPort::SerialPortError error)
{
    QString message;
    switch (error) {
    case QSerialPort::NoError:
        break;

    default:
        message = "Serial port error: " + mSerialPort->errorString();
        break;
    }

    if(!message.isEmpty()) {
        showStatusInfo(message, false);

        if(mSerialPort->isOpen()) {
            mSerialPort->close();
        }
    }
}

void MainWindow::timerSlot()
{
    // Update status label
    if (mStatusInfoTime) {
        mStatusInfoTime--;
        if (!mStatusInfoTime) {
            mStatusLabel->setStyleSheet(qApp->styleSheet());
        }
    } else {
        if (mSerialPort->isOpen()) {
            mStatusLabel->setText("Connected");
        } else {
            mStatusLabel->setText("Not connected");
        }
    }
}

void MainWindow::showStatusInfo(QString info, bool isGood)
{
    if (isGood) {
        mStatusLabel->setStyleSheet("QLabel { background-color : lightgreen; color : black; }");
    } else {
        mStatusLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
    }

    mStatusInfoTime = 80;
    mStatusLabel->setText(info);
}

void MainWindow::on_serialRefreshButton_clicked()
{
    ui->serialPortBox->clear();

    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    foreach(const QSerialPortInfo &port, ports) {
        QString name = port.portName();
        int index = ui->serialPortBox->count();
        // put ns-raw device first in list and add prefix
        if(port.manufacturer() == "Prolific Technology Inc.") {
            name.insert(0, "NS-RAW - ");
            index = 0;
        }
        ui->serialPortBox->insertItem(index, name, port.systemLocation());
    }

    ui->serialPortBox->setCurrentIndex(0);
}

void MainWindow::on_serialConnectButton_clicked()
{
    if(mSerialPort->isOpen()) {
        return;
    }

    mSerialPort->setPortName(ui->serialPortBox->currentData().toString());
    mSerialPort->open(QIODevice::ReadWrite);

    if(!mSerialPort->isOpen()) {
        return;
    }

    mSerialPort->setBaudRate(QSerialPort::Baud115200);
    mSerialPort->setDataBits(QSerialPort::Data8);
    mSerialPort->setParity(QSerialPort::NoParity);
    mSerialPort->setStopBits(QSerialPort::OneStop);
    mSerialPort->setFlowControl(QSerialPort::NoFlowControl);
}

void MainWindow::on_setUpdateRateButton_clicked()
{
    QByteArray pl;

    pl.append(0x0E);
    pl.append(ui->updateRateBox->currentText().toInt());
    pl.append(ui->updateRateFlashBox->isChecked());

    sendPacket(pl);
}

void MainWindow::on_serialDisconnectButton_clicked()
{
    if (mSerialPort->isOpen()) {
        mSerialPort->close();
    }
}

void MainWindow::sendPacket(QByteArray payload)
{
    QByteArray packet;
    packet.append(0xA0);
    packet.append(0xA1);
    packet.append((payload.size() >> 8) & 0xFF);
    packet.append(payload.size() & 0xFF);
    packet.append(payload);

    char cs = 0;
    for (int i = 0;i < payload.size();i++) {
        cs ^= payload[i];
    }

    packet.append(cs);
    packet.append(0x0D);
    packet.append(0x0A);

    if (mSerialPort->isOpen()) {
        mSerialPort->write(packet);
    }
}
