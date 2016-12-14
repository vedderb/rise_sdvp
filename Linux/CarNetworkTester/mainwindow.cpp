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

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QXmlStreamWriter>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    mTcpSocket = new QTcpSocket(this);
    mUdpSocket = new QUdpSocket(this);
    mTcpConnected = false;

    connect(mTcpSocket, SIGNAL(readyRead()), this, SLOT(tcpInputDataAvailable()));
    connect(mTcpSocket, SIGNAL(connected()), this, SLOT(tcpInputConnected()));
    connect(mTcpSocket, SIGNAL(disconnected()),
            this, SLOT(tcpInputDisconnected()));
    connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(tcpInputError(QAbstractSocket::SocketError)));
    connect(mUdpSocket, SIGNAL(readyRead()),
            this, SLOT(udpReadReady()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::tcpInputConnected()
{
    mTcpConnected = true;
    ui->disconnectedButton->setEnabled(true);
    ui->tcpButton->setEnabled(true);
    ui->udpButton->setEnabled(true);
}

void MainWindow::tcpInputDisconnected()
{
    mTcpConnected = false;
}

void MainWindow::tcpInputDataAvailable()
{
    ui->incomingEdit->appendPlainText(QString::fromUtf8(mTcpSocket->readAll()));
}

void MainWindow::tcpInputError(QAbstractSocket::SocketError socketError)
{
    (void)socketError;

    QString errorStr = mTcpSocket->errorString();
    QMessageBox::warning(0, "TCP Error", errorStr);

    mTcpSocket->close();
    mTcpConnected = false;
    ui->disconnectedButton->setChecked(true);
}

void MainWindow::udpReadReady()
{
    while (mUdpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(mUdpSocket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        mUdpSocket->readDatagram(datagram.data(), datagram.size(),
                                 &sender, &senderPort);

        (void)sender;
        ui->incomingEdit->appendPlainText(QString::fromUtf8(datagram));
    }
}

void MainWindow::on_sendButton_clicked()
{
    if (ui->tcpButton->isChecked()) {
        mTcpSocket->write(ui->outgoingEdit->toPlainText().toUtf8());
    } else if (ui->udpButton->isChecked()) {

        mUdpSocket->writeDatagram(ui->outgoingEdit->toPlainText().toUtf8(),
                                  QHostAddress(ui->serverEdit->text()), ui->portBox->value());
    } else {
        QMessageBox::warning(this, "Send Error",
        "You are not connected.");
    }
}

void MainWindow::on_clearButton_clicked()
{
    ui->incomingEdit->clear();
}

void MainWindow::on_disconnectedButton_toggled(bool checked)
{
    if (checked) {
        ui->portBox->setEnabled(true);
        ui->serverEdit->setEnabled(true);
        ui->disconnectedButton->setEnabled(true);
        ui->tcpButton->setEnabled(true);
        ui->udpButton->setEnabled(true);
    }
}

void MainWindow::on_tcpButton_toggled(bool checked)
{
    if (checked) {
        mTcpSocket->connectToHost(ui->serverEdit->text(), ui->portBox->value());

        ui->portBox->setEnabled(false);
        ui->serverEdit->setEnabled(false);
        ui->disconnectedButton->setEnabled(false);
        ui->tcpButton->setEnabled(false);
        ui->udpButton->setEnabled(false);
    } else {
        if (mTcpConnected) {
            mTcpSocket->abort();
            mTcpConnected = false;
        }
    }
}

void MainWindow::on_udpButton_toggled(bool checked)
{
    if (checked) {
        if (mUdpSocket->bind(QHostAddress::Any, ui->portBox->value() + 1)) {
            ui->portBox->setEnabled(false);
            ui->serverEdit->setEnabled(false);
        } else {
            qWarning() << "Binding UDP socket failed.";
            QMessageBox::warning(this, "UDP Error",
                                 "Listening on UDP port failed. Make sure that the port is not "
                                 "already in use.");
            ui->disconnectedButton->setChecked(true);
        }
    } else {
        mUdpSocket->close();
    }
}

void MainWindow::on_getStateGenerateButton_clicked()
{
    QString str;
    QXmlStreamWriter stream(&str);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("getState");
    stream.writeTextElement("id", QString::number(ui->getStateCarBox->value()));
    stream.writeEndDocument();

    ui->outgoingEdit->clear();
    ui->outgoingEdit->appendPlainText(str);
}

void MainWindow::on_addRoutePointGenerateButton_clicked()
{
    QString str;
    QXmlStreamWriter stream(&str);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("addRoutePoint");
    stream.writeTextElement("id", QString::number(ui->addRoutePointCarBox->value()));
    stream.writeTextElement("px", QString::number(ui->addRoutePointPxBox->value()));
    stream.writeTextElement("py", QString::number(ui->addRoutePointPyBox->value()));
    stream.writeTextElement("speed", QString::number(ui->addRoutePointSpeedBox->value()));
    stream.writeTextElement("time", QString::number(ui->addRoutePointTimeEdit->time().msecsSinceStartOfDay()));
    stream.writeEndDocument();

    ui->outgoingEdit->clear();
    ui->outgoingEdit->appendPlainText(str);
}

void MainWindow::on_removeLastPointGenerateButton_clicked()
{
    QString str;
    QXmlStreamWriter stream(&str);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("removeLastPoint");
    stream.writeTextElement("id", QString::number(ui->removeLastPointCarBox->value()));
    stream.writeEndDocument();

    ui->outgoingEdit->clear();
    ui->outgoingEdit->appendPlainText(str);
}

void MainWindow::on_clearRouteGenerateButton_clicked()
{
    QString str;
    QXmlStreamWriter stream(&str);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("clearRoute");
    stream.writeTextElement("id", QString::number(ui->removeLastPointCarBox->value()));
    stream.writeEndDocument();

    ui->outgoingEdit->clear();
    ui->outgoingEdit->appendPlainText(str);
}

void MainWindow::on_setAutopilotActiveGenerateButton_clicked()
{
    QString str;
    QXmlStreamWriter stream(&str);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("setAutopilotActive");
    stream.writeTextElement("id", QString::number(ui->setAutopilotActiveCarBox->value()));
    stream.writeTextElement("enabled", QString::number(ui->setAutopilotActiveEnabledButton->isChecked()));
    stream.writeEndDocument();

    ui->outgoingEdit->clear();
    ui->outgoingEdit->appendPlainText(str);
}

void MainWindow::on_getEnuRefGenerateButton_clicked()
{
    QString str;
    QXmlStreamWriter stream(&str);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("getEnuRef");
    stream.writeTextElement("id", QString::number(ui->getEnuRefCarBox->value()));
    if (ui->getEnuRefFromMapBox->isChecked()) {
        stream.writeTextElement("fromMap", QString::number(1));
    }
    stream.writeEndDocument();

    ui->outgoingEdit->clear();
    ui->outgoingEdit->appendPlainText(str);
}

void MainWindow::on_setEnuRefGenerateButton_clicked()
{
    QString str;
    QXmlStreamWriter stream(&str);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("setEnuRef");
    stream.writeTextElement("id", QString::number(ui->setEnuRefCarBox->value()));
    stream.writeTextElement("lat", QString::number(ui->setEnuRefLatBox->value(), 'g', 10));
    stream.writeTextElement("lon", QString::number(ui->setEnuRefLonBox->value(), 'g', 10));
    stream.writeTextElement("height", QString::number(ui->setEnuRefHeightBox->value()));
    stream.writeEndDocument();

    ui->outgoingEdit->clear();
    ui->outgoingEdit->appendPlainText(str);
}

void MainWindow::on_rcControlGenerateButton_clicked()
{
    QString str;
    QXmlStreamWriter stream(&str);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("rcControl");

    // RC_MODE_CURRENT = 0,
    // RC_MODE_DUTY,
    // RC_MODE_PID,
    // RC_MODE_CURRENT_BRAKE
    int mode = -1;
    if (ui->rcControlCurrentButton->isChecked()) {
        mode = 0;
    } else if (ui->rcControlDutyButton->isChecked()) {
        mode = 1;
    } else if (ui->rcControlSpeedButton->isChecked()) {
        mode = 2;
    } else if (ui->rcControlCurrentBrakeButton->isChecked()) {
        mode = 3;
    }

    stream.writeTextElement("id", QString::number(ui->rcControlCarBox->value()));
    stream.writeTextElement("mode", QString::number(mode));
    stream.writeTextElement("value", QString::number(ui->rcControlValueBox->value()));
    stream.writeTextElement("steering", QString::number((double)ui->rcControlSteeringSlider->value() / 100.0));
    stream.writeEndDocument();

    ui->outgoingEdit->clear();
    ui->outgoingEdit->appendPlainText(str);
}
