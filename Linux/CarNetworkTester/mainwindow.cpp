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
