#include "networkinterface.h"
#include "ui_networkinterface.h"
#include <QMessageBox>

NetworkInterface::NetworkInterface(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::NetworkInterface)
{
    ui->setupUi(this);

    mTcpServer = new TcpServerSimple(this);
    mUdpSocket = new QUdpSocket(this);
    mLastHostAddress.clear();
    mMap = 0;
    mPacketInterface = 0;

    connect(mTcpServer, SIGNAL(dataRx(QByteArray)),
            this, SLOT(tcpDataRx(QByteArray)));
    connect(mTcpServer, SIGNAL(connectionChanged(bool)),
            this, SLOT(tcpConnectionChanged(bool)));
    connect(mUdpSocket, SIGNAL(readyRead()),
            this, SLOT(udpReadReady()));

    tcpConnectionChanged(false);
}

NetworkInterface::~NetworkInterface()
{
    delete ui;
}

void NetworkInterface::setMap(MapWidget *map)
{
    mMap = map;
}

void NetworkInterface::setPacketInterface(PacketInterface *packetInterface)
{
    mPacketInterface = packetInterface;

    connect(mPacketInterface, SIGNAL(stateReceived(quint8,CAR_STATE)),
            this, SLOT(stateReceived(quint8,CAR_STATE)));
}

void NetworkInterface::sendState(quint8 car, const CAR_STATE &state)
{
    if (ui->noForwardStateBox->isChecked()) {
        return;
    }

    QByteArray data;
    QXmlStreamWriter stream(&data);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("getState");
    stream.writeTextElement("car_id", QString::number(car));

    stream.writeTextElement("fw_major", QString::number(state.fw_major));
    stream.writeTextElement("fw_minor", QString::number(state.fw_minor));
    stream.writeTextElement("roll", QString::number(state.roll));
    stream.writeTextElement("pitch", QString::number(state.pitch));
    stream.writeTextElement("yaw", QString::number(state.yaw));
    stream.writeTextElement("accel_0", QString::number(state.accel[0]));
    stream.writeTextElement("accel_1", QString::number(state.accel[1]));
    stream.writeTextElement("accel_2", QString::number(state.accel[2]));
    stream.writeTextElement("gyro_0", QString::number(state.gyro[0]));
    stream.writeTextElement("gyro_1", QString::number(state.gyro[1]));
    stream.writeTextElement("gyro_2", QString::number(state.gyro[2]));
    stream.writeTextElement("mag_0", QString::number(state.mag[0]));
    stream.writeTextElement("mag_1", QString::number(state.mag[1]));
    stream.writeTextElement("mag_2", QString::number(state.mag[2]));
    stream.writeTextElement("px", QString::number(state.px));
    stream.writeTextElement("py", QString::number(state.py));
    stream.writeTextElement("speed", QString::number(state.speed));
    stream.writeTextElement("vin", QString::number(state.vin));
    stream.writeTextElement("temp_fet", QString::number(state.temp_fet));
    stream.writeTextElement("mc_fault", QString::number((int)state.mc_fault));
    stream.writeTextElement("px_gps", QString::number(state.px_gps));
    stream.writeTextElement("py_gps", QString::number(state.py_gps));
    stream.writeTextElement("ap_goal_px", QString::number(state.ap_goal_px));
    stream.writeTextElement("ap_goal_py", QString::number(state.ap_goal_py));
    stream.writeTextElement("ap_rad", QString::number(state.ap_rad));
    stream.writeTextElement("ms_today", QString::number(state.ms_today));

    stream.writeEndDocument();
    sendData(data);
}

void NetworkInterface::sendError(const QString &txt)
{
    QByteArray data;
    QXmlStreamWriter stream(&data);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("error");
    stream.writeTextElement("description", txt);

    stream.writeEndDocument();
    sendData(data);
}

void NetworkInterface::tcpDataRx(const QByteArray &data)
{
    processData(data);
}

void NetworkInterface::tcpConnectionChanged(bool connected)
{
    QString style_red = "color: rgb(255, 255, 255);"
                        "background-color: rgb(150, 0, 0);";

    QString style_green = "color: rgb(255, 255, 255);"
                          "background-color: rgb(0, 150, 0);";

    if (connected) {
        ui->tcpClientConnectedLabel->setStyleSheet(QString("#tcpClientConnectedLabel {%1}").arg(style_green));
        ui->tcpClientConnectedLabel->setText(tr("Client Connected"));
    } else {
        ui->tcpClientConnectedLabel->setStyleSheet(QString("#tcpClientConnectedLabel {%1}").arg(style_red));
        ui->tcpClientConnectedLabel->setText(tr("Client Not Connected"));
    }
}

void NetworkInterface::udpReadReady()
{
    while (mUdpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(mUdpSocket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        mUdpSocket->readDatagram(datagram.data(), datagram.size(),
                                 &sender, &senderPort);
        mLastHostAddress = sender;

        // Process data
        processData(datagram);
    }
}

void NetworkInterface::stateReceived(quint8 id, CAR_STATE state)
{
    sendState(id, state);
}

void NetworkInterface::on_tcpActivateBox_toggled(bool checked)
{
    ui->tcpPortBox->setEnabled(false);

    if (checked) {
        if (!mTcpServer->startServer(ui->tcpPortBox->value())) {
            qWarning() << "Starting TCP server failed:" << mTcpServer->errorString();
            QMessageBox::warning(this, "TCP Server Error",
                                 tr("Starting TCP server failed. Make sure that the port is not "
                                 "already in use. Error: %1").arg(mTcpServer->errorString()));
            ui->tcpActivateBox->setChecked(false);
        }
    } else {
        mTcpServer->stopServer();
    }

    ui->tcpPortBox->setEnabled(!ui->tcpActivateBox->isChecked());
}

void NetworkInterface::on_udpActivateBox_toggled(bool checked)
{
    if (checked) {
        if (!mUdpSocket->bind(QHostAddress::Any, ui->udpPortBox->value())) {
            qWarning() << "Binding UDP socket failed.";
            QMessageBox::warning(this, "UDP Server Error",
                                 "Creating UDP server failed. Make sure that the port is not "
                                 "already in use.");
            ui->udpActivateBox->setChecked(false);
        }
    } else {
        mUdpSocket->close();
    }

    ui->udpPortBox->setEnabled(!ui->udpActivateBox->isChecked());
}

void NetworkInterface::processData(const QByteArray &data)
{
    mRxBuffer.append(data);

    int start = mRxBuffer.indexOf("<message>");
    int end = mRxBuffer.indexOf("</message>");

    if (start >= 0 && end >= 0) {
        QByteArray xml = mRxBuffer.mid(start, end - start + 10);
        mRxBuffer.remove(start, end - start + 10);
        processXml(xml);
    }

    // Clear buffer if it becomes too long
    if (mRxBuffer.size() > 5e6) {
        mRxBuffer.clear();
    }

    // Clear buffer if no message part is present
    if (mRxBuffer.indexOf("<message>") < 0) {
        mRxBuffer.clear();
    }
}

void NetworkInterface::processXml(const QByteArray &xml)
{
    QXmlStreamReader stream(xml);
    stream.readNextStartElement();

    while (stream.readNextStartElement()) {
        if (stream.hasError()) {
            break;
        }

        QString name = stream.name().toString();

        if (name == "getState") {
            quint8 id = 0;
            bool ok = true;

            while (stream.readNextStartElement()) {
                QString name2 = stream.name().toString();

                if (name2 == "id") {
                    id = stream.readElementText().toDouble();
                } else {
                    QString str;
                    str += name + ": argument not found: " + name2;
                    sendError(str);
                    qWarning() << str;
                    stream.skipCurrentElement();
                    ok = false;
                }
            }

            if (stream.hasError()) {
                break;
            }

            if (!ok) {
                continue;
            }

            if (!ui->disableSendCarBox->isChecked() && mPacketInterface) {
                mPacketInterface->getState(id);
            }
        } else if (name == "addRoutePoint") {
            quint8 id = 0;
            double px = 0.0;
            double py = 0.0;
            double speed = 0.0;
            int time = 0;
            bool ok = true;

            while (stream.readNextStartElement()) {
                QString name2 = stream.name().toString();

                if (name2 == "id") {
                    id = stream.readElementText().toDouble();
                } else if (name2 == "px") {
                    px = stream.readElementText().toDouble();
                } else if (name2 == "py") {
                    py = stream.readElementText().toDouble();
                } else if (name2 == "speed") {
                    speed = stream.readElementText().toDouble();
                } else if (name2 == "time") {
                    time = stream.readElementText().toInt();
                } else {
                    QString str;
                    str += name + ": argument not found: " + name2;
                    sendError(str);
                    qWarning() << str;
                    stream.skipCurrentElement();
                    ok = false;
                }
            }

            if (stream.hasError()) {
                break;
            }

            if (!ok) {
                continue;
            }

            if (!ui->disableSendCarBox->isChecked() && mPacketInterface) {
                LocPoint p;
                p.setXY(px, py);
                p.setSpeed(speed);
                p.setTime(time);
                QList<LocPoint> route;
                route.append(p);
                bool res = mPacketInterface->setRoutePoints(id, route);

                if (!res) {
                    sendError("Sending route point to car failed. Make sure that the car connection "
                              "works.");
                }
            }

            if (mMap && ui->plotRouteMapBox->isChecked()) {
                mMap->addRoutePoint(px, py, speed, time);
            }
        } else {
            QString str;
            str += "Command not found: " + name;
            sendError(str);
            qWarning() << str;
            stream.skipCurrentElement();
        }
    }

    if (stream.hasError()) {
        QString str;
        str += "XML Parse error: " + stream.errorString();
        sendError(str);
        qWarning() << str;
    }
}

void NetworkInterface::sendData(const QByteArray &data)
{
    if (ui->tcpActivateBox->isChecked()) {
        mTcpServer->sendData(data);
    }

    if (ui->udpActivateBox->isChecked()) {
        if (QString::compare(mLastHostAddress.toString(), "0.0.0.0") != 0) {
            mUdpSocket->writeDatagram(data, mLastHostAddress, ui->udpPortBox->value() + 1);
        }
    }
}
