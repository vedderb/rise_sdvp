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

#include "networkinterface.h"
#include "ui_networkinterface.h"
#include <QMessageBox>
#include "utility.h"

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
    mPollTimer = new QTimer(this);
    mPollTimerCarId = -1;
    mPollTimer->setSingleShot(false);
    mCars = 0;

    connect(mTcpServer, SIGNAL(dataRx(QByteArray)),
            this, SLOT(tcpDataRx(QByteArray)));
    connect(mTcpServer, SIGNAL(connectionChanged(bool)),
            this, SLOT(tcpConnectionChanged(bool)));
    connect(mUdpSocket, SIGNAL(readyRead()),
            this, SLOT(udpReadReady()));
    connect(mPollTimer, SIGNAL(timeout()),
            this, SLOT(pollTimerSlot()));

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
    connect(mPacketInterface, SIGNAL(enuRefReceived(quint8,double,double,double)),
            this, SLOT(enuRefReceived(quint8,double,double,double)));
    connect(mPacketInterface, SIGNAL(printReceived(quint8,QString)),
            this, SLOT(printReceived(quint8,QString)));
}

void NetworkInterface::setCars(QList<CarInterface *> *cars)
{
    mCars = cars;
}

void NetworkInterface::sendState(quint8 id, const CAR_STATE &state)
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

    stream.writeTextElement("id", QString::number(id));
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
    stream.writeTextElement("ap_route_left", QString::number(state.ap_route_left));
    stream.writeTextElement("px_uwb", QString::number(state.px_uwb));
    stream.writeTextElement("py_uwb", QString::number(state.py_uwb));

    stream.writeEndDocument();
    sendData(data);
}

void NetworkInterface::sendEnuRef(quint8 id, double lat, double lon, double height)
{
    if (ui->noForwardStateBox->isChecked()) {
        return;
    }

    QByteArray data;
    QXmlStreamWriter stream(&data);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("getEnuRef");

    stream.writeTextElement("id", QString::number(id));
    stream.writeTextElement("lat", QString::number(lat, 'g', 10));
    stream.writeTextElement("lon", QString::number(lon, 'g', 10));
    stream.writeTextElement("height", QString::number(height));

    stream.writeEndDocument();
    sendData(data);
}

void NetworkInterface::sendPrint(quint8 id, QString str)
{
    if (ui->noForwardStateBox->isChecked()) {
        return;
    }

    QByteArray data;
    QXmlStreamWriter stream(&data);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("terminalCmd");

    stream.writeTextElement("id", QString::number(id));
    stream.writeTextElement("str", str);

    stream.writeEndDocument();
    sendData(data);
}

void NetworkInterface::sendRoute(quint8 id, QList<LocPoint> route)
{
    if (ui->noForwardStateBox->isChecked()) {
        return;
    }

    QByteArray data;
    QXmlStreamWriter stream(&data);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("getRoute");

    stream.writeTextElement("id", QString::number(id));

    for (LocPoint p: route) {
        stream.writeStartElement("point");
        stream.writeTextElement("px", QString::number(p.getX()));
        stream.writeTextElement("py", QString::number(p.getY()));
        stream.writeTextElement("speed", QString::number(p.getSpeed()));
        stream.writeTextElement("time", QString::number(p.getTime()));
        stream.writeEndElement();
    }

    stream.writeEndDocument();
    sendData(data);
}

void NetworkInterface::sendError(const QString &txt, const QString &cmd)
{
    QByteArray data;
    QXmlStreamWriter stream(&data);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("error");
    if (!cmd.isEmpty()) {
        stream.writeTextElement("command", cmd);
    }
    stream.writeTextElement("description", txt);

    stream.writeEndDocument();
    sendData(data);

    qWarning() << "NetworkIf:" << cmd << ":" << txt;
}

void NetworkInterface::sendAck(const QString &cmd)
{
    QByteArray data;
    QXmlStreamWriter stream(&data);
    stream.setAutoFormatting(true);

    stream.writeStartDocument();
    stream.writeStartElement("message");
    stream.writeStartElement("ack");
    stream.writeTextElement("command", cmd);

    stream.writeEndDocument();
    sendData(data);
}

void NetworkInterface::setTcpEnabled(bool enabled, int port)
{
    ui->tcpActivateBox->setChecked(enabled);
    ui->tcpPortBox->setEnabled(false);

    if (port >= 0) {
        ui->tcpPortBox->setValue(port);
    }

    if (enabled) {
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

void NetworkInterface::setUdpEnabled(bool enabled, int port)
{
    ui->udpActivateBox->setChecked(enabled);

    if (port >= 0) {
        ui->udpPortBox->setValue(port);
    }

    if (enabled) {
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

void NetworkInterface::pollTimerSlot()
{
    if (mPacketInterface && mPollTimerCarId >= 0) {
        mPacketInterface->getState(mPollTimerCarId);
    }
}

void NetworkInterface::stateReceived(quint8 id, CAR_STATE state)
{
    sendState(id, state);
}

void NetworkInterface::enuRefReceived(quint8 id, double lat, double lon, double height)
{
    sendEnuRef(id, lat, lon, height);
}

void NetworkInterface::printReceived(quint8 id, QString str)
{
    sendPrint(id, str);
}

void NetworkInterface::on_tcpActivateBox_clicked(bool checked)
{
    setTcpEnabled(checked);
}

void NetworkInterface::on_udpActivateBox_clicked(bool checked)
{
    setUdpEnabled(checked);
}

void NetworkInterface::processData(const QByteArray &data)
{
    mRxBuffer.append(data);

    int start = mRxBuffer.indexOf("<message>");
    int end = mRxBuffer.indexOf("</message>");

    while (start >= 0 && end >= 0) {
        QByteArray xml = mRxBuffer.mid(start, end - start + 10);
        mRxBuffer.remove(start, end - start + 10);
        processXml(xml);

        start = mRxBuffer.indexOf("<message>");
        end = mRxBuffer.indexOf("</message>");
    }

    // Clear buffer if it becomes too long
    if (mRxBuffer.size() > 5e6) {
        mRxBuffer.clear();
    }

    // Clear buffer if no message part is present
    if (mRxBuffer.size() > 9 && mRxBuffer.indexOf("<message>") < 0) {
        mRxBuffer.clear();
    }
}

void NetworkInterface::processXml(const QByteArray &xml)
{
    QXmlStreamReader stream(xml);
    stream.readNextStartElement();
    QString name;

    while (stream.readNextStartElement()) {
        if (stream.hasError()) {
            break;
        }

        name = stream.name().toString();

        if (name == "getState") {
            quint8 id = 0;
            bool ok = true;

            while (stream.readNextStartElement()) {
                QString name2 = stream.name().toString();

                if (name2 == "id") {
                    id = stream.readElementText().toInt();
                } else {
                    QString str;
                    str += "argument not found: " + name2;
                    sendError(str, name);
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
        } else if (name == "addRoutePoint" || name == "replaceRoute") {
            quint8 id = 0;
            bool ok = true;
            bool mapOnly = false;
            int mapRoute = -1;

            QList<LocPoint> route;
            LocPoint p;

            while (stream.readNextStartElement()) {
                QString name2 = stream.name().toString();

                if (name2 == "id") {
                    id = stream.readElementText().toInt();
                } else if (name2 == "px") {
                    p.setX(stream.readElementText().toDouble());
                } else if (name2 == "py") {
                    p.setY(stream.readElementText().toDouble());
                } else if (name2 == "speed") {
                    p.setSpeed(stream.readElementText().toDouble());
                } else if (name2 == "time") {
                    p.setTime(stream.readElementText().toInt());
                } else if (name2 == "mapOnly") {
                    mapOnly = stream.readElementText().toInt();
                } else if (name2 == "mapRoute") {
                    mapRoute = stream.readElementText().toInt();
                } else if (name2 == "point") {
                    while (stream.readNextStartElement()) {
                        QString name3 = stream.name().toString();

                        if (name3 == "px") {
                            p.setX(stream.readElementText().toDouble());
                        } else if (name3 == "py") {
                            p.setY(stream.readElementText().toDouble());
                        } else if (name3 == "speed") {
                            p.setSpeed(stream.readElementText().toDouble());
                        } else if (name3 == "time") {
                            p.setTime(stream.readElementText().toInt());
                        } else {
                            QString str;
                            str += "argument not found: " + name3;
                            sendError(str, name);
                            stream.skipCurrentElement();
                            ok = false;
                        }
                    }

                    route.append(p);
                } else {
                    QString str;
                    str += "argument not found: " + name2;
                    sendError(str, name);
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

            if (route.isEmpty()) {
                route.append(p);
            }

            if (!mapOnly && !ui->disableSendCarBox->isChecked() && mPacketInterface) {
                if (name == "addRoutePoint") {
                    if (!utility::uploadRouteHelper(mPacketInterface, id, route)) {
                        ok = false;
                        sendError("No ACK received from car. Make sure that the car connection "
                                  "works.", name);
                    }
                } else {
                    if (!utility::replaceRouteHelper(mPacketInterface, id, route)) {
                        ok = false;
                        sendError("No ACK received from car. Make sure that the car connection "
                                  "works.", name);
                    }
                }
            }

            if (mMap && (ui->plotRouteMapBox->isChecked() || mapOnly) && mapRoute != -2) {
                int mapRouteLast = mMap->getRouteNow();
                if (mapRoute >= 0) {
                    mMap->setRouteNow(mapRoute);
                }

                if (name == "replaceRoute") {
                    mMap->clearRoute();
                }

                for (LocPoint p: route) {
                    mMap->addRoutePoint(p.getX(), p.getY(), p.getSpeed(), p.getTime());
                }

                mMap->setRouteNow(mapRouteLast);
            }

            if (ok) {
                sendAck(name);
            }
        } else if (name == "getRoute") {
            quint8 id = 0;
            bool ok = true;
            int mapRoute = -1;

            while (stream.readNextStartElement()) {
                QString name2 = stream.name().toString();

                if (name2 == "id") {
                    id = stream.readElementText().toInt();
                } else if (name2 == "mapRoute") {
                    mapRoute = stream.readElementText().toInt();
                } else {
                    QString str;
                    str += "argument not found: " + name2;
                    sendError(str, name);
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
            if (mapRoute >= 0) {
                if (mMap) {
                    sendRoute(id, mMap->getRoute(mapRoute));
                }
            } else {
                if (!ui->disableSendCarBox->isChecked() && mPacketInterface) {
                    QList<LocPoint> route;
                    if (mPacketInterface->getRoute(id, route)) {
                        sendRoute(id, route);
                    } else {
                        ok = false;
                        sendError("Route not received from car. Make sure that the car connection "
                                  "works.", name);
                    }
                }
            }
        } else if (name == "removeLastPoint") {
            quint8 id = 0;
            bool ok = true;

            while (stream.readNextStartElement()) {
                QString name2 = stream.name().toString();

                if (name2 == "id") {
                    id = stream.readElementText().toInt();
                } else {
                    QString str;
                    str += "argument not found: " + name2;
                    sendError(str, name);
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
                if (!mPacketInterface->removeLastRoutePoint(id)) {
                    ok = false;
                    sendError("No ACK received from car. Make sure that the car connection "
                              "works.", name);
                }
            }

            if (ok) {
                sendAck("removeLastPoint");
            }
        } else if (name == "clearRoute") {
            int id = 0;
            bool ok = true;
            int mapRoute = -1;

            while (stream.readNextStartElement()) {
                QString name2 = stream.name().toString();

                if (name2 == "id") {
                    id = stream.readElementText().toInt();
                } else if (name2 == "mapRoute") {
                    mapRoute = stream.readElementText().toInt();
                } else {
                    QString str;
                    str += "argument not found: " + name2;
                    sendError(str, name);
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

            if (id >= 0 && !ui->disableSendCarBox->isChecked() && mPacketInterface) {
                if (!mPacketInterface->clearRoute(id)) {
                    ok = false;
                    sendError("No ACK received from car. Make sure that the car connection "
                              "works.", name);
                }
            }

            if (mapRoute >= 0 && mMap) {
                int lastRoute = mMap->getRouteNow();
                mMap->setRouteNow(mapRoute);
                mMap->clearRoute();
                mMap->setRouteNow(lastRoute);
            }

            if (ok) {
                sendAck("clearRoute");
            }
        } else if (name == "setAutopilotActive") {
            quint8 id = 0;
            bool enabled = false;
            bool ok = true;

            while (stream.readNextStartElement()) {
                QString name2 = stream.name().toString();

                if (name2 == "id") {
                    id = stream.readElementText().toInt();
                } else if (name2 == "enabled") {
                    enabled = stream.readElementText().toInt();
                } else {
                    QString str;
                    str += "argument not found: " + name2;
                    sendError(str, name);
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
                if (mCars && enabled) {
                    for (int i = 0;i < mCars->size();i++) {
                        if (mCars->at(i)->getId() == id) {
                            mCars->at(i)->disableKbBox();
                        }
                    }
                }

                if (!mPacketInterface->setApActive(id, enabled)) {
                    ok = false;
                    sendError("No ACK received from car. Make sure that the car connection "
                              "works.", name);
                }
            }

            if (ok) {
                sendAck("setAutopilotActive");
            }
        } else if (name == "getEnuRef") {
            quint8 id = 0;
            bool ok = true;
            bool fromMap = false;

            while (stream.readNextStartElement()) {
                QString name2 = stream.name().toString();

                if (name2 == "id") {
                    id = stream.readElementText().toInt();
                } else if (name2 == "fromMap") {
                    fromMap = stream.readElementText().toInt();
                } else {
                    QString str;
                    str += "argument not found: " + name2;
                    sendError(str, name);
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
            if (fromMap) {
                if (mMap) {
                    double llh[3];
                    mMap->getEnuRef(llh);
                    sendEnuRef(255, llh[0], llh[1], llh[2]);
                }
            } else {
                if (!ui->disableSendCarBox->isChecked() && mPacketInterface) {
                    mPacketInterface->getEnuRef(id);
                }
            }
        } else if (name == "setEnuRef") {
            quint8 id = 0;
            double lat = 0.0;
            double lon = 0.0;
            double height = 0.0;
            bool ok = true;

            while (stream.readNextStartElement()) {
                QString name2 = stream.name().toString();

                if (name2 == "id") {
                    id = stream.readElementText().toInt();
                } else if (name2 == "lat") {
                    lat = stream.readElementText().toDouble();
                } else if (name2 == "lon") {
                    lon = stream.readElementText().toDouble();
                } else if (name2 == "height") {
                    height = stream.readElementText().toDouble();
                } else {
                    QString str;
                    str += "argument not found: " + name2;
                    sendError(str, name);
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
                double llh[3];
                llh[0] = lat;
                llh[1] = lon;
                llh[2] = height;

                if (!mPacketInterface->setEnuRef(id, llh)) {
                    ok = false;
                    sendError("No ACK received from car. Make sure that the car connection "
                              "works.", name);
                }
            }

            if (ok) {
                sendAck("setEnuRef");
            }

            if (mMap && ui->plotRouteMapBox->isChecked()) {
                mMap->setEnuRef(lat, lon, height);
            }
        } else if (name == "rcControl") {
            quint8 id = 0;
            int mode = 0;
            double value = 0.0;
            double steering = 0.0;
            bool ok = true;

            while (stream.readNextStartElement()) {
                QString name2 = stream.name().toString();

                if (name2 == "id") {
                    id = stream.readElementText().toInt();
                } else if (name2 == "mode") {
                    mode = stream.readElementText().toInt();
                } else if (name2 == "value") {
                    value = stream.readElementText().toDouble();
                } else if (name2 == "steering") {
                    steering = stream.readElementText().toDouble();
                } else {
                    QString str;
                    str += "argument not found: " + name2;
                    sendError(str, name);
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
                switch (mode) {
                case RC_MODE_CURRENT:
                    mPacketInterface->setRcControlCurrent(id, value, steering);
                    break;

                case RC_MODE_DUTY:
                    mPacketInterface->setRcControlDuty(id, value / 100.0, steering);
                    break;

                case RC_MODE_PID:
                    mPacketInterface->setRcControlPid(id, value, steering);
                    break;

                case RC_MODE_CURRENT_BRAKE:
                    mPacketInterface->setRcControlCurrentBrake(id, value, steering);
                    break;

                default:
                    sendError("Invaild mode", name);
                    break;
                }
            }
        } else if (name == "terminalCmd") {
            quint8 id = 0;
            QString cmd;
            bool ok = true;

            while (stream.readNextStartElement()) {
                QString name2 = stream.name().toString();

                if (name2 == "id") {
                    id = stream.readElementText().toInt();
                } else if (name2 == "cmd") {
                    cmd = stream.readElementText();
                } else {
                    QString str;
                    str += "argument not found: " + name2;
                    sendError(str, name);
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
                mPacketInterface->sendTerminalCmd(id, cmd);
            }
        } else if (name == "setStatusPoll") {
            int id = -1;
            bool ok = true;
            int time = false;

            while (stream.readNextStartElement()) {
                QString name2 = stream.name().toString();

                if (name2 == "id") {
                    id = stream.readElementText().toInt();
                } else if (name2 == "interval") {
                    time = stream.readElementText().toInt();
                } else {
                    QString str;
                    str += "argument not found: " + name2;
                    sendError(str, name);
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

            if (time > 10 && id >= 0) {
                mPollTimerCarId = id;
                mPollTimer->start(time);
            } else {
                mPollTimerCarId = -1;
                mPollTimer->stop();
            }
        } else {
            QString str;
            str += "Command not found: " + name;
            sendError(str);
            stream.skipCurrentElement();
        }
    }

    if (stream.hasError()) {
        QString str;
        str += "XML Parse error: " + stream.errorString();
        sendError(str, name);
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
