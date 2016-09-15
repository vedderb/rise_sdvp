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
#include <QSerialPortInfo>
#include <QDebug>
#include <cmath>
#include <QMessageBox>
#include <QFileDialog>

#include "nmeaserver.h"
#include "utility.h"

namespace {
void stepTowards(double &value, double goal, double step) {
    if (value < goal) {
        if ((value + step) < goal) {
            value += step;
        } else {
            value = goal;
        }
    } else if (value > goal) {
        if ((value - step) > goal) {
            value -= step;
        } else {
            value = goal;
        }
    }
}

void deadband(double &value, double tres, double max) {
    if (fabs(value) < tres) {
        value = 0.0;
    } else {
        double k = max / (max - tres);
        if (value > 0.0) {
            value = k * value + max * (1.0 - k);
        } else {
            value = -(k * -value + max * (1.0 - k));
        }

    }
}
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    qRegisterMetaType<LocPoint>("LocPoint");

    mTimer = new QTimer(this);
    mTimer->start(40);
    mStatusLabel = new QLabel(this);
    ui->statusBar->addPermanentWidget(mStatusLabel);
    mStatusInfoTime = 0;
    mPacketInterface = new PacketInterface(this);
    mSerialPort = new QSerialPort(this);

#ifdef HAS_JOYSTICK
    mJoystick = new Joystick(this);
    mJsType = JS_TYPE_HK;
#endif

    mPing = new Ping(this);

    mKeyUp = false;
    mKeyDown = false;
    mKeyLeft = false;
    mKeyRight = false;

    ui->mapWidget->setRoutePointSpeed(ui->mapRouteSpeedBox->value() / 3.6);
    ui->networkLoggerWidget->setMap(ui->mapWidget);

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    connect(mSerialPort, SIGNAL(readyRead()),
            this, SLOT(serialDataAvailable()));
    connect(mSerialPort, SIGNAL(error(QSerialPort::SerialPortError)),
            this, SLOT(serialPortError(QSerialPort::SerialPortError)));
    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    connect(mPacketInterface, SIGNAL(dataToSend(QByteArray&)),
            this, SLOT(packetDataToSend(QByteArray&)));
    connect(mPacketInterface, SIGNAL(stateReceived(quint8,CAR_STATE)),
            this, SLOT(stateReceived(quint8,CAR_STATE)));
    connect(ui->mapWidget, SIGNAL(posSet(quint8,LocPoint)),
            this, SLOT(mapPosSet(quint8,LocPoint)));
    connect(mPacketInterface, SIGNAL(ackReceived(quint8,CMD_PACKET,QString)),
            this, SLOT(ackReceived(quint8,CMD_PACKET,QString)));
    connect(ui->rtcmWidget, SIGNAL(rtcmReceived(QByteArray,int)),
            this, SLOT(rtcmReceived(QByteArray,int)));
    connect(ui->rtcmWidget, SIGNAL(refPosGet()), this, SLOT(rtcmRefPosGet()));
    connect(mPing, SIGNAL(pingRx(int,QString)), this, SLOT(pingRx(int,QString)));
    connect(mPing, SIGNAL(pingError(QString,QString)), this, SLOT(pingError(QString,QString)));
    connect(mPacketInterface, SIGNAL(enuRefReceived(quint8,double,double,double)),
            this, SLOT(enuRx(quint8,double,double,double)));

    on_serialRefreshButton_clicked();

    qApp->installEventFilter(this);
}

MainWindow::~MainWindow()
{
    // Remove all cars before this window is destroyed to not get segfaults
    // in their destructors.
    while (mCars.size() > 0) {
        CarInterface *car = (CarInterface*)ui->carsWidget->currentWidget();
        ui->carsWidget->removeTab(ui->carsWidget->currentIndex());
        mCars.removeOne(car);
        delete car;
    }

    delete ui;
}

bool MainWindow::eventFilter(QObject *object, QEvent *e)
{
    Q_UNUSED(object);

    // Emergency stop on escape
    if (e->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(e);
        if (keyEvent->key() == Qt::Key_Escape) {
            on_stopButton_clicked();
            return true;
        }
    }

#ifdef HAS_JOYSTICK
    if (mJoystick->isConnected()) {
        return false;
    }
#endif

    if (e->type() == QEvent::KeyPress || e->type() == QEvent::KeyRelease) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(e);
        bool isPress = e->type() == QEvent::KeyPress;

        switch(keyEvent->key()) {
        case Qt::Key_Up:
        case Qt::Key_Down:
        case Qt::Key_Left:
        case Qt::Key_Right:
            break;

        default:
            return false;
        }

        switch(keyEvent->key()) {
        case Qt::Key_Up: mKeyUp = isPress; break;
        case Qt::Key_Down: mKeyDown = isPress; break;
        case Qt::Key_Left: mKeyLeft = isPress; break;
        case Qt::Key_Right: mKeyRight = isPress; break;

        default:
            break;
        }

        // Return true to not pass the key event on
        return true;
    }

    return false;
}

void MainWindow::serialDataAvailable()
{
    while (mSerialPort->bytesAvailable() > 0) {
        QByteArray data = mSerialPort->readAll();
        mPacketInterface->processData(data);
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
    bool js_connected = false;

#ifdef HAS_JOYSTICK
    js_connected = mJoystick->isConnected();
#endif

    // Update throttle and steering from keys.
    if (js_connected) {
#ifdef HAS_JOYSTICK
        if (mJsType == JS_TYPE_HK) {
            mThrottle = -(double)mJoystick->getAxis(4) / 32768.0;
            deadband(mThrottle,0.1, 1.0);
            mSteering = -(double)mJoystick->getAxis(0) / 32768.0;
        } else if (mJsType == JS_TYPE_PS4) {
            mThrottle = -(double)mJoystick->getAxis(1) / 32768.0;
            deadband(mThrottle,0.1, 1.0);
            mSteering = (double)mJoystick->getAxis(2) / 32768.0;
        }

        //mSteering /= 2.0;
#endif
    } else {
        if (mKeyUp) {
            stepTowards(mThrottle, 1.0, ui->throttleGainBox->value());
        } else if (mKeyDown) {
            stepTowards(mThrottle, -1.0, ui->throttleGainBox->value());
        } else {
            stepTowards(mThrottle, 0.0, ui->throttleGainBox->value());
        }

        if (mKeyRight) {
            stepTowards(mSteering, 1.0, ui->steeringGainBox->value());
        } else if (mKeyLeft) {
            stepTowards(mSteering, -1.0, ui->steeringGainBox->value());
        } else {
            stepTowards(mSteering, 0.0, ui->steeringGainBox->value());
        }
    }

    ui->throttleBar->setValue(mThrottle * 100.0);
    ui->steeringBar->setValue(mSteering * 100.0);

    // Notify about key events
    for(QList<CarInterface*>::Iterator it_car = mCars.begin();it_car < mCars.end();it_car++) {
        CarInterface *car = *it_car;
        car->setControlValues(mThrottle, mSteering, ui->throttleMaxBox->value(), ui->throttleCurrentButton->isChecked());
    }

    // Update status label
    if (mStatusInfoTime) {
        mStatusInfoTime--;
        if (!mStatusInfoTime) {
            mStatusLabel->setStyleSheet(qApp->styleSheet());
        }
    } else {
        if (mSerialPort->isOpen() || mPacketInterface->isUdpConnected()) {
            mStatusLabel->setText("Connected");
        } else {
            mStatusLabel->setText("Not connected");
        }
    }

    // Poll data (one car per timeslot)
    static int next_car = 0;
    int ind = 0;
    int largest = 0;
    bool polled = false;
    for(QList<CarInterface*>::Iterator it_car = mCars.begin();it_car < mCars.end();it_car++) {
        CarInterface *car = *it_car;
        if (car->pollData() && ind >= next_car && !polled) {
            mPacketInterface->getState(car->getId());
            next_car = ind + 1;
            polled = true;
        }

        if (car->pollData() && ind > largest) {
            largest = ind;
        }

        ind++;
    }
    if (next_car > largest) {
        next_car = 0;
    }

    // Update map settings
    if (ui->mapFollowBox->isChecked()) {
        ui->mapWidget->setFollowCar(ui->mapCarBox->value());
    } else {
        ui->mapWidget->setFollowCar(-1);
    }
    if (ui->mapTraceBox->isChecked()) {
        ui->mapWidget->setTraceCar(ui->mapCarBox->value());
    } else {
        ui->mapWidget->setTraceCar(-1);
    }
    ui->mapWidget->setSelectedCar(ui->mapCarBox->value());

    // Joystick connected
#ifdef HAS_JOYSTICK
    static bool jsWasconn = false;
    if (mJoystick->isConnected() != jsWasconn) {
        jsWasconn = mJoystick->isConnected();

        if (jsWasconn) {
            ui->jsConnectedLabel->setText("Connected");
        } else {
            ui->jsConnectedLabel->setText("Not connected");
        }
    }
#endif
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

void MainWindow::packetDataToSend(QByteArray &data)
{
    if (mSerialPort->isOpen()) {
        mSerialPort->write(data);
    }
}

void MainWindow::stateReceived(quint8 id, CAR_STATE state)
{
    for(QList<CarInterface*>::Iterator it_car = mCars.begin();it_car < mCars.end();it_car++) {
        CarInterface *car = *it_car;
        if (car->getId() == id) {
            car->setStateData(state);
        }
    }
}

void MainWindow::mapPosSet(quint8 id, LocPoint pos)
{
    mPacketInterface->setPos(id, pos.getX(), pos.getY(), pos.getAlpha() * 180.0 / M_PI);
}

void MainWindow::ackReceived(quint8 id, CMD_PACKET cmd, QString msg)
{
    (void)cmd;
    QString str;
    str.sprintf("Car %d ack: ", id);
    str += msg;
    showStatusInfo(str, true);
}

void MainWindow::rtcmReceived(QByteArray data, int type)
{
    // Only send GPS observations and base station position
    // to save bandwidth. This requires that sync is 0 for
    // RTKLIB. This is done by rtcmclient for now. Also, rtcmclient
    // will re-encode 1004 to 1002 to save some bandwidth.
    switch (type) {
    case 1001:
    case 1002:
    case 1003:
    case 1004:
    case 1005:
    case 1006:
    case 1019:
        mPacketInterface->sendRtcmUsb(255, data);
        break;
    default:
        break;
    }
}

void MainWindow::rtcmRefPosGet()
{
    double lat, lon, height;
    if (ui->baseStationWidget->getAvgPosLlh(lat, lon, height) > 0) {
        ui->rtcmWidget->setRefPos(lat, lon, height);
    } else {
        QMessageBox::warning(this, "Reference Position",
                             "No samples collected yet.");
    }
}

void MainWindow::pingRx(int time, QString msg)
{
    QString str;
    str.sprintf("ping response time: %.3f ms", (double)time / 1000.0);
    QMessageBox::information(this, "Ping " + msg, str);
}

void MainWindow::pingError(QString msg, QString error)
{
    QMessageBox::warning(this, "Error ping " + msg, error);
}

void MainWindow::enuRx(quint8 id, double lat, double lon, double height)
{
    (void)id;
    ui->mapWidget->setEnuRef(lat, lon, height);
}

void MainWindow::on_carAddButton_clicked()
{
    CarInterface *car = new CarInterface(this);
    int id = mCars.size();
    mCars.append(car);
    QString name;
    name.sprintf("Car %d", id);
    car->setID(id);
    ui->carsWidget->addTab(car, name);
    car->setMap(ui->mapWidget);
    car->setPacketInterface(mPacketInterface);

    connect(car, SIGNAL(showStatusInfo(QString,bool)), this, SLOT(showStatusInfo(QString,bool)));
}

void MainWindow::on_carRemoveButton_clicked()
{
    if (mCars.size() > 0) {
        CarInterface *car = (CarInterface*)ui->carsWidget->currentWidget();
        ui->carsWidget->removeTab(ui->carsWidget->currentIndex());
        mCars.removeOne(car);
        delete car;
    }
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

    mPacketInterface->stopUdpConnection();
}

void MainWindow::on_serialRefreshButton_clicked()
{
    ui->serialPortBox->clear();
    bool found = false;

    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    foreach(const QSerialPortInfo &port, ports) {
        QString name = port.portName();
        int index = ui->serialPortBox->count();
        // put STMicroelectronics device first in list and add prefix
        if(port.manufacturer() == "STMicroelectronics") {
            name.insert(0, "IF - ");
            index = 0;
            found = true;
        }
        ui->serialPortBox->insertItem(index, name, port.systemLocation());
    }

    ui->serialPortBox->setCurrentIndex(0);

    if (found && !mSerialPort->isOpen()) {
        on_serialConnectButton_clicked();
    }
}

void MainWindow::on_disconnectButton_clicked()
{
    if (mSerialPort->isOpen()) {
        mSerialPort->close();
    }

    if (mPacketInterface->isUdpConnected()) {
        mPacketInterface->stopUdpConnection();
    }
}

void MainWindow::on_mapRemoveTraceButton_clicked()
{
    ui->mapWidget->clearTrace();
}

void MainWindow::on_MapRemovePixmapsButton_clicked()
{
    ui->mapWidget->clearPerspectivePixmaps();
}

void MainWindow::on_udpConnectButton_clicked()
{
    QHostAddress ip;

    if (ip.setAddress(ui->udpIpEdit->text().trimmed())) {
        if (mSerialPort->isOpen()) {
            mSerialPort->close();
        }

        mPacketInterface->startUdpConnection(ip, ui->udpPortBox->value());
    } else {
        showStatusInfo("Invalid IP address", false);
    }
}

void MainWindow::on_mapZeroButton_clicked()
{
    ui->mapWidget->setXOffset(0);
    ui->mapWidget->setYOffset(0);
}

void MainWindow::on_mapRemoveRouteButton_clicked()
{
    ui->mapWidget->clearRoute();
}

void MainWindow::on_mapRouteSpeedBox_valueChanged(double arg1)
{
    ui->mapWidget->setRoutePointSpeed(arg1 / 3.6);
}

void MainWindow::on_jsConnectButton_clicked()
{
#ifdef HAS_JOYSTICK
    if (mJoystick->init(ui->jsPortEdit->text()) == 0) {
        qDebug() << "Axes:" << mJoystick->numAxes();
        qDebug() << "Buttons:" << mJoystick->numButtons();
        qDebug() << "Name:" << mJoystick->getName();

        if (mJoystick->getName().contains("sony", Qt::CaseInsensitive)) {
            mJsType = JS_TYPE_PS4;
            qDebug() << "Treating joystick as PS4 USB controller.";
            showStatusInfo("PS4 USB joystick connected!", true);
        } else {
            mJsType = JS_TYPE_HK;
            qDebug() << "Treating joystick as hobbyking simulator.";
            showStatusInfo("HK joystick connected!", true);
        }
    } else {
        qWarning() << "Opening joystick failed.";
        showStatusInfo("Opening joystick failed.", false);
    }
#else
    QMessageBox::warning(this, "Joystick",
                         "This build does not have joystick support.");
#endif
}

void MainWindow::on_jsDisconnectButton_clicked()
{
#ifdef HAS_JOYSTICK
    mJoystick->stop();
#endif
}

void MainWindow::on_mapAntialiasBox_toggled(bool checked)
{
    ui->mapWidget->setAntialiasDrawings(checked);
}

void MainWindow::on_carsWidget_tabCloseRequested(int index)
{
    if (mCars.size() > 0) {
        CarInterface *car = (CarInterface*)ui->carsWidget->widget(index);
        ui->carsWidget->removeTab(index);
        mCars.removeOne(car);
        delete car;
    }
}

void MainWindow::on_genCircButton_clicked()
{
    double rad = ui->genCircRadBox->value();
    double speed = ui->mapRouteSpeedBox->value() / 3.6;
    double ang_ofs = M_PI;
    double cx = 0;
    double cy = rad;
    int points = ui->genCircPointsBox->value();

    CarInfo *car = ui->mapWidget->getCarInfo(ui->mapCarBox->value());
    if (car) {
        LocPoint p = car->getLocation();
        double ang = p.getAlpha();

        cx = p.getX();
        cy = p.getY();

        cx += rad * sin(ang);
        cy += rad * cos(ang);
        ang_ofs = ang + M_PI;
    }

    for (int i = 1;i <= points;i++) {
        int ind = i;

        if (rad < 0.0) {
            ind = points - i;
        }

        double ang = -((double)ind * 2.0 * M_PI) / (double)points + ang_ofs;

        double px = sin(ang) * rad;
        double py = cos(ang) * rad;

        // Move up
        px += cx;
        py += cy;

        ui->mapWidget->addRoutePoint(px, py, speed);

        bool res = true;
        LocPoint pos;
        pos.setXY(px, py);
        pos.setSpeed(speed);

        QList<LocPoint> points;
        points.append(pos);

        for (int i = 0;i < mCars.size();i++) {
            if (mCars[i]->updateRouteFromMap()) {
                res = mPacketInterface->setRoutePoints(mCars[i]->getId(), points);
            }
        }

        if (!res) {
            QMessageBox::warning(this, "Generate Cirlce",
                                 "No ack from car when uploading point.");
            break;
        }
    }
}

void MainWindow::on_mapSetAbsYawButton_clicked()
{
    CarInfo *car = ui->mapWidget->getCarInfo(ui->mapCarBox->value());
    if (car) {
        if (mSerialPort->isOpen() || mPacketInterface->isUdpConnected()) {
            ui->mapSetAbsYawButton->setEnabled(false);
            ui->mapAbsYawSlider->setEnabled(false);
            bool ok = mPacketInterface->setYawOffsetAck(car->getId(), (double)ui->mapAbsYawSlider->value());
            ui->mapSetAbsYawButton->setEnabled(true);
            ui->mapAbsYawSlider->setEnabled(true);

            if (!ok) {
                qDebug() << "No pos ack received";
            }
        }
    }
}

void MainWindow::on_mapAbsYawSlider_valueChanged(int value)
{
    (void)value;
    CarInfo *car = ui->mapWidget->getCarInfo(ui->mapCarBox->value());
    if (car) {
        mPacketInterface->setYawOffset(car->getId(), (double)ui->mapAbsYawSlider->value());
    }
}

void MainWindow::on_mapAbsYawSlider_sliderReleased()
{
    on_mapSetAbsYawButton_clicked();
}

void MainWindow::on_stopButton_clicked()
{
    for (int i = 0;i < mCars.size();i++) {
        mCars[i]->emergencyStop();
    }

    mPacketInterface->setRcControlCurrentBrake(255, 15.0, 0.0);
    mPacketInterface->setRcControlCurrentBrake(255, 15.0, 0.0);
    mPacketInterface->setRcControlCurrentBrake(255, 15.0, 0.0);
    mPacketInterface->setRcControlCurrentBrake(255, 15.0, 0.0);
}

void MainWindow::on_mapUploadRouteButton_clicked()
{
    if (!mSerialPort->isOpen() && !mPacketInterface->isUdpConnected()) {
        QMessageBox::warning(this, "Upload route",
                             "Serial port not connected.");
        return;
    }

    QList<LocPoint> route = ui->mapWidget->getRoute();
    int len = route.size();
    int car = ui->mapCarBox->value();
    bool ok = true;

    if (len <= 0) {
        QMessageBox::warning(this, "Upload route",
                             "No route on map.");
        return;
    }

    ui->mapUploadRouteButton->setEnabled(false);

    // Stop car
    for (int i = 0;i < mCars.size();i++) {
        if (mCars[i]->getId() == car) {
            ok = mCars[i]->setAp(false);
            break;
        }
    }

    // Clear previous route
    if (ok) {
        ok = mPacketInterface->clearRoute(car);
    }

    if (ok) {
        int ind = 0;
        for (ind = 0;ind < len;ind += 5) {
            QList<LocPoint> tmpList;
            for (int j = ind;j < (ind + 5);j++) {
                if (j < len) {
                    tmpList.append(route.at(j));
                }
            }

            ok = mPacketInterface->setRoutePoints(car, tmpList);

            if (!ok) {
                break;
            }

            ui->mapUploadRouteProgressBar->setValue((100 * (ind + 5)) / len);
        }
    }

    if (!ok) {
        QMessageBox::warning(this, "Upload route",
                             "No response when uploading route.");
    } else {
        ui->mapUploadRouteProgressBar->setValue(100);
    }

    ui->mapUploadRouteButton->setEnabled(true);
}

void MainWindow::on_mapApButton_clicked()
{
    for (int i = 0;i < mCars.size();i++) {
        if (mCars[i]->getId() == ui->mapCarBox->value()) {
            mCars[i]->setCtrlAp();
        }
    }
}

void MainWindow::on_mapKbButton_clicked()
{
    for (int i = 0;i < mCars.size();i++) {
        if (mCars[i]->getId() == ui->mapCarBox->value()) {
            mCars[i]->setCtrlKb();
        }
    }
}

void MainWindow::on_mapOffButton_clicked()
{
    for (int i = 0;i < mCars.size();i++) {
        if (mCars[i]->getId() == ui->mapCarBox->value()) {
            mCars[i]->emergencyStop();
        }
    }
}

void MainWindow::on_mapUpdateSpeedButton_clicked()
{
    QList<LocPoint> route = ui->mapWidget->getRoute();

    for (int i = 0;i < route.size();i++) {
        route[i].setSpeed(ui->mapRouteSpeedBox->value() / 3.6);
    }

    ui->mapWidget->setRoute(route);
}

void MainWindow::on_udpPingButton_clicked()
{
    mPing->pingHost(ui->udpIpEdit->text(), 64, "UDP Host");
}

void MainWindow::on_mapOpenStreetMapBox_toggled(bool checked)
{
    ui->mapWidget->setDrawOpenStreetmap(checked);
    ui->mapWidget->update();
}

void MainWindow::on_mapAntialiasOsmBox_toggled(bool checked)
{
    ui->mapWidget->setAntialiasOsm(checked);
}

void MainWindow::on_mapOsmResSlider_valueChanged(int value)
{
    ui->mapWidget->setOsmRes((double)value / 100.0);
}

void MainWindow::on_mapChooseNmeaButton_clicked()
{
    QString path;
    path = QFileDialog::getOpenFileName(this, tr("Choose log file to open"));
    if (path.isNull()) {
        return;
    }

    ui->mapImportNmeaEdit->setText(path);
}

void MainWindow::on_mapImportNmeaButton_clicked()
{
    QFile file;
    file.setFileName(ui->mapImportNmeaEdit->text());

    if (file.exists()) {
        bool ok = file.open(QIODevice::ReadOnly | QIODevice::Text);

        if (ok) {
            QTextStream in(&file);

            double i_llh[3];
            bool i_llh_set = false;

            while(!in.atEnd()) {
                QString line = in.readLine();

                NmeaServer::nmea_gga_info_t gga;
                int res = NmeaServer::decodeNmeaGGA(line.toLocal8Bit(), gga);

                if (res > 5) {
                    if (!i_llh_set) {
                        if (ui->mapImportNmeaZeroEnuBox->isChecked()) {
                            i_llh[0] = gga.lat;
                            i_llh[1] = gga.lon;
                            i_llh[2] = gga.height;
                            ui->mapWidget->setEnuRef(i_llh[0], i_llh[1], i_llh[2]);
                        } else {
                            ui->mapWidget->getEnuRef(i_llh);
                        }

                        i_llh_set = true;
                    }

                    double llh[3];
                    double xyz[3];

                    llh[0] = gga.lat;
                    llh[1] = gga.lon;
                    llh[2] = gga.height;
                    utility::llhToEnu(i_llh, llh, xyz);

                    LocPoint p;
                    p.setXY(xyz[0], xyz[1]);
                    QString info;

                    info.sprintf("Fix type: %d\n"
                                 "Height  : %.2f",
                                 gga.fix_type, gga.height);

                    p.setInfo(info);
                    ui->mapWidget->addInfoPoint(p);
                }
            }
        } else {
            QMessageBox::warning(this, "Open Error", "Could not open " + file.fileName());
        }

    } else {
        QMessageBox::warning(this, "Open Error", "Please select a valid log file");
    }
}

void MainWindow::on_mapRemoveInfoButton_clicked()
{
    ui->mapWidget->clearInfoTrace();
}

void MainWindow::on_traceInfoMinZoomBox_valueChanged(double arg1)
{
    ui->mapWidget->setInfoTraceTextZoom(arg1);
}

void MainWindow::on_removeRouteExtraButton_clicked()
{
    on_mapRemoveRouteButton_clicked();
}

void MainWindow::on_mapOsmClearCacheButton_clicked()
{
    ui->mapWidget->osmClient()->clearCache();
    ui->mapWidget->update();
}

void MainWindow::on_mapOsmServerOsmButton_toggled(bool checked)
{
    if (checked) {
        ui->mapWidget->osmClient()->setTileServerUrl("http://tile.openstreetmap.org");
    }
}

void MainWindow::on_mapOsmServerHiResButton_toggled(bool checked)
{
    if (checked) {
        ui->mapWidget->osmClient()->setTileServerUrl("http://c.osm.rrze.fau.de/osmhd"); // Also https
    }
}

void MainWindow::on_mapOsmServerVedderButton_toggled(bool checked)
{
    if (checked) {
        ui->mapWidget->osmClient()->setTileServerUrl("http://home.vedder.se/osm_tiles");
    }
}

void MainWindow::on_mapOsmServerVedderHdButton_toggled(bool checked)
{
    if (checked) {
        ui->mapWidget->osmClient()->setTileServerUrl("http://home.vedder.se/osm_tiles_hd");
    }
}

void MainWindow::on_mapOsmMaxZoomBox_valueChanged(int arg1)
{
    ui->mapWidget->setOsmMaxZoomLevel(arg1);
}

void MainWindow::on_mapDrawGridBox_toggled(bool checked)
{
    ui->mapWidget->setDrawGrid(checked);
}

void MainWindow::on_mapGetEnuButton_clicked()
{
    mPacketInterface->getEnuRef(ui->mapCarBox->value());
}

void MainWindow::on_mapSetEnuButton_clicked()
{
    double llh[3];
    ui->mapWidget->getEnuRef(llh);
    mPacketInterface->setEnuRef(ui->mapCarBox->value(), llh);
}

void MainWindow::on_mapOsmStatsBox_toggled(bool checked)
{
    ui->mapWidget->setDrawOsmStats(checked);
}

void MainWindow::on_removeTraceExtraButton_clicked()
{
    ui->mapWidget->clearTrace();
}

void MainWindow::on_mapEditHelpButton_clicked()
{
    QMessageBox::information(this, tr("Keyboard shortcuts"),
                             tr("<b>CTRL + Left click:</b> Move selected car<br>"
                                "<b>CTRL + Right click:</b> Update route point speed<br>"
                                "<b>Shift + Left click:</b> Add route point<br>"
                                "<b>Shift + Left drag:</b> Move route point<br>"
                                "<b>Shift + right click:</b> Delete route point<br>"
                                "<b>CTRL + SHIFT + Left click:</b> Zero map ENU coordinates<br>"));
}
