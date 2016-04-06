#include "carinterface.h"
#include "ui_carinterface.h"
#include "carinfo.h"
#include "utility.h"
#include <QFileDialog>
#include <QMessageBox>
#include <cmath>

namespace {
void faultToStr(mc_fault_code fault, QString &str, bool &isOk)
{
    switch (fault) {
    case FAULT_CODE_NONE: str = "FAULT_CODE_NONE"; isOk = true; break;
    case FAULT_CODE_OVER_VOLTAGE: str = "FAULT_CODE_OVER_VOLTAGE"; isOk = false; break;
    case FAULT_CODE_UNDER_VOLTAGE: str = "FAULT_CODE_UNDER_VOLTAGE"; isOk = false; break;
    case FAULT_CODE_DRV8302: str = "FAULT_CODE_DRV8302"; isOk = false; break;
    case FAULT_CODE_ABS_OVER_CURRENT: str = "FAULT_CODE_ABS_OVER_CURRENT"; isOk = false; break;
    case FAULT_CODE_OVER_TEMP_FET: str = "FAULT_CODE_OVER_TEMP_FET"; isOk = false; break;
    case FAULT_CODE_OVER_TEMP_MOTOR: str = "FAULT_CODE_OVER_TEMP_MOTOR"; isOk = false; break;
    default: str = "Unknown fault"; isOk = false; break;
    }
}
}

CarInterface::CarInterface(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CarInterface)
{
    ui->setupUi(this);

    // Plots
    ui->accelPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->gyroPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->magPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    // The raw IMU plots
    maxSampleSize = 1000;
    accelXData.resize(maxSampleSize);
    accelYData.resize(maxSampleSize);
    accelZData.resize(maxSampleSize);
    gyroXData.resize(maxSampleSize);
    gyroYData.resize(maxSampleSize);
    gyroZData.resize(maxSampleSize);
    magXData.resize(maxSampleSize);
    magYData.resize(maxSampleSize);
    magZData.resize(maxSampleSize);
    accelGyroMagXAxis.resize(maxSampleSize);
    for(int i = 0;i < accelGyroMagXAxis.size();i++) {
        accelGyroMagXAxis[accelGyroMagXAxis.size() - i - 1] = (40.0 / 1000.0 * i);
    }

    mMap = 0;
    mPacketInterface = 0;
    mId = 0;

    mTimer = new QTimer(this);
    mTimer->start(20);

    mUdpSocket = new QUdpSocket(this);
    mLastHostAddress.clear();
    mUdpPort = 27800;

    mNmeaForwardServer = new TcpBroadcast(this);

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    connect(mUdpSocket, SIGNAL(readyRead()), this, SLOT(udpReadReady()));

    ui->accelPlot->clearGraphs();
    ui->accelPlot->addGraph();
    ui->accelPlot->xAxis->setRangeReversed(true);
    ui->accelPlot->graph()->setPen(QPen(Qt::black));
    ui->accelPlot->graph()->setData(accelGyroMagXAxis, accelXData);
    ui->accelPlot->graph()->setName(tr("X"));
    ui->accelPlot->addGraph();
    ui->accelPlot->graph()->setPen(QPen(Qt::green));
    ui->accelPlot->graph()->setData(accelGyroMagXAxis, accelYData);
    ui->accelPlot->graph()->setName(tr("Y"));
    ui->accelPlot->addGraph();
    ui->accelPlot->graph()->setPen(QPen(Qt::blue));
    ui->accelPlot->graph()->setData(accelGyroMagXAxis, accelZData);
    ui->accelPlot->graph()->setName(tr("Z"));
    ui->accelPlot->rescaleAxes();
    ui->accelPlot->xAxis->setLabel("Seconds");
    ui->accelPlot->yAxis->setLabel("G");
    ui->accelPlot->legend->setVisible(true);
    ui->accelPlot->replot();

    ui->gyroPlot->clearGraphs();
    ui->gyroPlot->addGraph();
    ui->gyroPlot->xAxis->setRangeReversed(true);
    ui->gyroPlot->graph()->setPen(QPen(Qt::black));
    ui->gyroPlot->graph()->setData(accelGyroMagXAxis, gyroXData);
    ui->gyroPlot->graph()->setName(tr("X"));
    ui->gyroPlot->addGraph();
    ui->gyroPlot->graph()->setPen(QPen(Qt::green));
    ui->gyroPlot->graph()->setData(accelGyroMagXAxis, gyroYData);
    ui->gyroPlot->graph()->setName(tr("Y"));
    ui->gyroPlot->addGraph();
    ui->gyroPlot->graph()->setPen(QPen(Qt::blue));
    ui->gyroPlot->graph()->setData(accelGyroMagXAxis, gyroZData);
    ui->gyroPlot->graph()->setName(tr("Z"));
    ui->gyroPlot->rescaleAxes();
    ui->gyroPlot->xAxis->setLabel("Seconds");
    ui->gyroPlot->yAxis->setLabel("deg/s");
    ui->gyroPlot->legend->setVisible(true);
    ui->gyroPlot->replot();

    ui->magPlot->clearGraphs();
    ui->magPlot->addGraph();
    ui->magPlot->xAxis->setRangeReversed(true);
    ui->magPlot->graph()->setPen(QPen(Qt::black));
    ui->magPlot->graph()->setData(accelGyroMagXAxis, magXData);
    ui->magPlot->graph()->setName(tr("X"));
    ui->magPlot->addGraph();
    ui->magPlot->graph()->setPen(QPen(Qt::green));
    ui->magPlot->graph()->setData(accelGyroMagXAxis, magYData);
    ui->magPlot->graph()->setName(tr("Y"));
    ui->magPlot->addGraph();
    ui->magPlot->graph()->setPen(QPen(Qt::blue));
    ui->magPlot->graph()->setData(accelGyroMagXAxis, magZData);
    ui->magPlot->graph()->setName(tr("Z"));
    ui->magPlot->rescaleAxes();
    ui->magPlot->xAxis->setLabel("Seconds");
    ui->magPlot->yAxis->setLabel("uT");
    ui->magPlot->legend->setVisible(true);
    ui->magPlot->replot();
}

CarInterface::~CarInterface()
{
    if (mMap) {
        mMap->removeCar(mId);
    }

    delete ui;
}

void CarInterface::setID(int id)
{
    ui->idBox->setValue(id);
}

int CarInterface::getId()
{
    return mId;
}

bool CarInterface::pollData()
{
    return ui->pollBox->isChecked();
}

void CarInterface::setOrientation(double roll, double pitch, double yaw)
{
    ui->rollBar->setValue(roll);
    ui->pitchBar->setValue(pitch);
    ui->yawBar->setValue(yaw);
    ui->orientationWidget->setRollPitchYaw(roll, pitch, yaw);
}

void CarInterface::setStateData(CAR_STATE data)
{
    accelXData.append(data.accel[0]);
    accelXData.remove(0, 1);
    accelYData.append(data.accel[1]);
    accelYData.remove(0, 1);
    accelZData.append(data.accel[2]);
    accelZData.remove(0, 1);

    ui->accelPlot->graph(0)->setData(accelGyroMagXAxis, accelXData);
    ui->accelPlot->graph(1)->setData(accelGyroMagXAxis, accelYData);
    ui->accelPlot->graph(2)->setData(accelGyroMagXAxis, accelZData);
    ui->accelPlot->rescaleAxes();
    ui->accelPlot->replot();

    gyroXData.append(data.gyro[0] * 180.0 / M_PI);
    gyroXData.remove(0, 1);
    gyroYData.append(data.gyro[1] * 180.0 / M_PI);
    gyroYData.remove(0, 1);
    gyroZData.append(data.gyro[2] * 180.0 / M_PI);
    gyroZData.remove(0, 1);

    ui->gyroPlot->graph(0)->setData(accelGyroMagXAxis, gyroXData);
    ui->gyroPlot->graph(1)->setData(accelGyroMagXAxis, gyroYData);
    ui->gyroPlot->graph(2)->setData(accelGyroMagXAxis, gyroZData);
    ui->gyroPlot->rescaleAxes();
    ui->gyroPlot->replot();

    magXData.append(data.mag[0]);
    magXData.remove(0, 1);
    magYData.append(data.mag[1]);
    magYData.remove(0, 1);
    magZData.append(data.mag[2]);
    magZData.remove(0, 1);

    ui->magPlot->graph(0)->setData(accelGyroMagXAxis, magXData);
    ui->magPlot->graph(1)->setData(accelGyroMagXAxis, magYData);
    ui->magPlot->graph(2)->setData(accelGyroMagXAxis, magZData);
    ui->magPlot->rescaleAxes();
    ui->magPlot->replot();

    // Firmware label
    QString fwStr;
    fwStr.sprintf("FW %d.%d", data.fw_major, data.fw_minor);
    ui->fwLabel->setText(fwStr);

    // Speed bar
    QString speedTxt;
    speedTxt.sprintf("Speed: %.2f km/h", data.speed * 3.6);
    ui->speedBar->setValue(fabs(data.speed) * 3.6);
    ui->speedBar->setFormat(speedTxt);

    // Temp FET bar
    ui->tempFetBar->setValue(data.temp_fet);

    // Battery bar
    double battp = utility::map(data.vin, 34.0, 42.0, 0.0, 100.0);
    if (battp < 0.0) {
        battp = 0.0;
    }
    QString battTxt;
    battTxt.sprintf("Battery: %.1f %% (%.2f V)", battp, data.vin);
    if (battp > 100.0) {
        battp = 100.0;
    }
    ui->batteryBar->setValue((int)battp);
    ui->batteryBar->setFormat(battTxt);

    // Orientation
    setOrientation(data.roll, data.pitch, data.yaw);
    //ui->orientationWidget->setQuanternions(data.q[0], data.q[1], data.q[2], data.q[3]);
    //ui->rollBar->setValue(data.roll);
    //ui->pitchBar->setValue(data.pitch);
    //ui->yawBar->setValue(data.yaw);

    // Fault label
    QString fault_str;
    bool isOk;
    faultToStr(data.mc_fault, fault_str, isOk);

    static QString fault_last = "Fault code...";
    if (fault_last != fault_str) {
        fault_last = fault_str;
        ui->mcFaultLabel->setText(fault_str);
        if (isOk) {
            ui->mcFaultLabel->setStyleSheet("QLabel { background-color : lightgreen; color : black; }");
        } else {
            ui->mcFaultLabel->setStyleSheet("QLabel { background-color : red; color : black; }");

            QString msg;
            msg.sprintf("Car %d: ", mId);
            emit showStatusInfo(msg + fault_str, false);
        }
    }

    if (mMap) {
        CarInfo *car = mMap->getCarInfo(mId);
        LocPoint loc = car->getLocation();
        loc.setAlpha(data.yaw * M_PI / 180.0);
        loc.setX(data.px);
        loc.setY(data.py);
        car->setLocation(loc);
        mMap->repaintAfterEvents();
    }

    QVector<double> magXYZ;
    magXYZ.append(data.mag[0]);
    magXYZ.append(data.mag[1]);
    magXYZ.append(data.mag[2]);

    if (ui->magSampleStoreBox->isChecked()) {
        mMagSamples.append(magXYZ);
    }
}

void CarInterface::setMap(MapWidget *map)
{
    mMap = map;
    CarInfo car(mId);
    mMap->addCar(car);

    connect(mMap, SIGNAL(routePointAdded(LocPoint)),
            this, SLOT(routePointSet(LocPoint)));
    connect(mMap, SIGNAL(lastRoutePointRemoved(LocPoint)),
            this, SLOT(lastRoutePointRemoved()));
}

void CarInterface::setPacketInterface(PacketInterface *packetInterface)
{
    mPacketInterface = packetInterface;

    connect(this, SIGNAL(terminalCmd(quint8,QString)),
            mPacketInterface, SLOT(sendTerminalCmd(quint8,QString)));
    connect(mPacketInterface, SIGNAL(printReceived(quint8,QString)),
            this, SLOT(terminalPrint(quint8,QString)));
    connect(this, SIGNAL(forwardVesc(quint8,QByteArray)),
            mPacketInterface, SLOT(forwardVesc(quint8,QByteArray)));
    connect(mPacketInterface, SIGNAL(vescFwdReceived(quint8,QByteArray)),
            this, SLOT(vescFwdReceived(quint8,QByteArray)));
    connect(this, SIGNAL(setRcCurrent(quint8,double,double)),
            mPacketInterface, SLOT(setRcControlCurrent(quint8,double,double)));
    connect(this, SIGNAL(setRcDuty(quint8,double,double)),
            mPacketInterface, SLOT(setRcControlDuty(quint8,double,double)));
    connect(this, SIGNAL(setServoDirect(quint8,double)),
            mPacketInterface, SLOT(setServoDirect(quint8,double)));
    connect(mPacketInterface, SIGNAL(nmeaRadioReceived(quint8,QByteArray)),
            this, SLOT(nmeaReceived(quint8,QByteArray)));
    connect(mPacketInterface, SIGNAL(configurationReceived(quint8,MAIN_CONFIG)),
            this, SLOT(configurationReceived(quint8,MAIN_CONFIG)));
}

void CarInterface::setControlValues(double throttle, double steering, double max, bool currentMode)
{
    if (ui->keyboardControlBox->isChecked()) {
        if (fabs(throttle) < 0.005) {
            emit setRcCurrent(mId, 0.0, steering);
        } else {
            if (currentMode) {
                emit setRcCurrent(mId, throttle * 80.0 * max, steering);
            } else {
                emit setRcDuty(mId, throttle * max, steering);
            }
        }
    }
}

void CarInterface::timerSlot()
{
    // Update mag sample label
    static int lastMagSamples = 0;
    if (mMagSamples.size() != lastMagSamples) {
        ui->magSampleLabel->setText(QString::number(mMagSamples.size()) + " Samples");
        lastMagSamples = mMagSamples.size();
    }
}

void CarInterface::udpReadReady()
{
    while (mUdpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(mUdpSocket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        mUdpSocket->readDatagram(datagram.data(), datagram.size(),
                                 &sender, &senderPort);
        mLastHostAddress = sender;

        emit forwardVesc(mId, datagram);
    }
}

void CarInterface::terminalPrint(quint8 id, QString str)
{
    if (id == mId) {
        ui->terminalBrowser->append(str);
    }
}

void CarInterface::vescFwdReceived(quint8 id, QByteArray data)
{
    if (id == mId && QString::compare(mLastHostAddress.toString(), "0.0.0.0") != 0) {
        mUdpSocket->writeDatagram(data, mLastHostAddress, mUdpPort + 1);
    }
}

void CarInterface::routePointSet(LocPoint pos)
{
    if (mMap && mPacketInterface && ui->updateRouteFromMapBox->isChecked()) {
        QList<LocPoint> points;
        points.append(pos);

        mMap->setEnabled(false);
        bool ok = mPacketInterface->setRoutePoints(mId, points);
        mMap->setEnabled(true);

        if (!ok) {
            QMessageBox::warning(this, "Autopilot",
                                 "No ack received, so the the last route point was most likely not set.");
        }
    }
}

void CarInterface::lastRoutePointRemoved()
{
    if (mMap && mPacketInterface && ui->updateRouteFromMapBox->isChecked()) {
        mMap->setEnabled(false);
        bool ok = mPacketInterface->removeLastRoutePoint(mId);
        mMap->setEnabled(true);

        if (!ok) {
            QMessageBox::warning(this, "Autopilot",
                                 "No ack received, so the the last route point was most likely not removed.");
        }
    }
}

void CarInterface::nmeaReceived(quint8 id, QByteArray nmea_msg)
{
    if (id == mId) {
        ui->nmeaBrowser->append(QString::fromLocal8Bit(nmea_msg));
        mNmeaForwardServer->broadcastData(nmea_msg);
    }
}

void CarInterface::configurationReceived(quint8 id, MAIN_CONFIG config)
{
    if (id == mId) {
        setConfGui(config);
        QString str;
        str.sprintf("Car %d: Configuration Received", id);
        emit showStatusInfo(str, true);
    }
}

void CarInterface::on_terminalSendButton_clicked()
{
    emit terminalCmd(mId, ui->terminalEdit->text());
    ui->terminalEdit->clear();
}

void CarInterface::on_terminalClearButton_clicked()
{
    ui->terminalBrowser->clear();
}

void CarInterface::on_idBox_valueChanged(int arg1)
{
    if (mMap) {
        CarInfo *car = mMap->getCarInfo(mId);
        car->setId(arg1, true);
    }

    mId = arg1;
}

void CarInterface::on_magSampleClearButton_clicked()
{
    mMagSamples.clear();
}

void CarInterface::on_magSampleSaveButton_clicked()
{
    QString path;
    path = QFileDialog::getSaveFileName(this, tr("Choose where to save the magnetometer samples"));
    if (path.isNull()) {
        return;
    }

    QFile file(path);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);

    QVectorIterator<QVector<double> > i(mMagSamples);
    while (i.hasNext()) {
        QVector<double> element = i.next();
        out << element[0] << "\t" << element[1] << "\t" << element[2] << "\n";
    }

    file.close();
}

void CarInterface::on_bldcToolUdpBox_toggled(bool checked)
{
    if (checked) {
        if (!mUdpSocket->bind(QHostAddress::Any, mUdpPort)) {
            qWarning() << "Binding UDP socket failed.";
            QMessageBox::warning(this, "UDP Server Error",
                                 "Creating UDP server failed. Make sure that the port is not "
                                 "already in use.");
            ui->bldcToolUdpBox->setChecked(false);
        }
    } else {
        mUdpSocket->close();
    }
}

void CarInterface::on_autopilotBox_toggled(bool checked)
{
    if (!ui->autopilotBox->isEnabled()) {
        return;
    }

    if (mPacketInterface) {
        ui->autopilotBox->setEnabled(false);
        bool ok = mPacketInterface->setApActive(mId, checked);

        if (!ok) {
            ui->autopilotBox->setChecked(!checked);
            QMessageBox::warning(this, "Autopilot",
                                 "No ack received, so the autopilot state is unknown.");
        }

        ui->autopilotBox->setEnabled(true);
    }
}

void CarInterface::on_clearRouteButton_clicked()
{
    if (mPacketInterface) {
        ui->clearRouteButton->setEnabled(false);
        bool ok = mPacketInterface->clearRoute(mId);
        ui->clearRouteButton->setEnabled(true);

        if (!ok) {
            QMessageBox::warning(this, "Autopilot",
                                 "No ack received on clear route, so the route is most likely not cleared.");
        }
    }
}

void CarInterface::on_servoDirectSlider_valueChanged(int value)
{
    double val_mapped = (double)value / 1000.0;
    ui->servoDirectNumber->display(val_mapped);
    emit setServoDirect(mId, val_mapped);
}

void CarInterface::on_servoMappedSlider_valueChanged(int value)
{
    double val_mapped = (double)value / 1000.0;
    ui->servoMappedNumber->display(val_mapped);
    emit setRcCurrent(mId, 0.0, val_mapped);
}

void CarInterface::on_nmeaServerActiveBox_toggled(bool checked)
{
    if (checked) {
        if (!mNmeaForwardServer->startTcpServer(ui->nmeaServerPortBox->value())) {
            QMessageBox::warning(this, "TCP Server Error",
                                 "Creating TCP server for NMEA data failed. Make sure that the port is not "
                                 "already in use.");
            ui->nmeaServerActiveBox->setChecked(false);
        }
    } else {
        mNmeaForwardServer->stopServer();
    }
}

void CarInterface::on_confReadButton_clicked()
{
    if (mPacketInterface) {
        mPacketInterface->getConfiguration(mId);
    }
}

void CarInterface::on_confReadDefaultButton_clicked()
{
    if (mPacketInterface) {
        mPacketInterface->getDefaultConfiguration(mId);
    }
}

void CarInterface::on_confWriteButton_clicked()
{
    if (mPacketInterface) {
        MAIN_CONFIG conf;
        getConfGui(conf);
        ui->confWriteButton->setEnabled(false);
        bool ok = mPacketInterface->setConfiguration(mId, conf, 5);
        ui->confWriteButton->setEnabled(true);

        if (!ok) {
            QMessageBox::warning(this, "Configuration",
                                 "Could not write configuration.");
        }
    }
}

void CarInterface::getConfGui(MAIN_CONFIG &conf)
{
    conf.mag_comp = ui->confMagCompBox->isChecked();
    conf.yaw_imu_gain = ui->confYawImuGainBox->value();

    conf.mag_cal_cx = ui->confMagCxBox->value();
    conf.mag_cal_cy = ui->confMagCyBox->value();
    conf.mag_cal_cz = ui->confMagCzBox->value();
    conf.mag_cal_xx = ui->confMagXxBox->value();
    conf.mag_cal_xy = ui->confMagXyBox->value();
    conf.mag_cal_xz = ui->confMagXzBox->value();
    conf.mag_cal_yx = ui->confMagYxBox->value();
    conf.mag_cal_yy = ui->confMagYyBox->value();
    conf.mag_cal_yz = ui->confMagYzBox->value();
    conf.mag_cal_zx = ui->confMagZxBox->value();
    conf.mag_cal_zy = ui->confMagZyBox->value();
    conf.mag_cal_zz = ui->confMagZzBox->value();

    conf.gear_ratio = ui->confGearRatioBox->value();
    conf.wheel_diam = ui->confWheelDiamBox->value();
    conf.motor_poles = ui->confMotorPoleBox->value();
    conf.steering_center = ui->confServoCenterBox->value();
    conf.steering_left = ui->confServoLeftBox->value();
    conf.steering_right = ui->confServoRightBox->value();
    conf.steering_ramp_time = ui->confSteeringRampBox->value();
    conf.axis_distance = ui->confAxisDistanceBox->value();

    conf.steering_max_angle_rad = atan(ui->confAxisDistanceBox->value() / ui->confTurnRadBox->value());
}

void CarInterface::setConfGui(MAIN_CONFIG &conf)
{
    ui->confMagCompBox->setChecked(conf.mag_comp);
    ui->confYawImuGainBox->setValue(conf.yaw_imu_gain);

    ui->confMagCxBox->setValue(conf.mag_cal_cx);
    ui->confMagCyBox->setValue(conf.mag_cal_cy);
    ui->confMagCzBox->setValue(conf.mag_cal_cz);
    ui->confMagXxBox->setValue(conf.mag_cal_xx);
    ui->confMagXyBox->setValue(conf.mag_cal_xy);
    ui->confMagXzBox->setValue(conf.mag_cal_xz);
    ui->confMagYzBox->setValue(conf.mag_cal_yx);
    ui->confMagYyBox->setValue(conf.mag_cal_yy);
    ui->confMagYzBox->setValue(conf.mag_cal_yz);
    ui->confMagZxBox->setValue(conf.mag_cal_zx);
    ui->confMagZyBox->setValue(conf.mag_cal_zy);
    ui->confMagZzBox->setValue(conf.mag_cal_zz);

    ui->confGearRatioBox->setValue(conf.gear_ratio);
    ui->confWheelDiamBox->setValue(conf.wheel_diam);
    ui->confMotorPoleBox->setValue(conf.motor_poles);
    ui->confServoCenterBox->setValue(conf.steering_center);
    ui->confServoLeftBox->setValue(conf.steering_left);
    ui->confServoRightBox->setValue(conf.steering_right);
    ui->confSteeringRampBox->setValue(conf.steering_ramp_time);
    ui->confAxisDistanceBox->setValue(conf.axis_distance);

    ui->confTurnRadBox->setValue(conf.axis_distance / tan(conf.steering_max_angle_rad));
}

void CarInterface::on_nmeaLogChooseButton_clicked()
{
    QString path;
    path = QFileDialog::getSaveFileName(this, tr("Choose where to save the NMEA log"));
    if (path.isNull()) {
        return;
    }

    ui->nmeaLogEdit->setText(path);
}

void CarInterface::on_nmeaLogActiveBox_toggled(bool checked)
{
    if (checked) {
        bool ok = mNmeaForwardServer->logToFile(ui->nmeaLogEdit->text());

        if (!ok) {
            QMessageBox::warning(this, "NMEA Log",
                                 "Could not open log file.");
            ui->nmeaLogActiveBox->setChecked(false);
        }
    } else {
        mNmeaForwardServer->logStop();
    }
}
