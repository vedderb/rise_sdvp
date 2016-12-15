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

#include "carinterface.h"
#include "ui_carinterface.h"
#include "carinfo.h"
#include "utility.h"
#include "nmeaserver.h"
#include <QFileDialog>
#include <QMessageBox>
#include <cmath>
#include <QTime>
#include <QDateTime>

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

#ifdef HAS_OPENGL
    mOrientationWidget = new OrientationWidget(this);
    ui->orientationLayout->removeItem(ui->orientationSpacer);
    ui->orientationLayout->insertWidget(0, mOrientationWidget, 1);
#endif

    // Plots
    ui->accelPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->gyroPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->magPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->experimentPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

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
    mExperimentReplot = false;

    mTimer = new QTimer(this);
    mTimer->start(20);

    mUdpSocket = new QUdpSocket(this);
    mLastHostAddress.clear();
    mUdpPort = 27800;
    mTcpServer = new TcpServerSimple(this);

    mNmeaForwardServer = new TcpBroadcast(this);

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    connect(mUdpSocket, SIGNAL(readyRead()), this, SLOT(udpReadReady()));
    connect(mTcpServer->packet(), SIGNAL(packetReceived(QByteArray&)),
            this, SLOT(tcpRx(QByteArray&)));

    mTcpServer->setUsePacket(true);

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

bool CarInterface::updateRouteFromMap()
{
    return ui->updateRouteFromMapBox->isChecked();
}

void CarInterface::setOrientation(double roll, double pitch, double yaw)
{
#ifdef HAS_OPENGL
    ui->rollBar->setValue(roll);
    ui->pitchBar->setValue(pitch);
    ui->yawBar->setValue(yaw);
    mOrientationWidget->setRollPitchYaw(roll, pitch, yaw);
#else
    (void)roll;
    (void)pitch;
    (void)yaw;
#endif
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
        LocPoint loc_gps = car->getLocationGps();
        LocPoint ap_goal = car->getApGoal();
        loc.setAlpha(data.yaw * M_PI / 180.0);
        loc.setXY(data.px, data.py);
        loc_gps.setXY(data.px_gps, data.py_gps);
        ap_goal.setXY(data.ap_goal_px, data.ap_goal_py);
        ap_goal.setRadius(data.ap_rad);
        car->setLocation(loc);
        car->setLocationGps(loc_gps);
        car->setApGoal(ap_goal);
        car->setTime(data.ms_today);
        mMap->update();
    }

    QVector<double> magXYZ;
    magXYZ.append(data.mag[0]);
    magXYZ.append(data.mag[1]);
    magXYZ.append(data.mag[2]);

    if (ui->magSampleStoreBox->isChecked()) {
        mMagSamples.append(magXYZ);
    }

    // Clock
    if (data.ms_today >= 0) {
        QTime time = QTime::fromMSecsSinceStartOfDay(data.ms_today);
        QDateTime date = QDateTime::currentDateTime();
        QTime current = QTime::currentTime().addSecs(-date.offsetFromUtc());

        int diff = data.ms_today - current.msecsSinceStartOfDay();
        ui->clockLabel->setText(time.toString("HH:mm:ss:zzz") + " " +
                                QString("%1").arg(diff, 6, 10, QChar('0')) + " ms");
    } else {
        ui->clockLabel->setText("00:00:00:000");
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
    connect(mPacketInterface, SIGNAL(plotInitReceived(quint8,QString,QString)),
            this, SLOT(plotInitReceived(quint8,QString,QString)));
    connect(mPacketInterface, SIGNAL(plotDataReceived(quint8,double,double)),
            SLOT(plotDataReceived(quint8,double,double)));
    connect(mPacketInterface, SIGNAL(radarSetupReceived(quint8,radar_settings_t)),
            this, SLOT(radarSetupReceived(quint8,radar_settings_t)));
    connect(mPacketInterface, SIGNAL(radarSamplesReceived(quint8,QVector<QPair<double,double> >)),
            this, SLOT(radarSamplesReceived(quint8,QVector<QPair<double,double> >)));
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

void CarInterface::emergencyStop()
{
    ui->autopilotBox->setChecked(false);
    ui->keyboardControlBox->setChecked(false);
}

void CarInterface::setCtrlAp()
{
    ui->autopilotBox->setChecked(true);
    ui->keyboardControlBox->setChecked(false);
}

void CarInterface::setCtrlKb()
{
    ui->autopilotBox->setChecked(false);
    ui->keyboardControlBox->setChecked(true);
}

bool CarInterface::setAp(bool on)
{
    bool ok = false;

    if (mPacketInterface) {
        ok = mPacketInterface->setApActive(mId, on);

        if (ok) {
            ui->autopilotBox->setChecked(on);
        }
    }

    return ok;
}

void CarInterface::timerSlot()
{
    // Update mag sample label
    static int lastMagSamples = 0;
    if (mMagSamples.size() != lastMagSamples) {
        ui->magSampleLabel->setText(QString::number(mMagSamples.size()) + " Samples");
        lastMagSamples = mMagSamples.size();
    }

    if (mExperimentReplot) {
        ui->experimentPlot->graph()->setData(experimentDataX, experimentDataY);
        ui->experimentPlot->rescaleAxes();
        ui->experimentPlot->replot();
        mExperimentReplot = false;
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

void CarInterface::tcpRx(QByteArray &data)
{
    emit forwardVesc(mId, data);
}

void CarInterface::terminalPrint(quint8 id, QString str)
{
    if (id == mId) {
        ui->terminalBrowser->append(str);
    }
}

void CarInterface::vescFwdReceived(quint8 id, QByteArray data)
{
    if (id == mId) {
        if (QString::compare(mLastHostAddress.toString(), "0.0.0.0") != 0) {
            mUdpSocket->writeDatagram(data, mLastHostAddress, mUdpPort + 1);
        }

        mTcpServer->packet()->sendPacket(data);
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
        if (ui->nmeaPrintBox->isChecked()) {
            ui->nmeaBrowser->append(QString::fromLocal8Bit(nmea_msg));
        }

        mNmeaForwardServer->broadcastData(nmea_msg);

        NmeaServer::nmea_gga_info_t gga;
        QTextStream msgs(nmea_msg);

        while(!msgs.atEnd()) {
            QString str = msgs.readLine();
            QByteArray data = str.toLocal8Bit();

            // Hack
            if (str == "$GPGSA,A,1,,,,,,,,,,,,,,,*1E") {
                ui->nmeaFixTypeLabel->setText("Solution: Invalid");
                ui->nmeaSatsLabel->setText("Satellites: 0");
            }

            if (NmeaServer::decodeNmeaGGA(data, gga) >= 0) {
                QString satStr;
                satStr.sprintf("Satellites: %d", gga.n_sat);
                ui->nmeaSatsLabel->setText(satStr);

                QString fix_type;
                switch (gga.fix_type) {
                case 0: fix_type = "Solution: Invalid"; break;
                case 1: fix_type = "Solution: SPP"; break;
                case 2: fix_type = "Solution: DGPS"; break;
                case 3: fix_type = "Solution: PPS"; break;
                case 4: fix_type = "Solution: RTK Fix"; break;
                case 5: fix_type = "Solution: RTK Float"; break;
                default: fix_type = "Solution: Unknown"; break;
                }

                ui->nmeaFixTypeLabel->setText(fix_type);
            }
        }
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

void CarInterface::plotInitReceived(quint8 id, QString xLabel, QString yLabel)
{
    if (id == mId) {
        experimentDataX.clear();
        experimentDataY.clear();

        ui->experimentPlot->clearGraphs();
        ui->experimentPlot->addGraph();
        ui->experimentPlot->xAxis->setLabel(xLabel);
        ui->experimentPlot->yAxis->setLabel(yLabel);

        mExperimentReplot = true;
    }
}

void CarInterface::plotDataReceived(quint8 id, double x, double y)
{
    if (id == mId) {
        experimentDataX.append(x);
        experimentDataY.append(y);
        mExperimentReplot = true;
    }
}

void CarInterface::radarSetupReceived(quint8 id, radar_settings_t s)
{
    if (id == mId) {
        ui->radarFCenterBox->setValue(s.f_center / 1e9);
        ui->radarFSpanBox->setValue(s.f_span / 1e9);
        ui->radarPointsBox->setValue(s.points);
        ui->radarTSweepBox->setValue(s.t_sweep);
        ui->radarCcXBox->setValue(s.cc_x);
        ui->radarCcYBox->setValue(s.cc_y);
        ui->radarCRadBox->setValue(s.cc_rad);
        ui->radarLogRateBox->setValue((double)s.log_rate_ms / 1000.0);
        ui->radarLogEnBox->setChecked(s.log_en);

        QString str;
        str.sprintf("Car %d: Radar Setup Received", id);
        emit showStatusInfo(str, true);
    }
}

void CarInterface::radarSamplesReceived(quint8 id, QVector<QPair<double, double> > samples)
{
    if (mMap && ui->plotRadarBox->isChecked() && id == mId) {
        CarInfo *ci = mMap->getCarInfo(mId);
        if (ci) {
            LocPoint p_car = ci->getLocation();
            for (int i = 0;i < samples.size();i++) {
                LocPoint p;

                double cx = 0;
                double cy = 0;

                cx = p_car.getX() + samples[i].first * sin(p_car.getAlpha());
                cy = p_car.getY() + samples[i].first * cos(p_car.getAlpha());
                p.setXY(cx, cy);

                QString info;
                info.sprintf("%.1f", samples[i].second);

                p.setInfo(info);
                mMap->addInfoPoint(p);
            }
        }
    }
}

void CarInterface::on_terminalSendButton_clicked()
{
    emit terminalCmd(mId, ui->terminalEdit->text());
    ui->terminalEdit->clear();
}

void CarInterface::on_terminalSendVescButton_clicked()
{
    emit terminalCmd(mId, "vesc " + ui->terminalEditVesc->text());
    ui->terminalEditVesc->clear();
}

void CarInterface::on_terminalSendRadarButton_clicked()
{
    emit terminalCmd(mId, "radar_cmd " + ui->terminalEditRadar->text());
    ui->terminalEditRadar->clear();
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

void CarInterface::on_vescToolTcpBox_toggled(bool checked)
{
    if (checked) {
        if (!mTcpServer->startServer(65102)) {
            qWarning() << "Starting TCP server failed:" << mTcpServer->errorString();
            QMessageBox::warning(this, "TCP Server Error",
                                 tr("Starting TCP server failed. Make sure that the port is not "
                                 "already in use. Error: %1").arg(mTcpServer->errorString()));
            ui->vescToolTcpBox->setChecked(false);
        }
    } else {
        mTcpServer->stopServer();
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
    conf.mag_use = ui->confMagUseBox->isChecked();
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
    conf.steering_range = ui->confServoRangeBox->value();
    conf.steering_ramp_time = ui->confSteeringRampBox->value();
    conf.axis_distance = ui->confAxisDistanceBox->value();

    conf.gps_ant_x = ui->confGpsAntXBox->value();
    conf.gps_ant_y = ui->confGpsAntYBox->value();
    conf.gps_comp = ui->confGpsCorrBox->isChecked();
    conf.gps_corr_gain_stat = ui->confGpsCorrStatBox->value();
    conf.gps_corr_gain_dyn = ui->confGpsCorrDynBox->value();
    conf.gps_corr_gain_yaw = ui->confGpsCorrYawBox->value();

    conf.ap_repeat_routes = ui->confApRepeatBox->isChecked();
    conf.ap_base_rad = ui->confApBaseRadBox->value();
    conf.ap_mode_time = ui->confApModeTimeBox->isChecked();
    conf.ap_max_speed = ui->confApMaxSpeedBox->value() / 3.6;
    conf.ap_time_add_repeat_ms = ui->confApAddRepeatTimeEdit->time().msecsSinceStartOfDay();

    conf.steering_max_angle_rad = atan(ui->confAxisDistanceBox->value() / ui->confTurnRadBox->value());

    conf.log_en = ui->confLogEnBox->isChecked();
    strcpy(conf.log_name, ui->confLogNameEdit->text().toLocal8Bit().data());
}

void CarInterface::setConfGui(MAIN_CONFIG &conf)
{
    ui->confMagUseBox->setChecked(conf.mag_use);
    ui->confMagCompBox->setChecked(conf.mag_comp);
    ui->confYawImuGainBox->setValue(conf.yaw_imu_gain);

    ui->confMagCxBox->setValue(conf.mag_cal_cx);
    ui->confMagCyBox->setValue(conf.mag_cal_cy);
    ui->confMagCzBox->setValue(conf.mag_cal_cz);
    ui->confMagXxBox->setValue(conf.mag_cal_xx);
    ui->confMagXyBox->setValue(conf.mag_cal_xy);
    ui->confMagXzBox->setValue(conf.mag_cal_xz);
    ui->confMagYxBox->setValue(conf.mag_cal_yx);
    ui->confMagYyBox->setValue(conf.mag_cal_yy);
    ui->confMagYzBox->setValue(conf.mag_cal_yz);
    ui->confMagZxBox->setValue(conf.mag_cal_zx);
    ui->confMagZyBox->setValue(conf.mag_cal_zy);
    ui->confMagZzBox->setValue(conf.mag_cal_zz);

    ui->confGearRatioBox->setValue(conf.gear_ratio);
    ui->confWheelDiamBox->setValue(conf.wheel_diam);
    ui->confMotorPoleBox->setValue(conf.motor_poles);
    ui->confServoCenterBox->setValue(conf.steering_center);
    ui->confServoRangeBox->setValue(conf.steering_range);
    ui->confSteeringRampBox->setValue(conf.steering_ramp_time);
    ui->confAxisDistanceBox->setValue(conf.axis_distance);

    ui->confGpsAntXBox->setValue(conf.gps_ant_x);
    ui->confGpsAntYBox->setValue(conf.gps_ant_y);
    ui->confGpsCorrBox->setChecked(conf.gps_comp);
    ui->confGpsCorrStatBox->setValue(conf.gps_corr_gain_stat);
    ui->confGpsCorrDynBox->setValue(conf.gps_corr_gain_dyn);
    ui->confGpsCorrYawBox->setValue(conf.gps_corr_gain_yaw);

    ui->confApRepeatBox->setChecked(conf.ap_repeat_routes);
    ui->confApBaseRadBox->setValue(conf.ap_base_rad);
    ui->confApModeTimeBox->setChecked(conf.ap_mode_time);
    ui->confApMaxSpeedBox->setValue(conf.ap_max_speed * 3.6);
    ui->confApAddRepeatTimeEdit->setTime(QTime::fromMSecsSinceStartOfDay(conf.ap_time_add_repeat_ms));

    ui->confTurnRadBox->setValue(conf.axis_distance / tan(conf.steering_max_angle_rad));

    ui->confLogEnBox->setChecked(conf.log_en);
    ui->confLogNameEdit->setText(QString::fromLocal8Bit(conf.log_name));
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

void CarInterface::on_magCalChooseButton_clicked()
{
    QString path;
    path = QFileDialog::getOpenFileName(this, tr("Choose magnetometer calibration file."));
    if (path.isNull()) {
        return;
    }

    ui->magCalFileEdit->setText(path);
}

void CarInterface::on_magCalLoadButton_clicked()
{
    bool ok = false;
    QFile file(ui->magCalFileEdit->text());
    if (file.exists()) {
        if (file.open(QIODevice::ReadOnly)) {
            QTextStream in(&file);

            ui->confMagCxBox->setValue(in.readLine().toDouble());
            ui->confMagCyBox->setValue(in.readLine().toDouble());
            ui->confMagCzBox->setValue(in.readLine().toDouble());

            ui->confMagXxBox->setValue(in.readLine().toDouble());
            ui->confMagXyBox->setValue(in.readLine().toDouble());
            ui->confMagXzBox->setValue(in.readLine().toDouble());

            ui->confMagYxBox->setValue(in.readLine().toDouble());
            ui->confMagYyBox->setValue(in.readLine().toDouble());
            ui->confMagYzBox->setValue(in.readLine().toDouble());

            ui->confMagZxBox->setValue(in.readLine().toDouble());
            ui->confMagZyBox->setValue(in.readLine().toDouble());
            ui->confMagZzBox->setValue(in.readLine().toDouble());

            ok = true;
        }
    }

    if (!ok) {
        QMessageBox::warning(this, "Mag Cal",
                             "Could not load calibration file.");
    }
}

void CarInterface::on_radarReadButton_clicked()
{
    if (mPacketInterface) {
        mPacketInterface->radarSetupGet(mId);
    }
}

void CarInterface::on_radarWriteButton_clicked()
{
    radar_settings_t s;
    s.f_center = ui->radarFCenterBox->value() * 1e9;
    s.f_span = ui->radarFSpanBox->value() * 1e9;
    s.points = ui->radarPointsBox->value();
    s.t_sweep = ui->radarTSweepBox->value();
    s.cc_x = ui->radarCcXBox->value();
    s.cc_y = ui->radarCcYBox->value();
    s.cc_rad = ui->radarCRadBox->value();
    s.log_rate_ms = (int)ui->radarLogRateBox->value() * 1000;
    s.log_en = ui->radarLogEnBox->isChecked();

    if (mPacketInterface) {
        ui->radarWriteButton->setEnabled(false);
        bool ok = mPacketInterface->radarSetupSet(mId, &s);
        ui->radarWriteButton->setEnabled(true);

        if (!ok) {
            QMessageBox::warning(this, "Setup Radar",
                                 "Could not write radar settings.");
        }
    }
}

void CarInterface::on_radarGetRadCCButton_clicked()
{
    if (mMap) {
        CarInfo *car = mMap->getCarInfo(mId);
        ui->radarCcXBox->setValue(car->getLocation().getX());
        ui->radarCcYBox->setValue(car->getLocation().getY());
    }
}

void CarInterface::on_setClockButton_clicked()
{
    if (mPacketInterface) {
        QDateTime date = QDateTime::currentDateTime();
        QTime current = QTime::currentTime().addSecs(-date.offsetFromUtc());
        mPacketInterface->setMsToday(mId, current.msecsSinceStartOfDay());
    }
}

void CarInterface::on_setClockPiButton_clicked()
{
    if (mPacketInterface) {
        QDateTime date = QDateTime::currentDateTime();
        bool res = mPacketInterface->setSystemTime(mId, date.toTime_t(), date.time().msec() * 1000.0);
        if (!res) {
            QMessageBox::warning(this, "Set time on Raspberry Pi",
                                 "Could not set time, no ack received. Make sure that the "
                                 "connection works.");
        }
    }
}
