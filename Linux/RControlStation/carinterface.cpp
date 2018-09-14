/*
    Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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
#include <QFileDialog>
#include <QMessageBox>
#include <cmath>
#include <QTime>
#include <QDateTime>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>

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

    memset(&mLastCarState, 0, sizeof(CAR_STATE));

    // Plots
    ui->experimentPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->dwPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    mMap = 0;
    mPacketInterface = 0;
    mId = 0;
    mExperimentReplot = false;
    mExperimentPlotNow = 0;
    settingsReadDone = false;

    mTimer = new QTimer(this);
    mTimer->start(20);

    mUdpSocket = new QUdpSocket(this);
    mLastHostAddress.clear();
    mUdpPort = 27800;
    mTcpServer = new TcpServerSimple(this);
    mFaultLast = "Fault code...";

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    connect(mUdpSocket, SIGNAL(readyRead()), this, SLOT(udpReadReady()));
    connect(mTcpServer->packet(), SIGNAL(packetReceived(QByteArray&)),
            this, SLOT(tcpRx(QByteArray&)));
    connect(ui->confCommonWidget, SIGNAL(loadMagCal()),
            this, SLOT(loadMagCal()));

    mTcpServer->setUsePacket(true);

    ui->experimentPlot->xAxis->grid()->setSubGridVisible(true);
    ui->experimentPlot->yAxis->grid()->setSubGridVisible(true);

    ui->dwPlot->addGraph();
    ui->dwPlot->graph()->setName("Fusion Error");
    ui->dwPlot->graph()->setPen(QPen(Qt::blue));
    ui->dwPlot->addGraph();
    ui->dwPlot->graph()->setName("GPS Error");
    ui->dwPlot->graph()->setPen(QPen(Qt::red));
    ui->dwPlot->addGraph(ui->dwPlot->xAxis, ui->dwPlot->yAxis2);
    ui->dwPlot->graph()->setName("Current Anchor");
    ui->dwPlot->graph()->setPen(QPen(Qt::black));
    ui->dwPlot->graph()->setLineStyle(QCPGraph::lsNone);
    ui->dwPlot->graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, 4));
    ui->dwPlot->legend->setVisible(true);
    ui->dwPlot->yAxis2->setVisible(true);
    ui->dwPlot->xAxis->setLabel("Time (s)");
    ui->dwPlot->yAxis->setLabel("Error (m)");
    ui->dwPlot->yAxis2->setLabel("Anchor Now");

    connect(ui->experimentGraph1Button, &QPushButton::toggled,
            [=]() {mExperimentReplot = true;});
    connect(ui->experimentGraph2Button, &QPushButton::toggled,
            [=]() {mExperimentReplot = true;});
    connect(ui->experimentGraph3Button, &QPushButton::toggled,
            [=]() {mExperimentReplot = true;});
    connect(ui->experimentGraph4Button, &QPushButton::toggled,
            [=]() {mExperimentReplot = true;});
    connect(ui->experimentGraph5Button, &QPushButton::toggled,
            [=]() {mExperimentReplot = true;});
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
    mLastCarState = data;

    ui->imuPlot->addSample(data.accel, data.gyro, data.mag);

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

    if (mFaultLast != fault_str) {
        mFaultLast = fault_str;
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
        LocPoint loc_uwb = car->getLocationUwb();
        LocPoint ap_goal = car->getApGoal();
        loc.setYaw(data.yaw * M_PI / 180.0);
        loc.setXY(data.px, data.py);
        loc.setSpeed(data.speed);
        loc_gps.setXY(data.px_gps, data.py_gps);
        loc_uwb.setXY(data.px_uwb, data.py_uwb);
        ap_goal.setXY(data.ap_goal_px, data.ap_goal_py);
        ap_goal.setRadius(data.ap_rad);
        car->setLocation(loc);
        car->setLocationGps(loc_gps);
        car->setLocationUwb(loc_uwb);
        car->setApGoal(ap_goal);
        car->setTime(data.ms_today);
        mMap->update();
    }

    ui->magCal->addSample(data.mag[0], data.mag[1], data.mag[2]);

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

    // Anchor updates
    connect(ui->dwAnch0DrawBox, SIGNAL(toggled(bool)),
            this, SLOT(updateAnchorsMap()));
    connect(ui->dwAnch0IdBox, SIGNAL(valueChanged(int)),
            this, SLOT(updateAnchorsMap()));
    connect(ui->dwAnch0PxBox, SIGNAL(valueChanged(double)),
            this, SLOT(updateAnchorsMap()));
    connect(ui->dwAnch0PyBox, SIGNAL(valueChanged(double)),
            this, SLOT(updateAnchorsMap()));
    connect(ui->dwAnch0PzBox, SIGNAL(valueChanged(double)),
            this, SLOT(updateAnchorsMap()));

    connect(ui->dwAnch1DrawBox, SIGNAL(toggled(bool)),
            this, SLOT(updateAnchorsMap()));
    connect(ui->dwAnch1IdBox, SIGNAL(valueChanged(int)),
            this, SLOT(updateAnchorsMap()));
    connect(ui->dwAnch1PxBox, SIGNAL(valueChanged(double)),
            this, SLOT(updateAnchorsMap()));
    connect(ui->dwAnch1PyBox, SIGNAL(valueChanged(double)),
            this, SLOT(updateAnchorsMap()));
    connect(ui->dwAnch1PzBox, SIGNAL(valueChanged(double)),
            this, SLOT(updateAnchorsMap()));

    connect(ui->dwAnch2DrawBox, SIGNAL(toggled(bool)),
            this, SLOT(updateAnchorsMap()));
    connect(ui->dwAnch2IdBox, SIGNAL(valueChanged(int)),
            this, SLOT(updateAnchorsMap()));
    connect(ui->dwAnch2PxBox, SIGNAL(valueChanged(double)),
            this, SLOT(updateAnchorsMap()));
    connect(ui->dwAnch2PyBox, SIGNAL(valueChanged(double)),
            this, SLOT(updateAnchorsMap()));
    connect(ui->dwAnch2PzBox, SIGNAL(valueChanged(double)),
            this, SLOT(updateAnchorsMap()));
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
    connect(mPacketInterface, SIGNAL(plotAddGraphReceived(quint8,QString)),
            this, SLOT(plotAddGraphReceived(quint8,QString)));
    connect(mPacketInterface, SIGNAL(plotSetGraphReceived(quint8,int)),
            this, SLOT(plotSetGraphReceived(quint8,int)));
    connect(mPacketInterface, SIGNAL(radarSetupReceived(quint8,radar_settings_t)),
            this, SLOT(radarSetupReceived(quint8,radar_settings_t)));
    connect(mPacketInterface, SIGNAL(radarSamplesReceived(quint8,QVector<QPair<double,double> >)),
            this, SLOT(radarSamplesReceived(quint8,QVector<QPair<double,double> >)));
    connect(mPacketInterface, SIGNAL(dwSampleReceived(quint8,DW_LOG_INFO)),
            this, SLOT(dwSampleReceived(quint8,DW_LOG_INFO)));
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
    if (ui->autopilotBox->isChecked()) {
        ui->autopilotBox->setChecked(false);
    } else {
        // Send the AP stop command even if the autopilot was not active in the UI.
        if (mPacketInterface) {
            mPacketInterface->setApActive(mId, false);
        }
    }
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

void CarInterface::disableKbBox()
{
    ui->keyboardControlBox->setChecked(false);
}

void CarInterface::timerSlot()
{   
    if (mExperimentReplot) {
        ui->experimentPlot->clearGraphs();

        for (int i = 0;i < mExperimentPlots.size();i++) {
            switch (i) {
            case 0: if (!ui->experimentGraph1Button->isChecked()) {continue;} break;
            case 1: if (!ui->experimentGraph2Button->isChecked()) {continue;} break;
            case 2: if (!ui->experimentGraph3Button->isChecked()) {continue;} break;
            case 3: if (!ui->experimentGraph4Button->isChecked()) {continue;} break;
            case 4: if (!ui->experimentGraph5Button->isChecked()) {continue;} break;
            default: break;
            }

            ui->experimentPlot->addGraph();
            ui->experimentPlot->graph()->setData(mExperimentPlots.at(i).xData, mExperimentPlots.at(i).yData);
            ui->experimentPlot->graph()->setName(mExperimentPlots.at(i).label);
            ui->experimentPlot->graph()->setPen(QPen(mExperimentPlots.at(i).color));
        }

        ui->experimentPlot->legend->setVisible(mExperimentPlots.size() > 1);

        if (ui->experimentAutoScaleButton->isChecked()) {
            ui->experimentPlot->rescaleAxes();
        }

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
        ui->nmeaWidget->inputNmea(nmea_msg);

        if (mMap) {
            CarInfo *car = mMap->getCarInfo(mId);

            if (car) {
                LocPoint loc_gps = car->getLocationGps();
                loc_gps.setInfo(ui->nmeaWidget->fixType());
                car->setLocationGps(loc_gps);
            }
        }
    }
}

void CarInterface::configurationReceived(quint8 id, MAIN_CONFIG config)
{
    if (id == mId) {
        settingsReadDone = true;
        setConfGui(config);
        QString str;
        str.sprintf("Car %d: Configuration Received", id);
        emit showStatusInfo(str, true);
    }
}

void CarInterface::plotInitReceived(quint8 id, QString xLabel, QString yLabel)
{
    if (id == mId) {
        mExperimentPlots.clear();

        ui->experimentPlot->clearGraphs();
        ui->experimentPlot->xAxis->setLabel(xLabel);
        ui->experimentPlot->yAxis->setLabel(yLabel);

        mExperimentReplot = true;
    }
}

void CarInterface::plotDataReceived(quint8 id, double x, double y)
{
    if (id == mId) {
        if (mExperimentPlots.size() <= mExperimentPlotNow) {
            mExperimentPlots.resize(mExperimentPlotNow + 1);
        }

        mExperimentPlots[mExperimentPlotNow].xData.append(x);
        mExperimentPlots[mExperimentPlotNow].yData.append(y);
        mExperimentReplot = true;
    }
}

void CarInterface::plotAddGraphReceived(quint8 id, QString name)
{
    if (id == mId) {
        mExperimentPlots.resize(mExperimentPlots.size() + 1);
        mExperimentPlots.last().label = name;

        if (mExperimentPlots.size() == 1) {
            mExperimentPlots.last().color = "blue";
        } else if (mExperimentPlots.size() == 2) {
            mExperimentPlots.last().color = "red";
        } else if (mExperimentPlots.size() == 3) {
            mExperimentPlots.last().color = "magenta";
        } else if (mExperimentPlots.size() == 4) {
            mExperimentPlots.last().color = "darkgreen";
        } else if (mExperimentPlots.size() == 5) {
            mExperimentPlots.last().color = "cyan";
        } else {
            mExperimentPlots.last().color = "blue";
        }

        mExperimentReplot = true;
    }
}

void CarInterface::plotSetGraphReceived(quint8 id, int graph)
{
    if (id == mId) {
        mExperimentPlotNow = graph;
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
        ui->radarMapPlotAvgFactorBox->setValue(s.map_plot_avg_factor);
        ui->radarMapPlotMaxDivBox->setValue(s.map_plot_max_div);
        ui->radarPlotModeBox->setValue(s.plot_mode);
        ui->radarMapPlotStartBox->setValue(s.map_plot_start);
        ui->radarMapPlotEndBox->setValue(s.map_plot_end);

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

                cx = p_car.getX() + samples[i].first * sin(p_car.getYaw());
                cy = p_car.getY() + samples[i].first * cos(p_car.getYaw());
                p.setXY(cx, cy);
                p.setDrawLine(false);

                QString info;
                QTime t = QTime::currentTime();

                info.sprintf("%02d:%02d:%02d:%03d\n"
                             "%.1f m (%.0f)",
                             t.hour(), t.minute(), t.second(), t.msec(),
                             samples[i].first, samples[i].second);

                p.setInfo(info);
                mMap->addInfoPoint(p);
            }
        }
    }
}

void CarInterface::dwSampleReceived(quint8 id, DW_LOG_INFO dw)
{
    if (id == mId) {
        mDwData.append(dw);
        plotDwData();
    }
}

void CarInterface::updateAnchorsMap()
{
    if (mMap) {
        bool update = false;

        LocPoint anch0_gui;
        anch0_gui.setId(100 * mId + 0);
        anch0_gui.setXY(ui->dwAnch0PxBox->value(), ui->dwAnch0PyBox->value());
        anch0_gui.setInfo(QString("Anchor %2").arg(ui->dwAnch0IdBox->value()));

        LocPoint anch1_gui;
        anch1_gui.setId(100 * mId + 1);
        anch1_gui.setXY(ui->dwAnch1PxBox->value(), ui->dwAnch1PyBox->value());
        anch1_gui.setInfo(QString("Anchor %2").arg(ui->dwAnch1IdBox->value()));

        LocPoint anch2_gui;
        anch2_gui.setId(100 * mId + 2);
        anch2_gui.setXY(ui->dwAnch2PxBox->value(), ui->dwAnch2PyBox->value());
        anch2_gui.setInfo(QString("Anchor %2").arg(ui->dwAnch2IdBox->value()));

        LocPoint *anch0 = mMap->getAnchor(anch0_gui.getId());
        if (ui->dwAnch0DrawBox->isChecked()) {
            if (anch0) {
                if (*anch0 != anch0_gui) {
                    *anch0 = anch0_gui;
                    update = true;
                }
            } else {
                mMap->addAnchor(anch0_gui);
            }
        } else {
            mMap->removeAnchor(anch0_gui.getId());
        }

        LocPoint *anch1 = mMap->getAnchor(anch1_gui.getId());
        if (ui->dwAnch1DrawBox->isChecked()) {
            if (anch1) {
                if (*anch1 != anch1_gui) {
                    *anch1 = anch1_gui;
                    update = true;
                }
            } else {
                mMap->addAnchor(anch1_gui);
            }
        } else {
            mMap->removeAnchor(anch1_gui.getId());
        }

        LocPoint *anch2 = mMap->getAnchor(anch2_gui.getId());
        if (ui->dwAnch2DrawBox->isChecked()) {
            if (anch2) {
                if (*anch2 != anch2_gui) {
                    *anch2 = anch2_gui;
                    update = true;
                }
            } else {
                mMap->addAnchor(anch2_gui);
            }
        } else {
            mMap->removeAnchor(anch2_gui.getId());
        }

        if (update) {
            mMap->update();
        }
    }
}

void CarInterface::loadMagCal()
{
    if (!ui->magCal->calculateCompensation()) {
        QMessageBox::warning(this, "Load Magnetometer Calibration",
                             "Magnetometer calibration is not done. Please go to "
                             "the calibration tab and collect "
                             "samples, or load a file.");
        return;
    }

    ui->confCommonWidget->setMagComp(ui->magCal->getComp());
    ui->confCommonWidget->setMagCompCenter(ui->magCal->getCenter());
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
    if (!settingsReadDone) {
        QMessageBox::warning(this, "Configuration",
                             "You must read the configuration at least once before writing it. "
                             "Otherwise everything would be set to 0.");
        return;
    }

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
    conf.car.yaw_use_odometry = ui->confOdometryYawBox->isChecked();
    conf.car.yaw_imu_gain = ui->confYawImuGainBox->value();
    conf.car.disable_motor = ui->confMiscDisableMotorBox->isChecked();
    conf.car.simulate_motor = ui->confMiscSimulateMotorBox->isChecked();
    conf.car.clamp_imu_yaw_stationary = ui->confClampImuYawBox->isChecked();

    conf.car.gear_ratio = ui->confGearRatioBox->value();
    conf.car.wheel_diam = ui->confWheelDiamBox->value();
    conf.car.motor_poles = ui->confMotorPoleBox->value();
    conf.car.steering_center = ui->confServoCenterBox->value();
    conf.car.steering_range = ui->confServoRangeBox->value();
    conf.car.steering_ramp_time = ui->confSteeringRampBox->value();
    conf.car.axis_distance = ui->confAxisDistanceBox->value();

    conf.car.steering_max_angle_rad = atan(ui->confAxisDistanceBox->value() / ui->confTurnRadBox->value());

    ui->confCommonWidget->getConfGui(conf);
}

void CarInterface::setConfGui(MAIN_CONFIG &conf)
{
    ui->confOdometryYawBox->setChecked(conf.car.yaw_use_odometry);
    ui->confYawImuGainBox->setValue(conf.car.yaw_imu_gain);
    ui->confMiscDisableMotorBox->setChecked(conf.car.disable_motor);
    ui->confMiscSimulateMotorBox->setChecked(conf.car.simulate_motor);
    ui->confClampImuYawBox->setChecked(conf.car.clamp_imu_yaw_stationary);

    ui->confGearRatioBox->setValue(conf.car.gear_ratio);
    ui->confWheelDiamBox->setValue(conf.car.wheel_diam);
    ui->confMotorPoleBox->setValue(conf.car.motor_poles);
    ui->confServoCenterBox->setValue(conf.car.steering_center);
    ui->confServoRangeBox->setValue(conf.car.steering_range);
    ui->confSteeringRampBox->setValue(conf.car.steering_ramp_time);
    ui->confAxisDistanceBox->setValue(conf.car.axis_distance);

    ui->confTurnRadBox->setValue(conf.car.axis_distance / tan(conf.car.steering_max_angle_rad));

    ui->confCommonWidget->setConfGui(conf);
}

void CarInterface::plotDwData()
{
    QVector<double> time;
    QVector<double> error;
    QVector<double> error_gps;
    QVector<double> anchor;

    for (int i = 0;i < mDwData.size();i++) {
        DW_LOG_INFO &dw = mDwData[i];

        double px = 0;
        double py = 0;
        double pz = 0;
        double anch = -1.0;

        if (dw.dw_anchor == ui->dwAnch0IdBox->value()) {
            px = ui->dwAnch0PxBox->value();
            py = ui->dwAnch0PyBox->value();
            pz = ui->dwAnch0PzBox->value();
            anch = 0.0;
        } else if (dw.dw_anchor == ui->dwAnch1IdBox->value()) {
            px = ui->dwAnch1PxBox->value();
            py = ui->dwAnch1PyBox->value();
            pz = ui->dwAnch1PzBox->value();
            anch = 1.0;
        } else if (dw.dw_anchor == ui->dwAnch2IdBox->value()) {
            px = ui->dwAnch2PxBox->value();
            py = ui->dwAnch2PyBox->value();
            pz = ui->dwAnch2PzBox->value();
            anch = 2.0;
        }

        double err = fabs(sqrt((dw.px - px) * (dw.px - px) +
                               (dw.py - py) * (dw.py - py) +
                               (pz) * (pz)) - dw.dw_dist);

        double err_gps = fabs(sqrt((dw.px_gps - px) * (dw.px_gps - px) +
                                   (dw.py_gps - py) * (dw.py_gps - py) +
                                   (pz) * (pz)) - dw.dw_dist);

        time.append((double)dw.time_today_ms / 1000.0);
        error.append(err);
        error_gps.append(err_gps);
        anchor.append(anch);
    }

    ui->dwPlot->graph(0)->setData(time, error);
    ui->dwPlot->graph(1)->setData(time, error_gps);
    ui->dwPlot->graph(2)->setData(time, anchor);
    ui->dwPlot->rescaleAxes();
    ui->dwPlot->yAxis->setRangeLower(0.0);
    ui->dwPlot->yAxis2->setRangeLower(-0.2);
    ui->dwPlot->yAxis2->setRangeUpper(2.2);
    ui->dwPlot->replot();
}

void CarInterface::updateExperimentZoom()
{
    Qt::Orientations plotOrientations = (Qt::Orientations)
            ((ui->experimentHZoomButton->isChecked() ? Qt::Horizontal : 0) |
             (ui->experimentVZoomButton->isChecked() ? Qt::Vertical : 0));
    ui->experimentPlot->axisRect()->setRangeZoom(plotOrientations);
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
    s.log_rate_ms = (int)(ui->radarLogRateBox->value() * 1000.0);
    s.log_en = ui->radarLogEnBox->isChecked();
    s.map_plot_avg_factor = ui->radarMapPlotAvgFactorBox->value();
    s.map_plot_max_div = ui->radarMapPlotMaxDivBox->value();
    s.plot_mode = ui->radarPlotModeBox->value();
    s.map_plot_start = ui->radarMapPlotStartBox->value();
    s.map_plot_end = ui->radarMapPlotEndBox->value();

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

void CarInterface::on_rebootPiButton_clicked()
{
    if (mPacketInterface) {
        bool res = mPacketInterface->sendReboot(mId, false);
        if (!res) {
            QMessageBox::warning(this, "Reboot Raspberry Pi",
                                 "Could not reboot the Raspberry Pi, no ack received. Make sure that the "
                                 "connection works.");
        }
    }
}

void CarInterface::on_shutdownPiButton_clicked()
{
    if (mPacketInterface) {
        bool res = mPacketInterface->sendReboot(mId, true);
        if (!res) {
            QMessageBox::warning(this, "Shutdown Raspberry Pi",
                                 "Could not shut down the Raspberry Pi, no ack received. Make sure that the "
                                 "connection works.");
        }
    }
}

void CarInterface::on_dwAnch0GetButton_clicked()
{
    ui->dwAnch0PxBox->setValue(mLastCarState.px);
    ui->dwAnch0PyBox->setValue(mLastCarState.py);
}

void CarInterface::on_dwAnch1GetButton_clicked()
{
    ui->dwAnch1PxBox->setValue(mLastCarState.px);
    ui->dwAnch1PyBox->setValue(mLastCarState.py);
}

void CarInterface::on_dwAnch2GetButton_clicked()
{
    ui->dwAnch2PxBox->setValue(mLastCarState.px);
    ui->dwAnch2PyBox->setValue(mLastCarState.py);
}

void CarInterface::on_dwClearSamplesButton_clicked()
{
    mDwData.clear();
    plotDwData();
}

void CarInterface::on_experimentSavePngButton_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save Image"), "",
                                                    tr("PNG Files (*.png)"));

    if (!fileName.isEmpty()) {
        if (!fileName.toLower().endsWith(".png")) {
            fileName.append(".png");
        }

        ui->experimentPlot->savePng(fileName,
                                    ui->experimentWBox->value(),
                                    ui->experimentHBox->value(),
                                    ui->experimentScaleBox->value());
    }
}

void CarInterface::on_experimentSavePdfButton_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save PDF"), "",
                                                    tr("PDF Files (*.pdf)"));

    if (!fileName.isEmpty()) {
        if (!fileName.toLower().endsWith(".pdf")) {
            fileName.append(".pdf");
        }

        ui->experimentPlot->savePdf(fileName,
                                    ui->experimentWBox->value(),
                                    ui->experimentHBox->value());
    }
}

void CarInterface::on_experimentSaveXmlButton_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save Plot"), "",
                                                    tr("Xml files (*.xml)"));

    if (filename.isEmpty()) {
        return;
    }

    if (!filename.toLower().endsWith(".xml")) {
        filename.append(".xml");
    }

    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        QMessageBox::critical(this, "Save Plot",
                              "Could not open\n" + filename + "\nfor writing");
        return;
    }

    QXmlStreamWriter stream(&file);
    stream.setCodec("UTF-8");
    stream.setAutoFormatting(true);
    stream.writeStartDocument();

    stream.writeStartElement("plot");
    stream.writeTextElement("xlabel", ui->experimentPlot->xAxis->label());
    stream.writeTextElement("ylabel", ui->experimentPlot->yAxis->label());

    for (EXPERIMENT_PLOT p: mExperimentPlots) {
        stream.writeStartElement("graph");
        stream.writeTextElement("label", p.label);
        stream.writeTextElement("color", p.color);
        for (int i = 0;i < p.xData.size();i++) {
            stream.writeStartElement("point");
            stream.writeTextElement("x", QString::number(p.xData.at(i)));
            stream.writeTextElement("y", QString::number(p.yData.at(i)));
            stream.writeEndElement();
        }
        stream.writeEndElement();
    }

    stream.writeEndDocument();
    file.close();
}

void CarInterface::on_experimentLoadXmlButton_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load Plot"), "",
                                                    tr("Xml files (*.xml)"));

    if (!filename.isEmpty()) {
        QFile file(filename);
        if (!file.open(QIODevice::ReadOnly)) {
            QMessageBox::critical(this, "Load Plot",
                                  "Could not open\n" + filename + "\nfor reading");
            return;
        }

        QXmlStreamReader stream(&file);

        // Look for plot tag
        bool plots_found = false;
        while (stream.readNextStartElement()) {
            if (stream.name() == "plot") {
                plots_found = true;
                break;
            }
        }

        if (plots_found) {
            mExperimentPlots.clear();

            while (stream.readNextStartElement()) {
                QString name = stream.name().toString();

                if (name == "xlabel") {
                    ui->experimentPlot->xAxis->setLabel(stream.readElementText());
                } else if (name == "ylabel") {
                    ui->experimentPlot->yAxis->setLabel(stream.readElementText());
                } else if (name == "graph") {
                    EXPERIMENT_PLOT p;

                    while (stream.readNextStartElement()) {
                        QString name2 = stream.name().toString();

                        if (name2 == "label") {
                            p.label = stream.readElementText();
                        } else if (name2 == "color") {
                            p.color = stream.readElementText();
                        } else if (name2 == "point") {
                            while (stream.readNextStartElement()) {
                                QString name3 = stream.name().toString();

                                if (name3 == "x") {
                                    p.xData.append(stream.readElementText().toDouble());
                                } else if (name3 == "y") {
                                    p.yData.append(stream.readElementText().toDouble());
                                } else {
                                    qWarning() << ": Unknown XML element :" << name2;
                                    stream.skipCurrentElement();
                                }
                            }
                        } else {
                            qWarning() << ": Unknown XML element :" << name2;
                            stream.skipCurrentElement();
                        }

                        if (stream.hasError()) {
                            qWarning() << " : XML ERROR :" << stream.errorString();
                        }
                    }

                    mExperimentPlots.append(p);
                }

                if (stream.hasError()) {
                    qWarning() << "XML ERROR :" << stream.errorString();
                    qWarning() << stream.lineNumber() << stream.columnNumber();
                }
            }

            mExperimentReplot = true;

            file.close();
            showStatusInfo("Loaded plot", true);
        } else {
            QMessageBox::critical(this, "Load Plot",
                                  "plot tag not found in " + filename);
        }
    }
}

void CarInterface::on_experimentHZoomButton_toggled(bool checked)
{
    (void)checked;
    updateExperimentZoom();
}

void CarInterface::on_experimentVZoomButton_toggled(bool checked)
{
    (void)checked;
    updateExperimentZoom();
}
