/*
    Copyright 2017 Benjamin Vedder	benjamin@vedder.se

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

#include "copterinterface.h"
#include "ui_copterinterface.h"

#include "copterinfo.h"
#include "utility.h"
#include <QFileDialog>
#include <QMessageBox>
#include <cmath>
#include <QTime>
#include <QDateTime>

CopterInterface::CopterInterface(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CopterInterface)
{
    ui->setupUi(this);

#ifdef HAS_OPENGL
    mOrientationWidget = new OrientationWidget(this, OrientationWidget::MODEL_QUADROTOR);
    ui->orientationLayout->removeItem(ui->orientationSpacer);
    ui->orientationLayout->insertWidget(0, mOrientationWidget, 1);
#endif

    mMap = 0;
    mPacketInterface = 0;
    mId = 0;
    settingsReadDone = false;

    mAltitudeData.resize(1000);
    mAltitudeXAxis.resize(1000);
    for(int i = 0;i < mAltitudeXAxis.size();i++) {
        mAltitudeXAxis[mAltitudeXAxis.size() - i - 1] = (40.0 / 1000.0 * i);
    }

    ui->altitudePlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->altitudePlot->clearGraphs();
    ui->altitudePlot->addGraph();
    ui->altitudePlot->xAxis->setRangeReversed(true);
    ui->altitudePlot->graph()->setPen(QPen(Qt::black));
    ui->altitudePlot->graph()->setData(mAltitudeXAxis, mAltitudeData);
    ui->altitudePlot->graph()->setName(tr("Altitude"));
    ui->altitudePlot->rescaleAxes();
    ui->altitudePlot->xAxis->setLabel("Seconds");
    ui->altitudePlot->yAxis->setLabel("Meters");
    ui->altitudePlot->legend->setVisible(true);
    ui->altitudePlot->replot();

    mTimer = new QTimer(this);
    mTimer->start(40);

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    connect(ui->confCommonWidget, SIGNAL(loadMagCal()), this, SLOT(loadMagCal()));
}

CopterInterface::~CopterInterface()
{
    if (mMap) {
        mMap->removeCopter(mId);
    }

    delete ui;
}

void CopterInterface::setID(int id)
{
    ui->idBox->setValue(id);
}

int CopterInterface::getId()
{
    return mId;
}

bool CopterInterface::pollData()
{
    return ui->pollBox->isChecked();
}

bool CopterInterface::updateRouteFromMap()
{
    return ui->updateRouteFromMapBox->isChecked();
}

void CopterInterface::setOrientation(double roll, double pitch, double yaw)
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

void CopterInterface::setStateData(MULTIROTOR_STATE data)
{
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

    // Battery bar
    double battp = utility::map(data.vin, 10.2, 12.6, 0.0, 100.0);
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

    // Height
    mAltitudeData.append(data.pz);
    mAltitudeData.remove(0, 1);
    ui->altitudePlot->graph(0)->setData(mAltitudeXAxis, mAltitudeData);
    ui->altitudePlot->rescaleAxes();
    ui->altitudePlot->replot();

    if (mMap) {
        CopterInfo *copter = mMap->getCopterInfo(mId);
        LocPoint loc = copter->getLocation();
        LocPoint loc_gps = copter->getLocationGps();
        LocPoint ap_goal = copter->getApGoal();
        loc.setRoll(data.roll * M_PI / 180.0);
        loc.setPitch(data.pitch * M_PI / 180.0);
        loc.setYaw(data.yaw * M_PI / 180.0);
        loc.setXY(data.px, data.py);
        loc_gps.setXY(data.px_gps, data.py_gps);
        ap_goal.setXY(data.ap_goal_px, data.ap_goal_py);
        copter->setLocation(loc);
        copter->setLocationGps(loc_gps);
        copter->setApGoal(ap_goal);
        copter->setTime(data.ms_today);
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

void CopterInterface::setMap(MapWidget *map)
{
    mMap = map;
    CopterInfo copter(mId);
    mMap->addCopter(copter);

    connect(mMap, SIGNAL(routePointAdded(LocPoint)),
            this, SLOT(routePointSet(LocPoint)));
    connect(mMap, SIGNAL(lastRoutePointRemoved(LocPoint)),
            this, SLOT(lastRoutePointRemoved()));
}

void CopterInterface::setPacketInterface(PacketInterface *packetInterface)
{
    mPacketInterface = packetInterface;

    connect(this, SIGNAL(terminalCmd(quint8,QString)),
            mPacketInterface, SLOT(sendTerminalCmd(quint8,QString)));
    connect(mPacketInterface, SIGNAL(printReceived(quint8,QString)),
            this, SLOT(terminalPrint(quint8,QString)));
    connect(mPacketInterface, SIGNAL(nmeaRadioReceived(quint8,QByteArray)),
            this, SLOT(nmeaReceived(quint8,QByteArray)));
    connect(mPacketInterface, SIGNAL(configurationReceived(quint8,MAIN_CONFIG)),
            this, SLOT(configurationReceived(quint8,MAIN_CONFIG)));
}

void CopterInterface::setControlValues(double throttle, double roll, double pitch, double yaw)
{
    if (ui->joystickControlBox->isChecked() && mPacketInterface) {
        mPacketInterface->mrRcControl(mId, throttle, roll, pitch, yaw);
    }
}

void CopterInterface::setCtrlAp()
{
    ui->autopilotBox->setChecked(true);
    ui->joystickControlBox->setChecked(false);
}

void CopterInterface::setCtrlJs()
{
    ui->autopilotBox->setChecked(false);
    ui->joystickControlBox->setChecked(true);
}

bool CopterInterface::setAp(bool on)
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

void CopterInterface::timerSlot()
{
    if (ui->overrideActiveBox->isChecked() && mPacketInterface) {
        mPacketInterface->mrOverridePower(mId,
                                          ui->overriderMotorFlFBox->value(),
                                          ui->overriderMotorBlLBox->value(),
                                          ui->overriderMotorFrRBox->value(),
                                          ui->overriderMotorBrBBox->value());
    }
}

void CopterInterface::terminalPrint(quint8 id, QString str)
{
    if (id == mId) {
        ui->terminalBrowser->append(str);
    }
}

void CopterInterface::routePointSet(LocPoint pos)
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

void CopterInterface::lastRoutePointRemoved()
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

void CopterInterface::nmeaReceived(quint8 id, QByteArray nmea_msg)
{
    if (id == mId) {
        ui->nmeaWidget->inputNmea(nmea_msg);
    }
}

void CopterInterface::configurationReceived(quint8 id, MAIN_CONFIG config)
{
    if (id == mId) {
        setConfGui(config);
        settingsReadDone = true;
        QString str;
        str.sprintf("Copter %d: Configuration Received", id);
        emit showStatusInfo(str, true);
    }
}

void CopterInterface::loadMagCal()
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

void CopterInterface::on_idBox_valueChanged(int arg1)
{
    if (mMap) {
        CopterInfo *copter = mMap->getCopterInfo(mId);
        copter->setId(arg1, true);
    }

    mId = arg1;
}

void CopterInterface::on_terminalSendButton_clicked()
{
    emit terminalCmd(mId, ui->terminalEdit->text());
    ui->terminalEdit->clear();
}

void CopterInterface::on_confWriteButton_clicked()
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

void CopterInterface::on_confReadDefaultButton_clicked()
{
    if (mPacketInterface) {
        mPacketInterface->getDefaultConfiguration(mId);
    }
}

void CopterInterface::on_confReadButton_clicked()
{
    if (mPacketInterface) {
        mPacketInterface->getConfiguration(mId);
    }
}

void CopterInterface::getConfGui(MAIN_CONFIG &conf)
{
    conf.mr.vel_decay_e = ui->confVelDecayEBox->value();
    conf.mr.vel_decay_l = ui->confVelDecayLBox->value();
    conf.mr.vel_max = ui->confVelMaxBox->value() / 3.6;

    conf.mr.map_min_x = ui->confMapMinXBox->value();
    conf.mr.map_max_x = ui->confMapMaxXBox->value();
    conf.mr.map_min_y = ui->confMapMinYBox->value();
    conf.mr.map_max_y = ui->confMapMaxYBox->value();

    conf.mr.vel_gain_p = ui->confVelGainPBox->value();
    conf.mr.vel_gain_i = ui->confVelGainIBox->value();
    conf.mr.vel_gain_d = ui->confVelGainDBox->value();

    conf.mr.tilt_gain_p = ui->confTiltGainPBox->value();
    conf.mr.tilt_gain_i = ui->confTiltGainIBox->value();
    conf.mr.tilt_gain_d = ui->confTiltGainDBox->value();

    conf.mr.max_corr_error = ui->confMaxCorrErrorBox->value();
    conf.mr.max_tilt_error = ui->confMaxTiltErrorBox->value();

    conf.mr.ctrl_gain_roll_p = ui->confCtrlGainRollPBox->value();
    conf.mr.ctrl_gain_roll_i = ui->confCtrlGainRollIBox->value();
    conf.mr.ctrl_gain_roll_dp = ui->confCtrlGainRollDPBox->value();
    conf.mr.ctrl_gain_roll_de = ui->confCtrlGainRollDEBox->value();

    conf.mr.ctrl_gain_pitch_p = ui->confCtrlGainPitchPBox->value();
    conf.mr.ctrl_gain_pitch_i = ui->confCtrlGainPitchIBox->value();
    conf.mr.ctrl_gain_pitch_dp = ui->confCtrlGainPitchDPBox->value();
    conf.mr.ctrl_gain_pitch_de = ui->confCtrlGainPitchDEBox->value();

    conf.mr.ctrl_gain_yaw_p = ui->confCtrlGainYawPBox->value();
    conf.mr.ctrl_gain_yaw_i = ui->confCtrlGainYawIBox->value();
    conf.mr.ctrl_gain_yaw_dp = ui->confCtrlGainYawDPBox->value();
    conf.mr.ctrl_gain_yaw_de = ui->confCtrlGainYawDEBox->value();

    conf.mr.ctrl_gain_pos_p = ui->confCtrlGainPosPBox->value();
    conf.mr.ctrl_gain_pos_i = ui->confCtrlGainPosIBox->value();
    conf.mr.ctrl_gain_pos_d = ui->confCtrlGainPosDBox->value();

    conf.mr.ctrl_gain_alt_p = ui->confCtrlGainAltPBox->value();
    conf.mr.ctrl_gain_alt_i = ui->confCtrlGainAltIBox->value();
    conf.mr.ctrl_gain_alt_d = ui->confCtrlGainAltDBox->value();

    conf.mr.js_gain_tilt = ui->confJsGainTiltBox->value();
    conf.mr.js_gain_yaw = ui->confJsGainYawBox->value();
    conf.mr.js_mode_rate = ui->confJsModeRateBox->isChecked();

    conf.mr.motor_fl_f = ui->confMotorFlFBox->value();
    conf.mr.motor_bl_l = ui->confMotorBlLBox->value();
    conf.mr.motor_fr_r = ui->confMotorFrRBox->value();
    conf.mr.motor_br_b = ui->confMotorBrBBox->value();
    conf.mr.motors_x = ui->confMotorsXBox->isChecked();
    conf.mr.motors_cw = ui->confMotorsCwBox->isChecked();
    conf.mr.motor_pwm_min_us = ui->confMotorMinPulseBox->value();
    conf.mr.motor_pwm_max_us = ui->confMotorMaxPulseBox->value();

    ui->confCommonWidget->getConfGui(conf);
}

void CopterInterface::setConfGui(MAIN_CONFIG &conf)
{
    ui->confVelDecayEBox->setValue(conf.mr.vel_decay_e);
    ui->confVelDecayLBox->setValue(conf.mr.vel_decay_l);
    ui->confVelMaxBox->setValue(conf.mr.vel_max * 3.6);

    ui->confMapMinXBox->setValue(conf.mr.map_min_x);
    ui->confMapMaxXBox->setValue(conf.mr.map_max_x);
    ui->confMapMinYBox->setValue(conf.mr.map_min_y);
    ui->confMapMaxYBox->setValue(conf.mr.map_max_y);

    ui->confVelGainPBox->setValue(conf.mr.vel_gain_p);
    ui->confVelGainIBox->setValue(conf.mr.vel_gain_i);
    ui->confVelGainDBox->setValue(conf.mr.vel_gain_d);

    ui->confTiltGainPBox->setValue(conf.mr.tilt_gain_p);
    ui->confTiltGainIBox->setValue(conf.mr.tilt_gain_i);
    ui->confTiltGainDBox->setValue(conf.mr.tilt_gain_d);

    ui->confMaxCorrErrorBox->setValue(conf.mr.max_corr_error);
    ui->confMaxTiltErrorBox->setValue(conf.mr.max_tilt_error);

    ui->confCtrlGainRollPBox->setValue(conf.mr.ctrl_gain_roll_p);
    ui->confCtrlGainRollIBox->setValue(conf.mr.ctrl_gain_roll_i);
    ui->confCtrlGainRollDPBox->setValue(conf.mr.ctrl_gain_roll_dp);
    ui->confCtrlGainRollDEBox->setValue(conf.mr.ctrl_gain_roll_de);

    ui->confCtrlGainPitchPBox->setValue(conf.mr.ctrl_gain_pitch_p);
    ui->confCtrlGainPitchIBox->setValue(conf.mr.ctrl_gain_pitch_i);
    ui->confCtrlGainPitchDPBox->setValue(conf.mr.ctrl_gain_pitch_dp);
    ui->confCtrlGainPitchDEBox->setValue(conf.mr.ctrl_gain_pitch_de);

    ui->confCtrlGainYawPBox->setValue(conf.mr.ctrl_gain_yaw_p);
    ui->confCtrlGainYawIBox->setValue(conf.mr.ctrl_gain_yaw_i);
    ui->confCtrlGainYawDPBox->setValue(conf.mr.ctrl_gain_yaw_dp);
    ui->confCtrlGainYawDEBox->setValue(conf.mr.ctrl_gain_yaw_de);

    ui->confCtrlGainPosPBox->setValue(conf.mr.ctrl_gain_pos_p);
    ui->confCtrlGainPosIBox->setValue(conf.mr.ctrl_gain_pos_i);
    ui->confCtrlGainPosDBox->setValue(conf.mr.ctrl_gain_pos_d);

    ui->confCtrlGainAltPBox->setValue(conf.mr.ctrl_gain_alt_p);
    ui->confCtrlGainAltIBox->setValue(conf.mr.ctrl_gain_alt_i);
    ui->confCtrlGainAltDBox->setValue(conf.mr.ctrl_gain_alt_d);

    ui->confJsGainTiltBox->setValue(conf.mr.js_gain_tilt);
    ui->confJsGainYawBox->setValue(conf.mr.js_gain_yaw);
    ui->confJsModeRateBox->setChecked(conf.mr.js_mode_rate);

    ui->confMotorFlFBox->setValue(conf.mr.motor_fl_f);
    ui->confMotorBlLBox->setValue(conf.mr.motor_bl_l);
    ui->confMotorFrRBox->setValue(conf.mr.motor_fr_r);
    ui->confMotorBrBBox->setValue(conf.mr.motor_br_b);
    ui->confMotorsXBox->setChecked(conf.mr.motors_x);
    ui->confMotorsCwBox->setChecked(conf.mr.motors_cw);
    ui->confMotorMinPulseBox->setValue(conf.mr.motor_pwm_min_us);
    ui->confMotorMaxPulseBox->setValue(conf.mr.motor_pwm_max_us);

    ui->confCommonWidget->setConfGui(conf);
}

void CopterInterface::on_setClockButton_clicked()
{
    if (mPacketInterface) {
        QDateTime date = QDateTime::currentDateTime();
        QTime current = QTime::currentTime().addSecs(-date.offsetFromUtc());
        mPacketInterface->setMsToday(mId, current.msecsSinceStartOfDay());
    }
}

void CopterInterface::on_setClockPiButton_clicked()
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

void CopterInterface::on_rebootPiButton_clicked()
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

void CopterInterface::on_shutdownPiButton_clicked()
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
