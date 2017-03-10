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

    mTimer = new QTimer(this);
    mTimer->start(20);

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
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

    //ui->magCal->addSample(data.mag[0], data.mag[1], data.mag[2]);

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
