/*
    Copyright 2016 - 2018 Benjamin Vedder	benjamin@vedder.se

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

#include "basestation.h"
#include "ui_basestation.h"
#include <QDebug>
#include <QMessageBox>
#include <QIcon>
#include <QSerialPortInfo>
#include <cmath>
#include "nmeaserver.h"
#include "utility.h"
#include "rtcm3_simple.h"

BaseStation::BaseStation(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::BaseStation)
{
    ui->setupUi(this);

    mMap = 0;
    mUblox = new Ublox(this);
    mTcpServer = new TcpBroadcast(this);
    mBasePosCnt = 0;
    mBasePosSet = false;

    mTimer = new QTimer(this);
    mTimer->start(20);

    connect(mTimer, SIGNAL(timeout()),
            this, SLOT(timerSlot()));
    connect(mUblox, SIGNAL(rxRawx(ubx_rxm_rawx)),
            this, SLOT(rxRawx(ubx_rxm_rawx)));
    connect(mUblox, SIGNAL(rxNavSol(ubx_nav_sol)),
            this, SLOT(rxNavSol(ubx_nav_sol)));

    on_ubxSerialRefreshButton_clicked();
}

BaseStation::~BaseStation()
{
    delete ui;
}

void BaseStation::setMap(MapWidget *map)
{
    mMap = map;
}

void BaseStation::timerSlot()
{
    // Update serial connected label
    static bool wasSerialConnected = false;
    if (wasSerialConnected != mUblox->isSerialConnected()) {
        wasSerialConnected = mUblox->isSerialConnected();

        if (wasSerialConnected) {
            ui->ubxSerialConnectedLabel->setText("Connected");
        } else {
            ui->ubxSerialConnectedLabel->setText("Not connected");
        }
    }
}

void BaseStation::rxRawx(ubx_rxm_rawx rawx)
{
    uint8_t data_gps[1024];
    uint8_t data_glo[1024];
    uint8_t data_ref[512];

    int gps_len = 0;
    int glo_len = 0;
    int ref_len = 0;

    rtcm_obs_header_t header;
    rtcm_obs_t obs[rawx.num_meas];

    header.staid = 0;
    header.t_wn = rawx.week;
    header.t_tow = rawx.rcv_tow;
    header.t_tod = fmod(rawx.rcv_tow - (double)rawx.leaps + 10800.0, 86400.0);

    bool has_gps = false;
    bool has_glo = false;
    bool has_ref = false;

    for (int i = 0;i < rawx.num_meas;i++) {
        ubx_rxm_rawx_obs *raw_obs = &rawx.obs[i];

        if (raw_obs->gnss_id == 0) {
            has_gps = true;
        } else if (raw_obs->gnss_id == 6) {
            has_glo = true;
        }
    }

    // GPS
    if (has_gps) {
        int obs_ind = 0;
        for (int i = 0;i < rawx.num_meas;i++) {
            ubx_rxm_rawx_obs *raw_obs = &rawx.obs[i];

            if (raw_obs->gnss_id == 0) {
                obs[obs_ind].P[0] = raw_obs->pr_mes;
                obs[obs_ind].L[0] = raw_obs->cp_mes;
                obs[obs_ind].cn0[0] = raw_obs->cno;
                obs[obs_ind].lock[0] = raw_obs->locktime > 2000 ? 127 : 0;
                obs[obs_ind].code[0] = CODE_L1C;
                obs[obs_ind].prn = raw_obs->sv_id;
                obs_ind++;
            }
        }
        header.sync = has_glo;
        rtcm3_encode_1002(&header, obs, obs_ind, data_gps, &gps_len);
    }

    // GLONASS
    if (has_glo) {
        int obs_ind = 0;
        for (int i = 0;i < rawx.num_meas;i++) {
            ubx_rxm_rawx_obs *raw_obs = &rawx.obs[i];

            if (raw_obs->gnss_id == 6) {
                obs[obs_ind].P[0] = raw_obs->pr_mes;
                obs[obs_ind].L[0] = raw_obs->cp_mes;
                obs[obs_ind].cn0[0] = raw_obs->cno;
                obs[obs_ind].lock[0] = raw_obs->locktime > 2000 ? 127 : 0;
                obs[obs_ind].code[0] = CODE_L1C;
                obs[obs_ind].prn = raw_obs->sv_id;
                obs[obs_ind].freq = raw_obs->freq_id;
                obs_ind++;
            }
        }
        header.sync = 0;
        rtcm3_encode_1010(&header, obs, obs_ind, data_glo, &glo_len);
    }

    // Base station position
    if (ui->sendBaseBox->isChecked()) {
        rtcm_ref_sta_pos_t pos;
        pos.ant_height = ui->refSendAntHBox->value();
        pos.height = ui->refSendHBox->value();
        pos.lat = ui->refSendLatBox->value();
        pos.lon = ui->refSendLonBox->value();
        pos.staid = 0;
        rtcm3_encode_1006(pos, data_ref, &ref_len);
        has_ref = true;
    }

    if (ui->tcpServerBox->isChecked()) {
        QByteArray message;
        if (has_gps) {
            message.append((char*)data_gps, gps_len);
        }

        if (has_glo) {
            message.append((char*)data_glo, glo_len);
        }

        if (has_ref) {
            message.append((char*)data_ref, ref_len);
        }

        mTcpServer->broadcastData(message);
    }

    if (ui->sendVehiclesBox->isChecked()) {
        if (has_gps && has_glo && ((gps_len + glo_len) < 1000)) {
            QByteArray message;
            message.append((char*)data_gps, gps_len);
            message.append((char*)data_glo, glo_len);
            emit rtcmOut(message);
        } else {
            if (has_gps) {
                emit rtcmOut(QByteArray((char*)data_gps, gps_len));
            }

            if (has_glo) {
                emit rtcmOut(QByteArray((char*)data_glo, glo_len));
            }
        }

        // Send base station position every 5 cycles to save bandwidth.
        mBasePosCnt++;
        if (mBasePosCnt >= 5) {
            mBasePosCnt = 0;

            if (has_ref) {
                emit rtcmOut(QByteArray((char*)data_ref, ref_len));
            }
        }
    }
}

void BaseStation::rxNavSol(ubx_nav_sol sol)
{
    double llh[3];
    utility::xyzToLlh(sol.ecef_x, sol.ecef_y, sol.ecef_z, &llh[0], &llh[1], &llh[2]);

    QString txt = QString(
                "Lat:    %1\n"
                "Lon:    %2\n"
                "Height: %3\n"
                "Sats:   %4\n"
                "P ACC:  %5 m").
            arg(llh[0]).
            arg(llh[1]).
            arg(llh[2]).
            arg(sol.num_sv).
            arg(sol.p_acc);

    if (mMap && ui->ubxPlotMapBox->isChecked()) {
        double i_llh[3];

        mMap->getEnuRef(i_llh);

        double xyz[3];
        utility::llhToEnu(i_llh, llh, xyz);

        LocPoint p;
        p.setXY(xyz[0], xyz[1]);
        QString info;

        QString fix_t = "Unknown";
        if (sol.diffsoln) {
            fix_t = "RTK";
            p.setColor(Qt::green);
        } else {
            fix_t = "Single";
            p.setColor(Qt::red);
        }

        info.sprintf("Fix type: %s\n"
                     "Sats    : %d\n"
                     "Height  : %.2f",
                     fix_t.toLocal8Bit().data(),
                     sol.num_sv,
                     llh[2]);

        p.setInfo(info);
        mMap->addInfoPoint(p);

        if (ui->ubxFollowMapBox->isChecked()) {
            mMap->moveView(p.getX(), p.getY());
        }
    }

    ui->ubxText->clear();
    ui->ubxText->appendPlainText(txt);

    if (sol.p_acc < ui->surveyInMinAccBox->value() && ui->surveyInBox->isChecked()) {
        ui->sendBaseBox->setChecked(true);

        if (mBasePosSet) {
            double xyz[3];
            utility::llhToXyz(ui->refSendLatBox->value(),
                              ui->refSendLonBox->value(),
                              ui->refSendHBox->value(),
                              &xyz[0], &xyz[1], &xyz[2]);

            double diff = sqrt(pow(sol.ecef_x - xyz[0], 2.0) +
                    pow(sol.ecef_y - xyz[1], 2.0) +
                    pow(sol.ecef_z - xyz[2], 2.0));

            ui->ubxText->appendPlainText(QString("Diff:   %1").arg(diff));

            if (diff > ui->surveyInMaxDiffBox->value()) {
                QMessageBox::warning(this, "Base station",
                                     QString("Base seems to have moved %1 m, "
                                             "updating base station position.")
                                     .arg(diff));


                ui->refSendLatBox->setValue(llh[0]);
                ui->refSendLonBox->setValue(llh[1]);
                ui->refSendHBox->setValue(llh[2]);
            }
        } else {
            ui->refSendLatBox->setValue(llh[0]);
            ui->refSendLonBox->setValue(llh[1]);
            ui->refSendHBox->setValue(llh[2]);
            mBasePosSet = true;
        }
    }
}

void BaseStation::on_ubxSerialRefreshButton_clicked()
{
    ui->ubxSerialPortBox->clear();

    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    foreach(const QSerialPortInfo &port, ports) {
        if (port.manufacturer().toLower().replace("-", "").contains("ublox")) {
            ui->ubxSerialPortBox->insertItem(0, "Ublox - " + port.portName(), port.systemLocation());
        } else {
            ui->ubxSerialPortBox->addItem(port.portName(), port.systemLocation());
        }
    }

    ui->ubxSerialPortBox->setCurrentIndex(0);
}

void BaseStation::on_ubxSerialDisconnectButton_clicked()
{
    mUblox->disconnectSerial();
}

void BaseStation::on_ubxSerialConnectButton_clicked()
{
    bool res = mUblox->connectSerial(ui->ubxSerialPortBox->currentData().toString(),
                                     ui->ubxSerialBaudBox->value());

    if (res) {
        // Serial port baud rate
        // if it is too low the buffer will overfill and it won't work properly.
        ubx_cfg_prt_uart uart;
        uart.baudrate = 115200;
        uart.in_ubx = true;
        uart.in_nmea = true;
        uart.in_rtcm2 = false;
        uart.in_rtcm3 = true;
        uart.out_ubx = true;
        uart.out_nmea = true;
        uart.out_rtcm3 = true;
        mUblox->ubxCfgPrtUart(&uart);

        // Set configuration
        // Switch on RAWX and NMEA messages, set rate to 1 Hz and time reference to UTC
        mUblox->ubxCfgRate(1000, 1, 0);
        mUblox->ubxCfgMsg(UBX_CLASS_RXM, UBX_RXM_RAWX, 1); // Every second
        mUblox->ubxCfgMsg(UBX_CLASS_RXM, UBX_RXM_SFRBX, 1); // Every second
        mUblox->ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_SOL, 1); // Every second

        // Stationary dynamic model
        ubx_cfg_nav5 nav5;
        memset(&nav5, 0, sizeof(ubx_cfg_nav5));
        nav5.apply_dyn = true;
        nav5.dyn_model = 2;
        mUblox->ubxCfgNav5(&nav5);

        // Time pulse configuration
        ubx_cfg_tp5 tp5;
        memset(&tp5, 0, sizeof(ubx_cfg_tp5));
        tp5.active = true;
        tp5.polarity = true;
        tp5.alignToTow = true;
        tp5.lockGnssFreq = true;
        tp5.lockedOtherSet = true;
        tp5.syncMode = false;
        tp5.isFreq = false;
        tp5.isLength = true;
        tp5.freq_period = 1000000;
        tp5.pulse_len_ratio = 0;
        tp5.freq_period_lock = 1000000;
        tp5.pulse_len_ratio_lock = 100000;
        tp5.gridUtcGnss = 0;
        tp5.user_config_delay = 0;
        tp5.rf_group_delay = 0;
        tp5.ant_cable_delay = 50;
        mUblox->ubloxCfgTp5(&tp5);

        // Save everything
        ubx_cfg_cfg cfg;
        memset(&cfg, 0, sizeof(ubx_cfg_cfg));
        cfg.save_io_port = true;
        cfg.save_msg_conf = true;
        cfg.save_inf_msg = true;
        cfg.save_nav_conf = true;
        cfg.save_rxm_conf = true;
        cfg.save_sen_conf = true;
        cfg.save_rinv_conf = true;
        cfg.save_ant_conf = true;
        cfg.save_log_conf = true;
        cfg.save_fts_conf = true;
        cfg.dev_bbr = true;
        cfg.dev_flash = true;
        mUblox->ubloxCfgCfg(&cfg);
    }
}

void BaseStation::on_tcpServerBox_toggled(bool checked)
{
    if (checked) {
        if (!mTcpServer->startTcpServer(ui->tcpServerPortBox->value())) {
            QMessageBox::warning(this, "TCP Server Error",
                                 "Creating TCP server for RTCM data failed. Make sure that the port is not "
                                 "already in use.");
            ui->tcpServerBox->setChecked(false);
        }
    } else {
        mTcpServer->stopServer();
    }
}
