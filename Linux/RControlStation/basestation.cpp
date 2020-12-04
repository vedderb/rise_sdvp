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

#define RTCM_REF_MSG_DELAY_MULT 5

BaseStation::BaseStation(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::BaseStation)
{
    ui->setupUi(this);

    mMap = 0;
    mUblox = new Ublox(this);
    mTcpServer = new TcpBroadcast(this);
    mBasePosSet = false;

    mTimer = new QTimer(this);
    mTimer->start(20);

    connect(mTimer, SIGNAL(timeout()),
            this, SLOT(timerSlot()));
    connect(mUblox, SIGNAL(rxRawx(ubx_rxm_rawx)),
            this, SLOT(rxRawx(ubx_rxm_rawx)));
    connect(mUblox, SIGNAL(rxNavSol(ubx_nav_sol)),
            this, SLOT(rxNavSol(ubx_nav_sol)));
    connect(mUblox, SIGNAL(rxNavSat(ubx_nav_sat)),
            this, SLOT(rxNavSat(ubx_nav_sat)));
    connect(mUblox, SIGNAL(rxSvin(ubx_nav_svin)),
            this, SLOT(rxSvin(ubx_nav_svin)));
    connect(mUblox, SIGNAL(rtcmRx(QByteArray,int)),
            this, SLOT(rtcmRx(QByteArray,int)));

    connect(mUblox, &Ublox::rxMonVer, [this]
            (QString sw, QString hw, QStringList extensions) {
        QString txt = "SW: " + sw + "\nHW: " +
                hw + "\nExtensions:\n";

        for (QString s: extensions) {
            txt += s + "\n";
        }

        QMessageBox::information(this, "Ublox Version",
                                 txt.mid(0, txt.size() - 1));
    });

    connect(mUblox, &Ublox::rxCfgGnss, [this](ubx_cfg_gnss cfg) {
        QString str = QString("TrkChHw   : %1\n"
                              "TrkChUse  : %2\n"
                              "Blocks    : %3\n\n").
                arg(cfg.num_ch_hw).arg(cfg.num_ch_use).arg(cfg.num_blocks);

        for (int i = 0;i < cfg.num_blocks;i++) {
            str += QString("GNSS ID: %1, Enabled: %2\n"
                           "MinTrkCh  : %3\n"
                           "MaxTrkCh  : %4\n"
                           "Flags     : %5").
                    arg(cfg.blocks[i].gnss_id).
                    arg(cfg.blocks[i].en).
                    arg(cfg.blocks[i].minTrkCh).
                    arg(cfg.blocks[i].maxTrkCh).
                    arg(cfg.blocks[i].flags);

            if (i != cfg.num_blocks - 1) {
                str += "\n\n";
            }
        }

        QMessageBox::information(this, "Cfg GNSS", str);
    });

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
            mRtcmUbx[1002]++;
            mRtcmUbx[1010]++;
        } else {
            if (has_gps) {
                emit rtcmOut(QByteArray((char*)data_gps, gps_len));
                mRtcmUbx[1002]++;
            }

            if (has_glo) {
                emit rtcmOut(QByteArray((char*)data_glo, glo_len));
                mRtcmUbx[1010]++;
            }
        }

        // Send base station position every RTCM_REF_MSG_DELAY_MULT cycles to save bandwidth.
        static int basePosCnt = 0;
        basePosCnt++;
        if (basePosCnt >= RTCM_REF_MSG_DELAY_MULT) {
            basePosCnt = 0;

            if (has_ref) {
                emit rtcmOut(QByteArray((char*)data_ref, ref_len));
                mRtcmUbx[1006]++;
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

    if (sol.p_acc < ui->surveyInMinAccBox->value() && ui->surveyInRadioButton->isChecked()) {
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

void BaseStation::rxNavSat(ubx_nav_sat sat)
{
    int satsGps = 0;
    int satsGlo = 0;
    int satsGal = 0;
    int satsBds = 0;

    int visibleGps = 0;
    int visibleGlo = 0;
    int visibleGal = 0;
    int visibleBds = 0;

    for (int i = 0;i < sat.num_sv;i++) {
        ubx_nav_sat_info s = sat.sats[i];

        if (s.gnss_id == 0) {
            visibleGps++;
        } else if (s.gnss_id == 2) {
            visibleGal++;
        } else if (s.gnss_id == 3) {
            visibleBds++;
        } else if (s.gnss_id == 6) {
            visibleGlo++;
        }

        if (s.used && s.quality >= 4) {
            if (s.gnss_id == 0) {
                satsGps++;
            } else if (s.gnss_id == 2) {
                satsGal++;
            } else if (s.gnss_id == 3) {
                satsBds++;
            } else if (s.gnss_id == 6) {
                satsGlo++;
            }
        }
    }

    QString rtcmMsgs;
    QMapIterator<int, int> i(mRtcmUbx);
    while (i.hasNext()) {
        i.next();
        if (!rtcmMsgs.isEmpty()) {
            rtcmMsgs += ", ";
        }

        rtcmMsgs += QString("%1:%2").
                arg(i.key()).arg(i.value());
    }

    QString txt = QString("         Visible   Used\n"
                          "GPS:     %1        %5\n"
                          "GLONASS: %2        %6\n"
                          "Galileo: %3        %7\n"
                          "BeiDou:  %4        %8\n"
                          "Total:   %9        %10\n\n"
                          "RTCM Sent:\n"
                          + rtcmMsgs).
            arg(visibleGps, -2).arg(visibleGlo, -2).
            arg(visibleGal, -2).arg(visibleBds, -2).
            arg(satsGps, -2).arg(satsGlo, -2).
            arg(satsGal, -2).arg(satsBds, -2).
            arg(visibleGps + visibleGlo + visibleGal + visibleBds, -2).
            arg(satsGps + satsGlo + satsGal + satsBds, -2);

    ui->ubxText2->clear();
    ui->ubxText2->appendPlainText(txt);
}

void BaseStation::rxSvin(ubx_nav_svin svin)
{
    double llh[3];
    utility::xyzToLlh(svin.meanX, svin.meanY, svin.meanZ,
                      &llh[0], &llh[1], &llh[2]);

    if (ui->surveyInRadioButton->isChecked()) {
        ui->refSendLatBox->setValue(llh[0]);
        ui->refSendLonBox->setValue(llh[1]);
        ui->refSendHBox->setValue(llh[2]);
    }

    QString txt = QString(
                "Lat:          %1\n"
                "Lon:          %2\n"
                "Height:       %3\n"
                "Observarions: %4\n"
                "P ACC:        %5 m\n"
                "Duration:     %6 s\n"
                "Valid:        %7\n"
                "Active:       %8").
            arg(llh[0], 0, 'f', 8).
            arg(llh[1], 0, 'f', 8).
            arg(llh[2]).
            arg(svin.obs).
            arg(svin.meanAcc).
            arg(svin.dur).
            arg(svin.valid).
            arg(svin.active);

    ui->ubxText->clear();
    ui->ubxText->appendPlainText(txt);

    if (mMap && ui->ubxPlotMapBox->isChecked()) {
        double i_llh[3];

        mMap->getEnuRef(i_llh);

        double xyz[3];
        utility::llhToEnu(i_llh, llh, xyz);

        LocPoint p;
        p.setXY(xyz[0], xyz[1]);
        mMap->addInfoPoint(p);

        if (ui->ubxFollowMapBox->isChecked()) {
            mMap->moveView(p.getX(), p.getY());
        }
    }
}

void BaseStation::rtcmRx(QByteArray data, int type)
{
    // Send base station position every RTCM_REF_MSG_DELAY_MULT cycles to save bandwidth.
    static int basePosCnt = 0;
    if (type == 1006 || type == 1005) {
        basePosCnt++;
        if (basePosCnt < RTCM_REF_MSG_DELAY_MULT)
            return;
        basePosCnt = 0;
    }

    mRtcmUbx[type]++;
    emit rtcmOut(data);

    if (ui->tcpServerBox->isChecked()) {
        mTcpServer->broadcastData(data);
    }
}

void BaseStation::on_ubxSerialRefreshButton_clicked()
{
    ui->ubxSerialPortBox->clear();

    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    foreach(const QSerialPortInfo &port, ports) {
        if (port.manufacturer().toLower().replace("-", "").contains("ublox")) {
            ui->ubxSerialPortBox->insertItem(0, "Ublox - " + port.portName(),
                                             port.systemLocation());
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

// TODO: refactor! :-/
void BaseStation::configureUbx(Ublox* ublox, unsigned int baudrate, int rate_meas, int rate_nav, bool isF9p, bool isM8p, bool* basePosSet, double refSendLat, double refSendLon, double refSendH, BaseStationPositionMode positionMode, double surveyInMinAcc, int surveyInMinDuration)
{
    // Serial port baud rate
    // if it is too low the buffer will overfill and it won't work properly.
    ubx_cfg_prt_uart uart;
    uart.baudrate = baudrate;
    uart.in_ubx = true;
    uart.in_nmea = true;
    uart.in_rtcm2 = false;
    uart.in_rtcm3 = true;
    uart.out_ubx = true;
    uart.out_nmea = true;
    uart.out_rtcm3 = true;
/*    qDebug() << "CfgPrtUart:" << */ublox->ubxCfgPrtUart(&uart);

    ublox->ubxCfgRate(rate_meas, rate_nav, 0);

    bool encodeRaw = !isF9p && !isM8p;

    if (positionMode == MOVING_BASE) {
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1005, 0);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1074, 1);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1077, 0);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1084, 1);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1087, 0);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1094, 1);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1097, 0);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1124, 1);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1127, 0);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1230, 1);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_4072_0, 1);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_4072_1, 0);
        ublox->ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_SVIN, 0);
    } else {
        ublox->ubxCfgMsg(UBX_CLASS_RXM, UBX_RXM_RAWX, encodeRaw ? 1 : 0);
        ublox->ubxCfgMsg(UBX_CLASS_RXM, UBX_RXM_SFRBX, encodeRaw ? 1 : 0);
        ublox->ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_SOL, encodeRaw ? 1 : 0);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1005, encodeRaw ? 0 : 1);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1074, encodeRaw ? 0 : 1);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1077, encodeRaw ? 0 : 1);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1084, encodeRaw ? 0 : 1);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1087, encodeRaw ? 0 : 1);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1094, encodeRaw ? 0 : 1);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1097, encodeRaw ? 0 : 1);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1124, encodeRaw ? 0 : 1);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1127, encodeRaw ? 0 : 1);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1230, encodeRaw ? 0 : 1);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_4072_0, encodeRaw ? 0 : 1);
        ublox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_4072_1, encodeRaw ? 0 : 1);

        ublox->ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_SVIN, encodeRaw ? 0 : 1);
}

    ublox->ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_SAT, 1);

    if (isF9p) {
        unsigned char buffer[512];
        int ind = 0;
        ublox->ubloxCfgAppendEnableGps(buffer, &ind, true, true, true);
        ublox->ubloxCfgAppendEnableGal(buffer, &ind, true, true, true);
        ublox->ubloxCfgAppendEnableBds(buffer, &ind, true, true, true);
        ublox->ubloxCfgAppendEnableGlo(buffer, &ind, true, true, true);
        ublox->ubloxCfgValset(buffer, ind, true, true, true);

//          NOTE: set baudrate STM <-> UBX over USB, should not be necessary
//            ind = 0;
//            ublox->ubloxCfgAppendUart1Baud(buffer, &ind, 115200);
//            ublox->ubloxCfgAppendUart1InProt(buffer, &ind, true, true, true);
//            ublox->ubloxCfgAppendUart1OutProt(buffer, &ind, true, true, true);
//            qDebug() << "UBX UART1 Config:" << ublox->ubloxCfgValset(buffer, ind, true, true, true);
    } else {
        ubx_cfg_nmea nmea;
        memset(&nmea, 0, sizeof(ubx_cfg_nmea));
        nmea.nmeaVersion = 0x41;
        nmea.numSv = 0;
        nmea.highPrec = true;

        qDebug() << "CFG NMEA:" << ublox->ubloxCfgNmea(&nmea);

        ubx_cfg_gnss gnss;
        gnss.num_ch_hw = 32;
        gnss.num_ch_use = 32;
        gnss.num_blocks = 4;

        gnss.blocks[0].gnss_id = UBX_GNSS_ID_GPS;
        gnss.blocks[0].en = true;
        gnss.blocks[0].minTrkCh = 8;
        gnss.blocks[0].maxTrkCh = 32;
        gnss.blocks[0].flags = UBX_CFG_GNSS_GPS_L1C;

        gnss.blocks[1].gnss_id = UBX_GNSS_ID_GLONASS;
        gnss.blocks[1].en = true;
        gnss.blocks[1].minTrkCh = 8;
        gnss.blocks[1].maxTrkCh = 32;
        gnss.blocks[1].flags = UBX_CFG_GNSS_GLO_L1;

        gnss.blocks[2].gnss_id = UBX_GNSS_ID_BEIDOU;
        gnss.blocks[2].en = false;
        gnss.blocks[2].minTrkCh = 8;
        gnss.blocks[2].maxTrkCh = 16;
        gnss.blocks[2].flags = UBX_CFG_GNSS_BDS_B1L;

        gnss.blocks[3].gnss_id = UBX_GNSS_ID_GALILEO;
        gnss.blocks[3].en = true;
        gnss.blocks[3].minTrkCh = 8;
        gnss.blocks[3].maxTrkCh = 10;
        gnss.blocks[3].flags = UBX_CFG_GNSS_GAL_E1;

        qDebug() << "CFG GNSS:" << ublox->ubloxCfgGnss(&gnss);
    }

    ubx_cfg_tmode3 cfg_mode;
    memset(&cfg_mode, 0, sizeof(cfg_mode));
    if (positionMode == MOVING_BASE)
        cfg_mode.mode = 0;
    else {
        cfg_mode.mode = isF9p ? 1 : 0;
        cfg_mode.fixed_pos_acc = 5.0;
        cfg_mode.svin_min_dur = surveyInMinDuration;
        cfg_mode.svin_acc_limit = surveyInMinAcc;

        if (positionMode == FIXED && isF9p) {
            *basePosSet = true;

            cfg_mode.mode = 2;
            cfg_mode.lla = true;
            cfg_mode.ecefx_lat = refSendLat;
            cfg_mode.ecefy_lon = refSendLon;
            cfg_mode.ecefz_alt = refSendH;
        }
    }
    ublox->ubxCfgTmode3(&cfg_mode);

    // Stationary dynamic model
    ubx_cfg_nav5 nav5;
    memset(&nav5, 0, sizeof(ubx_cfg_nav5));
    nav5.apply_dyn = true;
    nav5.apply_dyn = true;
    nav5.dyn_model = 2;
    ublox->ubxCfgNav5(&nav5);

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
    ublox->ubloxCfgTp5(&tp5);

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
    ublox->ubloxCfgCfg(&cfg);
}

void BaseStation::on_ubxSerialConnectButton_clicked()
{
    bool res = mUblox->connectSerial(ui->ubxSerialPortBox->currentData().toString(),
                                     ui->ubxSerialBaudBox->value());

    if (res) {
        mRtcmUbx.clear();
        ui->ubxText->clear();
        ui->ubxText2->clear();

        bool isM8p = ui->m8PButton->isChecked();
        bool isF9p = ui->f9Button->isChecked();
        if (!isF9p)
            ui->rateMeasBox->setValue(5);

        BaseStationPositionMode positionMode;
        if (ui->surveyInRadioButton->isChecked())
            positionMode = SURVEY_IN;
        else if (ui->movingBaseRadioButton->isChecked())
            positionMode = MOVING_BASE;
        else
            positionMode = FIXED;

        double surveyInMinAcc = ui->surveyInMinAccBox->value();

        double refSendLat = ui->refSendLatBox->value();
        double refSendLon = ui->refSendLonBox->value();
        double refSendH = ui->refSendHBox->value();

        configureUbx(mUblox, ui->ubxSerialBaudBox->value(), 1000/ui->rateMeasBox->value(), ui->rateNavBox->value(), isF9p, isM8p, &mBasePosSet, refSendLat, refSendLon, refSendH, positionMode, surveyInMinAcc, ui->surveyInMinDurationBox->value());
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

void BaseStation::on_readVersionButton_clicked()
{
    mUblox->ubxPoll(UBX_CLASS_MON, UBX_MON_VER);
}

void BaseStation::on_gnssInfoButton_clicked()
{
    mUblox->ubxPoll(UBX_CLASS_CFG, UBX_CFG_GNSS);
}

void BaseStation::on_surveyInRadioButton_toggled(bool checked)
{
    ui->surveyInMinAccBox->setEnabled(checked);
    ui->surveyInMaxDiffBox->setEnabled(checked);
    ui->surveyInMinDurationBox->setEnabled(checked);
}

void BaseStation::on_m8Button_toggled(bool checked)
{
    ui->movingBaseRadioButton->setEnabled(!checked);
    if (checked)
        ui->surveyInRadioButton->setChecked(true);
}

void BaseStation::on_movingBaseRadioButton_toggled(bool checked)
{
    if (checked)
        ui->rateMeasBox->setValue(5);
}
