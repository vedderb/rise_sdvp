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

    mTcpSocket = new QTcpSocket(this);
    mTcpConnected = false;
    mXNow = 0.0;
    mYNow = 0.0;
    mZNow = 0.0;
    mXAvg = 0.0;
    mYAvg = 0.0;
    mZAvg = 0.0;
    mAvgSamples = 0.0;
    mBasePosCnt = 0;

    mFixNowStr = "Solution...";
    mSatNowStr = "Sats...";

    mUblox = new Ublox(this);
    mTcpServer = new TcpBroadcast(this);

    mTimer = new QTimer(this);
    mTimer->start(20);

    connect(mTcpSocket, SIGNAL(readyRead()), this, SLOT(tcpInputDataAvailable()));
    connect(mTcpSocket, SIGNAL(connected()), this, SLOT(tcpInputConnected()));
    connect(mTcpSocket, SIGNAL(disconnected()),
            this, SLOT(tcpInputDisconnected()));
    connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(tcpInputError(QAbstractSocket::SocketError)));
    connect(mTimer, SIGNAL(timeout()),
            this, SLOT(timerSlot()));
    connect(mUblox, SIGNAL(rxGga(int,NmeaServer::nmea_gga_info_t)),
            this, SLOT(rxGga(int,NmeaServer::nmea_gga_info_t)));
    connect(mUblox, SIGNAL(rxRawx(ubx_rxm_rawx)),
            this, SLOT(rxRawx(ubx_rxm_rawx)));

    updateNmeaText();
    on_ubxSerialRefreshButton_clicked();
}

BaseStation::~BaseStation()
{
    delete ui;
}

int BaseStation::getAvgPosLlh(double &lat, double &lon, double &height)
{
    double xAvg = mXAvg / mAvgSamples;
    double yAvg = mYAvg / mAvgSamples;
    double zAvg = mZAvg / mAvgSamples;

    utility::xyzToLlh(xAvg, yAvg, zAvg, &lat, &lon, &height);

    return (int)mAvgSamples;
}

void BaseStation::tcpInputConnected()
{
    ui->nmeaConnectButton->setEnabled(true);
    ui->nmeaConnectButton->setToolTip("Disconnect");
    ui->nmeaConnectButton->setText("");
    ui->nmeaConnectButton->setIcon(QIcon(":/models/Icons/Disconnected-96.png"));
    mTcpConnected = true;
}

void BaseStation::tcpInputDisconnected()
{
    ui->nmeaConnectButton->setEnabled(true);
    ui->nmeaConnectButton->setToolTip("Connect");
    ui->nmeaConnectButton->setText("");
    ui->nmeaConnectButton->setIcon(QIcon(":/models/Icons/Connected-96.png"));
    mTcpConnected = false;
}

void BaseStation::tcpInputDataAvailable()
{
    QByteArray nmea_msg =  mTcpSocket->readAll();

    NmeaServer::nmea_gga_info_t gga;
    QTextStream msgs(nmea_msg);

    while(!msgs.atEnd()) {
        QString str = msgs.readLine();
        QByteArray data = str.toLocal8Bit();

        int fields = NmeaServer::decodeNmeaGGA(data, gga);
        rxGga(fields, gga);
    }
}

void BaseStation::tcpInputError(QAbstractSocket::SocketError socketError)
{
    (void)socketError;

    QString errorStr = mTcpSocket->errorString();
    qWarning() << "TcpError:" << errorStr;
    QMessageBox::warning(this, "BaseStation TCP Error", errorStr);

    mTcpSocket->close();

    ui->nmeaConnectButton->setEnabled(true);
    ui->nmeaConnectButton->setToolTip("Connect");
    ui->nmeaConnectButton->setText("");
    ui->nmeaConnectButton->setIcon(QIcon(":/models/Icons/Connected-96.png"));
    mTcpConnected = false;
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

void BaseStation::rxGga(int fields, NmeaServer::nmea_gga_info_t gga)
{
    if (fields >= 2) {
        mSatNowStr.sprintf("Satellites: %d", gga.n_sat);

        //qDebug() << data;
        //qDebug() << QString().sprintf("%.9f", gga.lat);

        switch (gga.fix_type) {
        case 0: mFixNowStr = "Solution: Invalid"; break;
        case 1: mFixNowStr = "Solution: SPP"; break;
        case 2: mFixNowStr = "Solution: DGPS"; break;
        case 3: mFixNowStr = "Solution: PPS"; break;
        case 4: mFixNowStr = "Solution: RTK Fix"; break;
        case 5: mFixNowStr = "Solution: RTK Float"; break;
        default: mFixNowStr = "Solution: Unknown"; break;
        }

        if (gga.fix_type == 1 || gga.fix_type == 2 || gga.fix_type == 4 || gga.fix_type == 5) {
            utility::llhToXyz(gga.lat, gga.lon, gga.height, &mXNow, &mYNow, &mZNow);
            mXAvg += mXNow;
            mYAvg += mYNow;
            mZAvg += mZNow;
            mAvgSamples += 1.0;
        }
    } else {
        mFixNowStr = "Solution: Invalid";
        mSatNowStr = "Satellites: 0";
    }

    updateNmeaText();
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

void BaseStation::on_nmeaConnectButton_clicked()
{
    if (mTcpConnected) {
        mTcpSocket->abort();

        ui->nmeaConnectButton->setEnabled(true);
        ui->nmeaConnectButton->setToolTip("Connect");
        ui->nmeaConnectButton->setText("");
        ui->nmeaConnectButton->setIcon(QIcon(":/models/Icons/Connected-96.png"));
        mTcpConnected = false;
    } else {
        mTcpSocket->abort();
        mTcpSocket->connectToHost(ui->nmeaServerEdit->text(), ui->nmeaPortBox->value());

        ui->nmeaConnectButton->setEnabled(false);
        ui->nmeaConnectButton->setText("Connecting...");
    }
}

void BaseStation::on_nmeaSampleClearButton_clicked()
{
    mXAvg = 0.0;
    mYAvg = 0.0;
    mZAvg = 0.0;
    mAvgSamples = 0.0;

    updateNmeaText();
}

void BaseStation::updateNmeaText()
{
    QString sampStr;
    sampStr.sprintf("Samples %d", (int)mAvgSamples);
    ui->nmeaSampleLabel->setText(sampStr);

    double xAvg = mXAvg / mAvgSamples;
    double yAvg = mYAvg / mAvgSamples;
    double zAvg = mZAvg / mAvgSamples;

    double lat_now, lon_now, height_now;
    double lat_avg, lon_avg, height_avg;

    utility::xyzToLlh(mXNow, mYNow, mZNow, &lat_now, &lon_now, &height_now);
    utility::xyzToLlh(xAvg, yAvg, zAvg, &lat_avg, &lon_avg, &height_avg);

    QString statStr;
    statStr += mFixNowStr + "\n" + mSatNowStr + "\n\n";
    statStr += QString().sprintf("XYZ Now: %.3f, %.3f, %.3f\n", mXNow, mYNow, mZNow);
    statStr += QString().sprintf("LLH Now: %.8f, %.8f, %.3f\n\n", lat_now, lon_now, height_now);

    statStr += QString().sprintf("XYZ Avg: %.3f, %.3f, %.3f\n", xAvg, yAvg, zAvg);
    statStr += QString().sprintf("LLH Avg: %.8f, %.8f, %.3f", lat_avg, lon_avg, height_avg);

    ui->nmeaBrowser->setText(statStr);
}

void BaseStation::on_ubxSerialRefreshButton_clicked()
{
    ui->ubxSerialPortBox->clear();

    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    foreach(const QSerialPortInfo &port, ports) {
        ui->ubxSerialPortBox->addItem(port.portName(), port.systemLocation());
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
        mUblox->ubxCfgMsg(UBX_CLASS_NMEA, UBX_NMEA_GGA, 1); // Every second

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
    }
}

void BaseStation::on_refGetButton_clicked()
{
    double lat, lon, height;
    if (getAvgPosLlh(lat, lon, height) > 0) {
        ui->refSendLatBox->setValue(lat);
        ui->refSendLonBox->setValue(lon);
        ui->refSendHBox->setValue(height);
    } else {
        QMessageBox::warning(this, "Reference Position",
                             "No samples collected yet.");
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
