/*
    Copyright 2017 - 2018 Benjamin Vedder	benjamin@vedder.se

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

#include "ublox.h"
#include <QEventLoop>
#include <cmath>

namespace {
static uint8_t ubx_get_U1(uint8_t *msg, int *ind) {
    return msg[(*ind)++];
}

static int8_t ubx_get_I1(uint8_t *msg, int *ind) {
    return (int8_t)msg[(*ind)++];
}

static uint8_t ubx_get_X1(uint8_t *msg, int *ind) {
    return msg[(*ind)++];
}

static uint16_t ubx_get_U2(uint8_t *msg, int *ind) {
    uint16_t res =	((uint16_t) msg[*ind + 1]) << 8 |
                    ((uint16_t) msg[*ind]);
    *ind += 2;
    return res;
}

static int16_t ubx_get_I2(uint8_t *msg, int *ind) {
    int16_t res =	((uint16_t) msg[*ind + 1]) << 8 |
                    ((uint16_t) msg[*ind]);
    *ind += 2;
    return res;
}

static uint16_t ubx_get_X2(uint8_t *msg, int *ind) {
    uint16_t res =	((uint16_t) msg[*ind + 1]) << 8 |
                    ((uint16_t) msg[*ind]);
    *ind += 2;
    return res;
}

static uint32_t ubx_get_U4(uint8_t *msg, int *ind) {
    uint32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
                    ((uint32_t) msg[*ind + 2]) << 16 |
                    ((uint32_t) msg[*ind + 1]) << 8 |
                    ((uint32_t) msg[*ind]);
    *ind += 4;
    return res;
}

static int32_t ubx_get_I4(uint8_t *msg, int *ind) {
    int32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
                    ((uint32_t) msg[*ind + 2]) << 16 |
                    ((uint32_t) msg[*ind + 1]) << 8 |
                    ((uint32_t) msg[*ind]);
    *ind += 4;
    return res;
}

static uint32_t ubx_get_X4(uint8_t *msg, int *ind) {
    uint32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
                    ((uint32_t) msg[*ind + 2]) << 16 |
                    ((uint32_t) msg[*ind + 1]) << 8 |
                    ((uint32_t) msg[*ind]);
    *ind += 4;
    return res;
}

static float ubx_get_R4(uint8_t *msg, int *ind) {
    uint32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
                    ((uint32_t) msg[*ind + 2]) << 16 |
                    ((uint32_t) msg[*ind + 1]) << 8 |
                    ((uint32_t) msg[*ind]);
    *ind += 4;

    union asd {
        float f;
        uint32_t i;
    } x;

    x.i = res;

    return x.f;
}

static double ubx_get_R8(uint8_t *msg, int *ind) {
    uint64_t res =	((uint64_t) msg[*ind + 7]) << 56 |
                    ((uint64_t) msg[*ind + 6]) << 48 |
                    ((uint64_t) msg[*ind + 5]) << 40 |
                    ((uint64_t) msg[*ind + 4]) << 32 |
                    ((uint64_t) msg[*ind + 3]) << 24 |
                    ((uint64_t) msg[*ind + 2]) << 16 |
                    ((uint64_t) msg[*ind + 1]) << 8 |
                    ((uint64_t) msg[*ind]);
    *ind += 8;

    union asd {
        double f;
        uint64_t i;
    } x;

    x.i = res;

    return x.f;
}

static void ubx_put_U1(uint8_t *msg, int *ind, uint8_t data) {
    msg[(*ind)++] = data;
}

static void ubx_put_I1(uint8_t *msg, int *ind, int8_t data) {
    msg[(*ind)++] = data;
}

static void ubx_put_X1(uint8_t *msg, int *ind, uint8_t data) {
    msg[(*ind)++] = data;
}

static void ubx_put_U2(uint8_t *msg, int *ind, uint16_t data) {
    msg[(*ind)++] = data;
    msg[(*ind)++] = data >> 8;
}

static void ubx_put_I2(uint8_t *msg, int *ind, int16_t data) {
    msg[(*ind)++] = data;
    msg[(*ind)++] = data >> 8;
}

static void ubx_put_X2(uint8_t *msg, int *ind, uint16_t data) {
    msg[(*ind)++] = data;
    msg[(*ind)++] = data >> 8;
}

static void ubx_put_U4(uint8_t *msg, int *ind, uint32_t data) {
    msg[(*ind)++] = data;
    msg[(*ind)++] = data >> 8;
    msg[(*ind)++] = data >> 16;
    msg[(*ind)++] = data >> 24;
}

static void ubx_put_I4(uint8_t *msg, int *ind, int32_t data) {
    msg[(*ind)++] = data;
    msg[(*ind)++] = data >> 8;
    msg[(*ind)++] = data >> 16;
    msg[(*ind)++] = data >> 24;
}

static void ubx_put_X4(uint8_t *msg, int *ind, uint32_t data) {
    msg[(*ind)++] = data;
    msg[(*ind)++] = data >> 8;
    msg[(*ind)++] = data >> 16;
    msg[(*ind)++] = data >> 24;
}

static void ubx_put_R4(uint8_t *msg, int *ind, float data) {
    union asd {
        float f;
        uint64_t i;
    } x;

    x.f = data;

    msg[(*ind)++] = x.i;
    msg[(*ind)++] = x.i >> 8;
    msg[(*ind)++] = x.i >> 16;
    msg[(*ind)++] = x.i >> 24;
}

static void ubx_put_R8(uint8_t *msg, int *ind, double data) {
    union asd {
        double f;
        uint64_t i;
    } x;

    x.f = data;

    msg[(*ind)++] = x.i;
    msg[(*ind)++] = x.i >> 8;
    msg[(*ind)++] = x.i >> 16;
    msg[(*ind)++] = x.i >> 24;
    msg[(*ind)++] = x.i >> 32;
    msg[(*ind)++] = x.i >> 40;
    msg[(*ind)++] = x.i >> 48;
    msg[(*ind)++] = x.i >> 56;
}
}

Ublox::Ublox(QObject *parent) : QObject(parent)
{
    memset(&mDecoderState, 0, sizeof(decoder_state));
    rtcm3_init_state(&mRtcmState);

    mSerialPort = new QSerialPort(this);

    connect(mSerialPort, SIGNAL(readyRead()), this, SLOT(serialDataAvailable()));
    connect(mSerialPort, SIGNAL(error(QSerialPort::SerialPortError)),
            this, SLOT(serialPortError(QSerialPort::SerialPortError)));

    // Prevent unused warnings
    (void)ubx_get_U1;
    (void)ubx_get_I1;
    (void)ubx_get_X1;
    (void)ubx_get_U2;
    (void)ubx_get_I2;
    (void)ubx_get_X2;
    (void)ubx_get_U4;
    (void)ubx_get_I4;
    (void)ubx_get_X4;
    (void)ubx_get_R4;
    (void)ubx_get_R8;

    (void)ubx_put_U1;
    (void)ubx_put_I1;
    (void)ubx_put_X1;
    (void)ubx_put_U2;
    (void)ubx_put_I2;
    (void)ubx_put_X2;
    (void)ubx_put_U4;
    (void)ubx_put_I4;
    (void)ubx_put_X4;
    (void)ubx_put_R4;
    (void)ubx_put_R8;
}

bool Ublox::connectSerial(QString port, int baudrate)
{
    if(mSerialPort->isOpen()) {
        mSerialPort->close();
    }

    mSerialPort->setPortName(port);
    mSerialPort->open(QIODevice::ReadWrite);
    mWaitingAck = false;

    if(!mSerialPort->isOpen()) {
        return false;
    }

    mSerialPort->setBaudRate(baudrate);
    mSerialPort->setDataBits(QSerialPort::Data8);
    mSerialPort->setParity(QSerialPort::NoParity);
    mSerialPort->setStopBits(QSerialPort::OneStop);
    mSerialPort->setFlowControl(QSerialPort::NoFlowControl);

    return true;
}

void Ublox::disconnectSerial()
{
    mSerialPort->close();
}

bool Ublox::isSerialConnected()
{
    return mSerialPort->isOpen();
}

void Ublox::writeRaw(QByteArray data)
{
    ubx_send(data);
}

void Ublox::ubxPoll(uint8_t msg_class, uint8_t id)
{
    ubx_encode_send(msg_class, id, 0, 0);
}

/**
 * Set the uart1 port configuration.
 *
 * @param cfg
 * The configuration. Notice that always 8N1 configuration
 * and no tx ready function is used.
 *
 * @return
 * true: ack or nak received
 * false: timeout
 */
bool Ublox::ubxCfgPrtUart(ubx_cfg_prt_uart *cfg)
{
    uint8_t buffer[20];
    int ind = 0;

    ubx_put_U1(buffer, &ind, 1); // ID for UART1
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_X2(buffer, &ind, 0); // Always disable txready function

    uint32_t mode = 0;
    mode |= 3 << 6; // Always use 8 bits
    mode |= 4 << 9; // No parity
    mode |= 0 << 12; // 1 stop bit

    ubx_put_X4(buffer, &ind, mode);
    ubx_put_U4(buffer, &ind, cfg->baudrate);

    uint16_t in_proto = 0;
    in_proto |= (cfg->in_ubx ? 1 : 0) << 0;
    in_proto |= (cfg->in_nmea ? 1 : 0) << 1;
    in_proto |= (cfg->in_rtcm2 ? 1 : 0) << 2;
    in_proto |= (cfg->in_rtcm3 ? 1 : 0) << 5;

    ubx_put_X2(buffer, &ind, in_proto);

    uint16_t out_proto = 0;
    out_proto |= (cfg->out_ubx ? 1 : 0) << 0;
    out_proto |= (cfg->out_nmea ? 1 : 0) << 1;
    out_proto |= (cfg->out_rtcm3 ? 1 : 0) << 5;

    ubx_put_X2(buffer, &ind, out_proto);
    ubx_put_X2(buffer, &ind, 0); // No extended timeout
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_PRT, buffer, ind, 10);
}

/**
 * Set the tmode3 configuration.
 *
 * @param cfg
 * The configuration.
 *
 * @return
 * true: ack or nak received
 * false: timeout
 */
bool Ublox::ubxCfgTmode3(ubx_cfg_tmode3 *cfg)
{
    uint8_t buffer[40];
    int ind = 0;

    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    uint16_t flags = ((cfg->lla ? 1 : 0) << 8) | cfg->mode;
    ubx_put_X2(buffer, &ind, flags);

    int32_t x_lat = 0;
    int32_t y_lon = 0;
    int32_t z_alt = 0;
    int8_t x_lat_hp = 0;
    int8_t y_lon_hp = 0;
    int8_t z_alt_hp = 0;
    if (cfg->lla) {
        x_lat = round(cfg->ecefx_lat * D(1e7));
        y_lon = round(cfg->ecefy_lon * D(1e7));
        z_alt = round(cfg->ecefz_alt * D(1e2));
        x_lat_hp = ((cfg->ecefx_lat - ((double)x_lat * D(1e-7))) * D(1e9));
        y_lon_hp = ((cfg->ecefy_lon - ((double)y_lon * D(1e-7))) * D(1e9));
        z_alt_hp = ((cfg->ecefz_alt - ((double)z_alt * D(1e-2))) * D(1e4));
    } else {
        x_lat = cfg->ecefx_lat * D(1e2);
        y_lon = cfg->ecefy_lon * D(1e2);
        z_alt = cfg->ecefz_alt * D(1e2);
        x_lat_hp = ((cfg->ecefx_lat - ((double)x_lat * D(1e-2))) * D(1e4));
        y_lon_hp = ((cfg->ecefy_lon - ((double)y_lon * D(1e-2))) * D(1e4));
        z_alt_hp = ((cfg->ecefz_alt - ((double)z_alt * D(1e-2))) * D(1e4));
    }

    ubx_put_I4(buffer, &ind, x_lat);
    ubx_put_I4(buffer, &ind, y_lon);
    ubx_put_I4(buffer, &ind, z_alt);
    ubx_put_I1(buffer, &ind, x_lat_hp);
    ubx_put_I1(buffer, &ind, y_lon_hp);
    ubx_put_I1(buffer, &ind, z_alt_hp);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U4(buffer, &ind, cfg->fixed_pos_acc * 1e4);
    ubx_put_U4(buffer, &ind, cfg->svin_min_dur);
    ubx_put_U4(buffer, &ind, cfg->svin_acc_limit * 1e4);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_TMODE3, buffer, ind, 10);
}

/**
 * Set the msg output configuration.
 *
 * @param msg_class
 * The message class.
 *
 * @param id
 * The message id
 *
 * @param
 * The message rate. 0 = disbaled
 *
 * @return
 * true: ack or nak received
 * false: timeout
 */
bool Ublox::ubxCfgMsg(uint8_t msg_class, uint8_t id, uint8_t rate)
{
    uint8_t buffer[8];
    int ind = 0;

    ubx_put_U1(buffer, &ind, msg_class);
    ubx_put_U1(buffer, &ind, id);
    ubx_put_U1(buffer, &ind, rate);
    ubx_put_U1(buffer, &ind, rate);
    ubx_put_U1(buffer, &ind, rate);
    ubx_put_U1(buffer, &ind, rate);
    ubx_put_U1(buffer, &ind, rate);
    ubx_put_U1(buffer, &ind, rate);

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_MSG, buffer, ind, 10);
}

/**
 * Set the measurement rate, navigation rate and time reference.
 *
 * @param meas_rate_ms
 * The elapsed time between GNSS measurements, which defines the rate,
 * e.g. 100ms => 10Hz, 1000ms => 1Hz, 10000ms => 0.1Hz
 *
 * @param nav_rate_ms
 * The ratio between the number of measurements and the number of navigation
 * solutions, e.g. 5 means five measurements for every navigation solution.
 * Max. value is 127. (This parameter is ignored and the navRate is fixed to 1
 * in protocol versions less than 18)
 *
 * @param time_ref
 * The time system to which measurements are aligned:
 * 0: UTC time
 * 1: GPS time
 * 2: GLONASS time (not supported in protocol versions less than 18)
 * 3: BeiDou time (not supported in protocol versions less than 18)
 * 4: Galileo time (not supported in protocol versions less than 18)
 *
 * @return
 * true: ack or nak received
 * false: timeout
 */
bool Ublox::ubxCfgRate(uint16_t meas_rate_ms, uint16_t nav_rate_ms, uint16_t time_ref)
{
    uint8_t buffer[6];
    int ind = 0;

    ubx_put_U2(buffer, &ind, meas_rate_ms);
    ubx_put_U2(buffer, &ind, nav_rate_ms);
    ubx_put_U2(buffer, &ind, time_ref);

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_RATE, buffer, ind, 10);
}

bool Ublox::ubloxCfgCfg(ubx_cfg_cfg *cfg)
{
    uint8_t buffer[13];
    int ind = 0;

    uint32_t clear = 0;
    clear |= (cfg->clear_io_port ? 1 : 0) << 0;
    clear |= (cfg->clear_msg_conf ? 1 : 0) << 1;
    clear |= (cfg->clear_inf_msg ? 1 : 0) << 2;
    clear |= (cfg->clear_nav_conf ? 1 : 0) << 3;
    clear |= (cfg->clear_rxm_conf ? 1 : 0) << 4;
    clear |= (cfg->clear_sen_conf ? 1 : 0) << 8;
    clear |= (cfg->clear_rinv_conf ? 1 : 0) << 9;
    clear |= (cfg->clear_ant_conf ? 1 : 0) << 10;
    clear |= (cfg->clear_log_conf ? 1 : 0) << 11;
    clear |= (cfg->clear_fts_conf ? 1 : 0) << 12;

    uint32_t save = 0;
    save |= (cfg->save_io_port ? 1 : 0) << 0;
    save |= (cfg->save_msg_conf ? 1 : 0) << 1;
    save |= (cfg->save_inf_msg ? 1 : 0) << 2;
    save |= (cfg->save_nav_conf ? 1 : 0) << 3;
    save |= (cfg->save_rxm_conf ? 1 : 0) << 4;
    save |= (cfg->save_sen_conf ? 1 : 0) << 8;
    save |= (cfg->save_rinv_conf ? 1 : 0) << 9;
    save |= (cfg->save_ant_conf ? 1 : 0) << 10;
    save |= (cfg->save_log_conf ? 1 : 0) << 11;
    save |= (cfg->save_fts_conf ? 1 : 0) << 12;

    uint32_t load = 0;
    load |= (cfg->load_io_port ? 1 : 0) << 0;
    load |= (cfg->load_msg_conf ? 1 : 0) << 1;
    load |= (cfg->load_inf_msg ? 1 : 0) << 2;
    load |= (cfg->load_nav_conf ? 1 : 0) << 3;
    load |= (cfg->load_rxm_conf ? 1 : 0) << 4;
    load |= (cfg->load_sen_conf ? 1 : 0) << 8;
    load |= (cfg->load_rinv_conf ? 1 : 0) << 9;
    load |= (cfg->load_ant_conf ? 1 : 0) << 10;
    load |= (cfg->load_log_conf ? 1 : 0) << 11;
    load |= (cfg->load_fts_conf ? 1 : 0) << 12;

    uint8_t device = 0;
    device |= (cfg->dev_bbr ? 1 : 0) << 0;
    device |= (cfg->dev_flash ? 1 : 0) << 1;
    device |= (cfg->dev_eeprom ? 1 : 0) << 2;
    device |= (cfg->dev_spi_flash ? 1 : 0) << 4;

    ubx_put_X4(buffer, &ind, clear);
    ubx_put_X4(buffer, &ind, save);
    ubx_put_X4(buffer, &ind, load);
    ubx_put_X1(buffer, &ind, device);

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_CFG, buffer, ind, 10);
}

/**
 * Set the nav5 configuration.
 *
 * @param cfg
 * The configuration.
 *
 * @return
 * true: ack or nak received
 * false: timeout
 */
bool Ublox::ubxCfgNav5(ubx_cfg_nav5 *cfg)
{
    uint8_t buffer[36];
    int ind = 0;

    uint16_t mask = 0;
    mask |= (cfg->apply_dyn ? 1 : 0) << 0;
    mask |= (cfg->apply_min_el ? 1 : 0) << 1;
    mask |= (cfg->apply_pos_fix_mode ? 1 : 0) << 2;
    mask |= (cfg->apply_pos_mask ? 1 : 0) << 4;
    mask |= (cfg->apply_time_mask ? 1 : 0) << 5;
    mask |= (cfg->apply_static_hold_mask ? 1 : 0) << 6;
    mask |= (cfg->apply_dgps ? 1 : 0) << 7;
    mask |= (cfg->apply_cno ? 1 : 0) << 8;
    mask |= (cfg->apply_utc ? 1 : 0) << 10;

    ubx_put_X2(buffer, &ind, mask);
    ubx_put_U1(buffer, &ind, cfg->dyn_model);
    ubx_put_U1(buffer, &ind, cfg->fix_mode);
    ubx_put_I4(buffer, &ind, cfg->fixed_alt * D(100.0));
    ubx_put_U4(buffer, &ind, cfg->fixed_alt_var * D(10000.0));
    ubx_put_I1(buffer, &ind, cfg->min_elev);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U2(buffer, &ind, cfg->p_dop * 10.0);
    ubx_put_U2(buffer, &ind, cfg->t_dop * 10.0);
    ubx_put_U2(buffer, &ind, cfg->p_acc);
    ubx_put_U2(buffer, &ind, cfg->t_acc);
    ubx_put_U1(buffer, &ind, cfg->static_hold_thres);
    ubx_put_U1(buffer, &ind, cfg->dgnss_timeout);
    ubx_put_U1(buffer, &ind, cfg->cno_tres_num_sat);
    ubx_put_U1(buffer, &ind, cfg->cno_tres);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U2(buffer, &ind, cfg->static_hold_max_dist);
    ubx_put_U1(buffer, &ind, cfg->utc_standard);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_NAV5, buffer, ind, 10);
}

bool Ublox::ubloxCfgTp5(ubx_cfg_tp5 *cfg)
{
    uint8_t buffer[32];
    int ind = 0;

    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 1);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_I2(buffer, &ind, cfg->ant_cable_delay);
    ubx_put_I2(buffer, &ind, cfg->rf_group_delay);
    ubx_put_U4(buffer, &ind, cfg->freq_period);
    ubx_put_U4(buffer, &ind, cfg->freq_period_lock);
    ubx_put_U4(buffer, &ind, cfg->pulse_len_ratio);
    ubx_put_U4(buffer, &ind, cfg->pulse_len_ratio_lock);
    ubx_put_I4(buffer, &ind, cfg->user_config_delay);

    uint32_t mask = 0;
    mask |= (cfg->active ? 1 : 0) << 0;
    mask |= (cfg->lockGnssFreq ? 1 : 0) << 1;
    mask |= (cfg->lockedOtherSet ? 1 : 0) << 2;
    mask |= (cfg->isFreq ? 1 : 0) << 3;
    mask |= (cfg->isLength ? 1 : 0) << 4;
    mask |= (cfg->alignToTow ? 1 : 0) << 5;
    mask |= (cfg->polarity ? 1 : 0) << 6;
    mask |= (cfg->gridUtcGnss & 0b1111) << 7;
    mask |= (cfg->syncMode & 0b111) << 8;

    ubx_put_X4(buffer, &ind, mask);

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_TP5, buffer, ind, 10);
}

bool Ublox::ubloxCfgGnss(ubx_cfg_gnss *gnss)
{
    if (gnss->num_blocks > 10) {
        return false;
    }

    uint8_t buffer[4 + 8 * gnss->num_blocks];
    int ind = 0;

    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, gnss->num_ch_hw);
    ubx_put_U1(buffer, &ind, gnss->num_ch_use);
    ubx_put_U1(buffer, &ind, gnss->num_blocks);

    for (int i = 0;i < gnss->num_blocks;i++) {
        ubx_put_U1(buffer, &ind, gnss->blocks[i].gnss_id);
        ubx_put_U1(buffer, &ind, gnss->blocks[i].minTrkCh);
        ubx_put_U1(buffer, &ind, gnss->blocks[i].maxTrkCh);
        ubx_put_U1(buffer, &ind, 0);
        uint32_t flags = gnss->blocks[i].en ? 1 : 0;
        flags |= gnss->blocks[i].flags << 16;
        ubx_put_X4(buffer, &ind, flags);
    }

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_GNSS, buffer, ind, 100);
}

bool Ublox::ubloxCfgNmea(ubx_cfg_nmea *nmea)
{
    uint8_t buffer[20];
    int ind = 0;

    uint8_t filter = 0;
    filter |= (nmea->posFilt ? 1 : 0) << 0;
    filter |= (nmea->mskPosFilt ? 1 : 0) << 1;
    filter |= (nmea->timeFilt ? 1 : 0) << 2;
    filter |= (nmea->dateFilt ? 1 : 0) << 3;
    filter |= (nmea->gpsOnlyFilt ? 1 : 0) << 4;
    filter |= (nmea->trackFilt ? 1 : 0) << 5;
    ubx_put_X1(buffer, &ind, filter);

    ubx_put_U1(buffer, &ind, nmea->nmeaVersion);
    ubx_put_U1(buffer, &ind, nmea->numSv);

    uint8_t flags = 0;
    flags |= (nmea->compat ? 1 : 0) << 0;
    flags |= (nmea->consider ? 1 : 0) << 1;
    flags |= (nmea->limit82 ? 1 : 0) << 2;
    flags |= (nmea->highPrec ? 1 : 0) << 3;
    ubx_put_X1(buffer, &ind, flags);

    uint32_t gnss_filter = 0;
    gnss_filter |= (nmea->disableGps ? 1 : 0) << 0;
    gnss_filter |= (nmea->disableSbas ? 1 : 0) << 1;
    gnss_filter |= (nmea->disableQzss ? 1 : 0) << 4;
    gnss_filter |= (nmea->disableGlonass ? 1 : 0) << 5;
    gnss_filter |= (nmea->disableBeidou ? 1 : 0) << 6;
    ubx_put_X4(buffer, &ind, gnss_filter);

    ubx_put_U1(buffer, &ind, nmea->svNumbering);
    ubx_put_U1(buffer, &ind, nmea->mainTalkerId);
    ubx_put_U1(buffer, &ind, nmea->gsvTalkerId);
    ubx_put_U1(buffer, &ind, 1);
    ubx_put_I1(buffer, &ind, nmea->bdsTalkerId[0]);
    ubx_put_I1(buffer, &ind, nmea->bdsTalkerId[1]);

    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_NMEA, buffer, ind, 100);
}

bool Ublox::ubloxCfgValset(unsigned char *values, int len,
                           bool ram, bool bbr, bool flash)
{
    uint8_t buffer[len + 4];
    int ind = 0;

    ubx_put_U1(buffer, &ind, 0);

    uint8_t mask = 0;
    mask |= (ram ? 1 : 0) << 0;
    mask |= (bbr ? 1 : 0) << 1;
    mask |= (flash ? 1 : 0) << 2;
    ubx_put_X1(buffer, &ind, mask);

    ubx_put_U1(buffer, &ind, 0);
    ubx_put_U1(buffer, &ind, 0);

    memcpy(buffer + ind, values, len);
    ind += len;

    return ubx_encode_send(UBX_CLASS_CFG, UBX_CFG_VALSET, buffer, ind, 100);
}

void Ublox::ubloxCfgAppendEnableGps(unsigned char *buffer, int *ind,
                                    bool en, bool en_l1c, bool en_l2c)
{
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GPS_ENA);
    ubx_put_U1(buffer, ind, en);
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GPS_L1C_ENA);
    ubx_put_U1(buffer, ind, en_l1c);
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GPS_L2C_ENA);
    ubx_put_U1(buffer, ind, en_l2c);
}

void Ublox::ubloxCfgAppendEnableGal(unsigned char *buffer, int *ind,
                                    bool en, bool en_e1, bool en_e5b)
{
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GAL_ENA);
    ubx_put_U1(buffer, ind, en);
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GAL_E1_ENA);
    ubx_put_U1(buffer, ind, en_e1);
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GAL_E5B_ENA);
    ubx_put_U1(buffer, ind, en_e5b);
}

void Ublox::ubloxCfgAppendEnableBds(unsigned char *buffer, int *ind,
                                    bool en, bool en_b1, bool en_b2)
{
    ubx_put_X4(buffer, ind, CFG_SIGNAL_BDS_ENA);
    ubx_put_U1(buffer, ind, en);
    ubx_put_X4(buffer, ind, CFG_SIGNAL_BDS_B1_ENA);
    ubx_put_U1(buffer, ind, en_b1);
    ubx_put_X4(buffer, ind, CFG_SIGNAL_BDS_B2_ENA);
    ubx_put_U1(buffer, ind, en_b2);
}

void Ublox::ubloxCfgAppendEnableGlo(unsigned char *buffer, int *ind,
                                    bool en, bool en_l1, bool en_l2)
{
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GLO_ENA);
    ubx_put_U1(buffer, ind, en);
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GLO_L1_ENA);
    ubx_put_U1(buffer, ind, en_l1);
    ubx_put_X4(buffer, ind, CFG_SIGNAL_GLO_L2_ENA);
    ubx_put_U1(buffer, ind, en_l2);
}

void Ublox::ubloxCfgAppendUart1Baud(unsigned char *buffer, int *ind, uint32_t baudrate)
{
    ubx_put_X4(buffer, ind, CFG_UART1_BAUDRATE);
    ubx_put_U4(buffer, ind, baudrate);
}

void Ublox::ubloxCfgAppendUart1InProt(unsigned char *buffer, int *ind, bool ubx, bool nmea, bool rtcm3x)
{
    ubx_put_X4(buffer, ind, CFG_UART1INPROT_UBX);
    ubx_put_U1(buffer, ind, ubx);
    ubx_put_X4(buffer, ind, CFG_UART1INPROT_NMEA);
    ubx_put_U1(buffer, ind, nmea);
    ubx_put_X4(buffer, ind, CFG_UART1INPROT_RTCM3X);
    ubx_put_U1(buffer, ind, rtcm3x);
}

void Ublox::ubloxCfgAppendUart1OutProt(unsigned char *buffer, int *ind, bool ubx, bool nmea, bool rtcm3x)
{
    ubx_put_X4(buffer, ind, CFG_UART1OUTPROT_UBX);
    ubx_put_U1(buffer, ind, ubx);
    ubx_put_X4(buffer, ind, CFG_UART1OUTPROT_NMEA);
    ubx_put_U1(buffer, ind, nmea);
    ubx_put_X4(buffer, ind, CFG_UART1OUTPROT_RTCM3X);
    ubx_put_U1(buffer, ind, rtcm3x);
}

void Ublox::serialDataAvailable()
{
    while (mSerialPort->bytesAvailable() > 0) {
        QByteArray data = mSerialPort->readAll();

        for (int i = 0;i < data.size();i++) {
            uint8_t ch = data.at(i);
            bool ch_used = false;

            // RTCM
            if (!ch_used && mDecoderState.line_pos == 0 && mDecoderState.ubx_pos == 0) {
                int res = rtcm3_input_data(ch, &mRtcmState);
                ch_used = res >= 0;

                if (res >= 1000) {
                    QByteArray rtcmData((const char*)mRtcmState.buffer, mRtcmState.len + 3);
                    emit rtcmRx(rtcmData, res);
                }
            }

            // Ubx
            if (!ch_used && mDecoderState.line_pos == 0) {
                int ubx_pos_last = mDecoderState.ubx_pos;

                if (mDecoderState.ubx_pos == 0) {
                    if (ch == 0xB5) {
                        mDecoderState.ubx_pos++;
                    }
                } else if (mDecoderState.ubx_pos == 1) {
                    if (ch == 0x62) {
                        mDecoderState.ubx_pos++;
                        mDecoderState.ubx_ck_a = 0;
                        mDecoderState.ubx_ck_b = 0;
                    }
                } else if (mDecoderState.ubx_pos == 2) {
                    mDecoderState.ubx_class = ch;
                    mDecoderState.ubx_ck_a += ch;
                    mDecoderState.ubx_ck_b += mDecoderState.ubx_ck_a;
                    mDecoderState.ubx_pos++;
                } else if (mDecoderState.ubx_pos == 3) {
                    mDecoderState.ubx_id = ch;
                    mDecoderState.ubx_ck_a += ch;
                    mDecoderState.ubx_ck_b += mDecoderState.ubx_ck_a;
                    mDecoderState.ubx_pos++;
                } else if (mDecoderState.ubx_pos == 4) {
                    mDecoderState.ubx_len = ch;
                    mDecoderState.ubx_ck_a += ch;
                    mDecoderState.ubx_ck_b += mDecoderState.ubx_ck_a;
                    mDecoderState.ubx_pos++;
                } else if (mDecoderState.ubx_pos == 5) {
                    mDecoderState.ubx_len |= ch << 8;
                    if (mDecoderState.ubx_len >= sizeof(mDecoderState.ubx)) {
                        qDebug() << "Too large UBX packet" << mDecoderState.ubx_len;
                    } else {
                        mDecoderState.ubx_ck_a += ch;
                        mDecoderState.ubx_ck_b += mDecoderState.ubx_ck_a;
                        mDecoderState.ubx_pos++;
                    }
                } else if ((mDecoderState.ubx_pos - 6) < mDecoderState.ubx_len) {
                    mDecoderState.ubx[mDecoderState.ubx_pos - 6] = ch;
                    mDecoderState.ubx_ck_a += ch;
                    mDecoderState.ubx_ck_b += mDecoderState.ubx_ck_a;
                    mDecoderState.ubx_pos++;
                } else if ((mDecoderState.ubx_pos - 6) == mDecoderState.ubx_len) {
                    if (ch == mDecoderState.ubx_ck_a) {
                        mDecoderState.ubx_pos++;
                    }
                } else if ((mDecoderState.ubx_pos - 6) == (mDecoderState.ubx_len + 1)) {
                    if (ch == mDecoderState.ubx_ck_b) {
                        ubx_decode(mDecoderState.ubx_class, mDecoderState.ubx_id,
                                   mDecoderState.ubx, mDecoderState.ubx_len);
                        mDecoderState.ubx_pos = 0;
                    }
                }

                if (ubx_pos_last != mDecoderState.ubx_pos) {
                    ch_used = true;
                } else {
                    mDecoderState.ubx_pos = 0;
                }
            }

            // NMEA
            if (!ch_used) {
                mDecoderState.line[mDecoderState.line_pos++] = ch;
                if (mDecoderState.line_pos == sizeof (mDecoderState.line)) {
                    mDecoderState.line_pos = 0;
                }

                if (mDecoderState.line_pos > 0 && mDecoderState.line[mDecoderState.line_pos - 1] == '\n') {
                    mDecoderState.line[mDecoderState.line_pos] = '\0';
                    mDecoderState.line_pos = 0;

                    QByteArray line((char*)mDecoderState.line);
                    NmeaServer::nmea_gga_info_t gga;
                    int fields = NmeaServer::decodeNmeaGGA(line, gga);
                    if (fields > 0) {
                        emit rxGga(fields, gga);
                    }
                }
            }
        }
    }
}

void Ublox::serialPortError(QSerialPort::SerialPortError error)
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
        qDebug() << message;

        if(mSerialPort->isOpen()) {
            mSerialPort->close();
        }
    }
}

void Ublox::ubx_send(QByteArray data)
{
    if (mSerialPort->isOpen()) {
        mSerialPort->write(data);
    }
}

bool Ublox::ubx_encode_send(uint8_t msg_class, uint8_t id, uint8_t *msg, int len, int timeoutMs)
{
    auto ubx = ubx_encode(msg_class, id, QByteArray((const char*)msg, len));

    bool retVal = false;
    if (timeoutMs > 0) {
        if (mWaitingAck) {
            qDebug() << "Already waiting for ack";
        } else {
            mWaitingAck = true;

            QEventLoop loop;
            QTimer timeoutTimer;
            timeoutTimer.setSingleShot(true);
            timeoutTimer.start(timeoutMs);
            auto conn = connect(this, &Ublox::rxAck,
                                [&loop, &retVal](uint8_t, uint8_t){retVal = true; loop.quit();});
            connect(this, SIGNAL(rxNak(uint8_t,uint8_t)), &loop, SLOT(quit()));
            connect(&timeoutTimer, SIGNAL(timeout()), &loop, SLOT(quit()));

            ubx_send(ubx);
            loop.exec();

            disconnect(conn);

            mWaitingAck = false;
        }
    } else {
        ubx_send(ubx);
        retVal = true;
    }

    return retVal;
}

QByteArray Ublox::ubx_encode(uint8_t msg_class, uint8_t id, const QByteArray &data)
{
    QByteArray ubx;

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;

    ubx.append(0xB5);
    ubx.append(0x62);

    ubx.append(msg_class);
    ck_a += ubx.at(ubx.size() - 1);
    ck_b += ck_a;

    ubx.append(id);
    ck_a += ubx.at(ubx.size() - 1);
    ck_b += ck_a;

    ubx.append(data.size() & 0xFF);
    ck_a += ubx.at(ubx.size() - 1);
    ck_b += ck_a;

    ubx.append((data.size() >> 8) & 0xFF);
    ck_a += ubx.at(ubx.size() - 1);
    ck_b += ck_a;

    for (int i = 0;i < data.size();i++) {
        ubx.append(data.at(i));
        ck_a += data.at(i);
        ck_b += ck_a;
    }

    ubx.append(ck_a);
    ubx.append(ck_b);

    return ubx;
}

void Ublox::ubx_decode(uint8_t msg_class, uint8_t id, uint8_t *msg, int len)
{
    emit ubxRx(ubx_encode(msg_class, id, QByteArray((const char*)msg, len)));

    switch (msg_class) {
    case UBX_CLASS_NAV: {
        switch (id) {
        case UBX_NAV_RELPOSNED:
            ubx_decode_relposned(msg, len);
            break;
        case UBX_NAV_SVIN:
            ubx_decode_svin(msg, len);
            break;
        case UBX_NAV_SOL: // TODO: dropped on F9P, implement UBX-NAV-PVT or UBX-NAV-HPPOSLLH (see: https://cdn.sparkfun.com/assets/learn_tutorials/8/5/6/ZED-F9P_FW_1.00_HPG_1.00_release_notes.pdf)
            ubx_decode_nav_sol(msg, len);
            break;
        case UBX_NAV_SAT:
            ubx_decode_nav_sat(msg, len);
            break;
        default:
            break;
        }
    } break;

    case UBX_CLASS_ACK: {
        switch (id) {
        case UBX_ACK_ACK:
            ubx_decode_ack(msg, len);
            break;

        case UBX_ACK_NAK:
            ubx_decode_nak(msg, len);
            break;

        default:
            break;
        }
    } break;

    case UBX_CLASS_RXM: {
        switch (id) {
        case UBX_RXM_RAWX:
            ubx_decode_rawx(msg, len);
            break;
        default:
            break;
        }
    } break;

    case UBX_CLASS_CFG: {
        switch (id) {
        case UBX_CFG_GNSS:
            ubx_decode_cfg_gnss(msg, len);
            break;
        default:
            break;
        }
    } break;

    case UBX_CLASS_MON: {
        switch (id) {
        case UBX_MON_VER:
            ubx_decode_mon_ver(msg, len);
            break;
        default:
            break;
        }
    } break;

    default:
        break;
    }
}

void Ublox::ubx_decode_nav_sol(uint8_t *msg, int len)
{
    (void)len;

    static ubx_nav_sol sol;
    int ind = 0;
    uint8_t flags;

    sol.i_tow = ubx_get_U4(msg, &ind); // 0
    sol.f_tow = ubx_get_I4(msg, &ind); // 4
    sol.weel = ubx_get_I2(msg, &ind); // 8
    sol.gps_fix = ubx_get_U1(msg, &ind); // 10
    flags = ubx_get_X1(msg, &ind); // 11
    sol.gpsfixok = flags & 0x01;
    sol.diffsoln = flags & 0x02;
    sol.wknset = flags & 0x04;
    sol.towset = flags & 0x08;
    sol.ecef_x = (double)ubx_get_I4(msg, &ind) / D(100.0); // 12
    sol.ecef_y = (double)ubx_get_I4(msg, &ind) / D(100.0); // 16
    sol.ecef_z = (double)ubx_get_I4(msg, &ind) / D(100.0); // 20
    sol.p_acc = (float)ubx_get_U4(msg, &ind) / 100.0; // 24
    sol.ecef_vx = (float)ubx_get_I4(msg, &ind) / 100.0; // 28
    sol.ecef_vy = (float)ubx_get_I4(msg, &ind) / 100.0; // 32
    sol.ecef_vz = (float)ubx_get_I4(msg, &ind) / 100.0; // 36
    sol.s_acc = (float)ubx_get_U4(msg, &ind) / 100.0; // 40
    sol.p_dop = (float)ubx_get_U2(msg, &ind) * 0.01; // 44
    ind += 1; // 46
    sol.num_sv = ubx_get_U1(msg, &ind); // 47

    emit rxNavSol(sol);
}

void Ublox::ubx_decode_relposned(uint8_t *msg, int len)
{
    (void)len;

    ubx_nav_relposned pos;
    int ind = 0;
    uint32_t flags;

    int version = ubx_get_U1(msg, &ind);
    ubx_get_U1(msg, &ind);

    pos.ref_station_id = ubx_get_U2(msg, &ind);
    pos.i_tow = ubx_get_U4(msg, &ind);
    pos.pos_n = (float)ubx_get_I4(msg, &ind) / 100.0;
    pos.pos_e = (float)ubx_get_I4(msg, &ind) / 100.0;
    pos.pos_d = (float)ubx_get_I4(msg, &ind) / 100.0;
    if (version == 1) {
        pos.pos_length = (float)ubx_get_I4(msg, &ind) / 100.0;
        pos.pos_heading = (float)ubx_get_I4(msg, &ind) / 100000.0;
        ind += 4;
    }

    pos.pos_n += (float)ubx_get_I1(msg, &ind) / 10000.0;
    pos.pos_e += (float)ubx_get_I1(msg, &ind) / 10000.0;
    pos.pos_d += (float)ubx_get_I1(msg, &ind) / 10000.0;
    if (version == 1)
        pos.pos_length += (float)ubx_get_I1(msg, &ind) / 10000.0;
    else
        ind += 1;

    pos.acc_n = (float)ubx_get_U4(msg, &ind) / 10000.0;
    pos.acc_e = (float)ubx_get_U4(msg, &ind) / 10000.0;
    pos.acc_d = (float)ubx_get_U4(msg, &ind) / 10000.0;
    if (version == 1) {
        pos.acc_length = (float)ubx_get_I4(msg, &ind) / 10000.0;
        pos.acc_heading = (float)ubx_get_I4(msg, &ind) / 100000.0;
        ind += 4;
    }

    flags = ubx_get_X4(msg, &ind);
    pos.fix_ok =                (flags >> 0) & 1;
    pos.diff_soln =             (flags >> 1) & 1;
    pos.rel_pos_valid =         (flags >> 2) & 1;
    pos.carr_soln =             (flags >> 3) & 3;
    pos.is_moving =             (flags >> 5) & 1;
    pos.ref_pos_miss =          (flags >> 6) & 1;
    pos.ref_obs_miss =          (flags >> 7) & 1;
    pos.rel_pos_heading_valid = (flags >> 8) & 1;
    pos.rel_pos_normalized =    (flags >> 9) & 1;

    emit rxRelPosNed(pos);
}

void Ublox::ubx_decode_svin(uint8_t *msg, int len)
{
    (void)len;

    ubx_nav_svin svin;
    int ind = 4;

    svin.i_tow = ubx_get_U4(msg, &ind);
    svin.dur = ubx_get_U4(msg, &ind);
    svin.meanX = (double)ubx_get_I4(msg, &ind) / D(100.0);
    svin.meanY = (double)ubx_get_I4(msg, &ind) / D(100.0);
    svin.meanZ = (double)ubx_get_I4(msg, &ind) / D(100.0);
    svin.meanX += (double)ubx_get_I1(msg, &ind) / D(10000.0);
    svin.meanY += (double)ubx_get_I1(msg, &ind) / D(10000.0);
    svin.meanZ += (double)ubx_get_I1(msg, &ind) / D(10000.0);
    ind += 1;
    svin.meanAcc = (float)ubx_get_U4(msg, &ind) / 10000.0;
    svin.obs = ubx_get_U4(msg, &ind);
    svin.valid = ubx_get_U1(msg, &ind);
    svin.active = ubx_get_U1(msg, &ind);

    emit rxSvin(svin);
}

void Ublox::ubx_decode_ack(uint8_t *msg, int len)
{
    (void)len;

    int ind = 0;

    uint8_t cls_id = ubx_get_I1(msg, &ind);
    uint8_t msg_id = ubx_get_I1(msg, &ind);

    emit rxAck(cls_id, msg_id);
}

void Ublox::ubx_decode_nak(uint8_t *msg, int len)
{
    (void)len;

    int ind = 0;

    uint8_t cls_id = ubx_get_I1(msg, &ind);
    uint8_t msg_id = ubx_get_I1(msg, &ind);

    emit rxNak(cls_id, msg_id);
}

void Ublox::ubx_decode_rawx(uint8_t *msg, int len)
{
    (void)len;

    ubx_rxm_rawx raw;
    int ind = 0;
    uint32_t flags;

    raw.rcv_tow = ubx_get_R8(msg, &ind);
    raw.week = ubx_get_U2(msg, &ind);
    raw.leaps = ubx_get_I1(msg, &ind);
    raw.num_meas = ubx_get_U1(msg, &ind);
    flags = ubx_get_X1(msg, &ind);
    raw.leap_sec = flags & 0x01;
    raw.clk_reset = flags & 0x02;

    ind = 16;

    for (int i = 0;i < raw.num_meas;i++) {
        raw.obs[i].pr_mes = ubx_get_R8(msg, &ind);
        raw.obs[i].cp_mes = ubx_get_R8(msg, &ind);
        raw.obs[i].do_mes = ubx_get_R4(msg, &ind);
        raw.obs[i].gnss_id = ubx_get_U1(msg, &ind);
        raw.obs[i].sv_id = ubx_get_U1(msg, &ind);
        ind += 1;
        raw.obs[i].freq_id = ubx_get_U1(msg, &ind);
        raw.obs[i].locktime = ubx_get_U2(msg, &ind);
        raw.obs[i].cno = ubx_get_U1(msg, &ind);
        raw.obs[i].pr_stdev = ubx_get_X1(msg, &ind) & 0x0F;
        raw.obs[i].cp_stdev = ubx_get_X1(msg, &ind) & 0x0F;
        raw.obs[i].do_stdev = ubx_get_X1(msg, &ind) & 0x0F;
        flags = ubx_get_X1(msg, &ind);
        raw.obs[i].pr_valid = flags & 0x01;
        raw.obs[i].cp_valid = flags & 0x02;
        raw.obs[i].half_cyc_valid = flags & 0x04;
        raw.obs[i].half_cyc_sub = flags & 0x08;
        ind += 1;
    }

    emit rxRawx(raw);
}

void Ublox::ubx_decode_nav_sat(uint8_t *msg, int len)
{
    (void)len;

    ubx_nav_sat sat;
    int ind = 0;

    sat.i_tow_ms = ubx_get_U4(msg, &ind);
    ubx_get_U1(msg, &ind);
    sat.num_sv = ubx_get_U1(msg, &ind);
    ubx_get_U1(msg, &ind);
    ubx_get_U1(msg, &ind);

    if (sat.num_sv > 128) {
        sat.num_sv = 128;
    }

    for (int i = 0;i < sat.num_sv;i++) {
        sat.sats[i].gnss_id = ubx_get_U1(msg, &ind);
        sat.sats[i].sv_id = ubx_get_U1(msg, &ind);
        sat.sats[i].cno = ubx_get_U1(msg, &ind);
        sat.sats[i].elev = ubx_get_I1(msg, &ind);
        sat.sats[i].azim = ubx_get_I2(msg, &ind);
        sat.sats[i].pr_res = (float)ubx_get_I2(msg, &ind) * 0.1;
        uint32_t flags = ubx_get_X4(msg, &ind);
        sat.sats[i].quality = (flags >> 0) & 0x07;
        sat.sats[i].used = (flags >> 3) & 0x01;
        sat.sats[i].health = (flags >> 4) & 0x03;
        sat.sats[i].diffcorr = (flags >> 6) & 0x01;
    }

    emit rxNavSat(sat);
}

void Ublox::ubx_decode_cfg_gnss(uint8_t *msg, int len)
{
    (void)len;

    ubx_cfg_gnss cfg;
    int ind = 0;

    ubx_get_U1(msg, &ind);
    cfg.num_ch_hw = ubx_get_U1(msg, &ind);
    cfg.num_ch_use = ubx_get_U1(msg, &ind);
    cfg.num_blocks = ubx_get_U1(msg, &ind);

    if (cfg.num_blocks > 10) {
        cfg.num_blocks = 10;
    }

    for (int i = 0;i < cfg.num_blocks;i++) {
        cfg.blocks[i].gnss_id = ubx_get_U1(msg, &ind);
        cfg.blocks[i].minTrkCh = ubx_get_U1(msg, &ind);
        cfg.blocks[i].maxTrkCh = ubx_get_U1(msg, &ind);
        ubx_get_U1(msg, &ind);
        uint32_t flags = ubx_get_X4(msg, &ind);
        cfg.blocks[i].en = flags & 1;
        cfg.blocks[i].flags = flags >> 16 & 0xFF;
    }

    emit rxCfgGnss(cfg);
}

void Ublox::ubx_decode_mon_ver(uint8_t *msg, int len)
{
    QString sw, hw;
    QStringList extensions;

    int ind = 0;
    sw = QString::fromLocal8Bit((const char*)msg + ind);
    ind += 30;
    hw = QString::fromLocal8Bit((const char*)msg + ind);
    ind += 10;

    while (ind < len) {
        extensions.append(QString::fromLocal8Bit((const char*)msg + ind));
        ind += 30;
    }

    emit rxMonVer(sw, hw, extensions);
}
