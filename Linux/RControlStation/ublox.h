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

#ifndef UBLOX_H
#define UBLOX_H

#include <QObject>
#include <QSerialPort>
#include <QTimer>
#include <cstdint>
#include "nmeaserver.h"
#include "rtcm3_simple.h"
#include "datatypes.h"

class Ublox : public QObject
{
    Q_OBJECT
public:
    explicit Ublox(QObject *parent = 0);
    bool connectSerial(QString port, int baudrate = 115200);
    void disconnectSerial();
    bool isSerialConnected();
    void writeRaw(QByteArray data);

    void ubxPoll(uint8_t msg_class, uint8_t id);
    bool ubxCfgPrtUart(ubx_cfg_prt_uart *cfg);
    bool ubxCfgTmode3(ubx_cfg_tmode3 *cfg);
    bool ubxCfgMsg(uint8_t msg_class, uint8_t id, uint8_t rate);
    bool ubxCfgRate(uint16_t meas_rate_ms, uint16_t nav_rate_ms, uint16_t time_ref);
    bool ubloxCfgCfg(ubx_cfg_cfg *cfg);
    bool ubxCfgNav5(ubx_cfg_nav5 *cfg);
    bool ubloxCfgTp5(ubx_cfg_tp5 *cfg);
    bool ubloxCfgGnss(ubx_cfg_gnss *gnss);
    bool ubloxCfgNmea(ubx_cfg_nmea *nmea);
    bool ubloxCfgValset(unsigned char *values, int len,
                        bool ram, bool bbr, bool flash);

    void ubloxCfgAppendEnableGps(unsigned char *buffer, int *ind,
                                 bool en, bool en_l1c, bool en_l2c);
    void ubloxCfgAppendEnableGal(unsigned char *buffer, int *ind,
                                 bool en, bool en_e1, bool en_e5b);
    void ubloxCfgAppendEnableBds(unsigned char *buffer, int *ind,
                                 bool en, bool en_b1, bool en_b2);
    void ubloxCfgAppendEnableGlo(unsigned char *buffer, int *ind,
                                 bool en, bool en_l1, bool en_l2);
    void ubloxCfgAppendUart1Baud(unsigned char *buffer, int *ind, uint32_t baudrate);
    void ubloxCfgAppendUart1InProt(unsigned char *buffer, int *ind, bool ubx, bool nmea, bool rtcm3x);
    void ubloxCfgAppendUart1OutProt(unsigned char *buffer, int *ind, bool ubx, bool nmea, bool rtcm3x);

signals:
    void rxGga(int fields, NmeaServer::nmea_gga_info_t gga);
    void rxNavSol(ubx_nav_sol sol);
    void rxRelPosNed(ubx_nav_relposned pos);
    void rxSvin(ubx_nav_svin svin);
    void rxAck(uint8_t cls_id, uint8_t msg_id);
    void rxNak(uint8_t cls_id, uint8_t msg_id);
    void rxRawx(ubx_rxm_rawx rawx);
    void rxNavSat(ubx_nav_sat sat);
    void rxCfgGnss(ubx_cfg_gnss gnss);
    void rxMonVer(QString sw, QString hw, QStringList extensions);
    void ubxRx(const QByteArray &data);
    void rtcmRx(QByteArray data, int type);

public slots:

private slots:
    void serialDataAvailable();
    void serialPortError(QSerialPort::SerialPortError error);

private:
    typedef struct {
        uint8_t line[256];
        uint8_t ubx[4096];
        int line_pos;
        int ubx_pos;
        uint8_t ubx_class;
        uint8_t ubx_id;
        uint8_t ubx_ck_a;
        uint8_t ubx_ck_b;
        int ubx_len;
    } decoder_state;

    QSerialPort *mSerialPort;
    decoder_state mDecoderState;
    rtcm3_state mRtcmState;
    bool mWaitingAck;

    void ubx_send(QByteArray data);
    bool ubx_encode_send(uint8_t msg_class, uint8_t id, uint8_t *msg, int len, int timeoutMs = -1);
    QByteArray ubx_encode(uint8_t msg_class, uint8_t id, const QByteArray &data);

    void ubx_decode(uint8_t msg_class, uint8_t id, uint8_t *msg, int len);
    void ubx_decode_nav_sol(uint8_t *msg, int len);
    void ubx_decode_relposned(uint8_t *msg, int len);
    void ubx_decode_svin(uint8_t *msg, int len);
    void ubx_decode_ack(uint8_t *msg, int len);
    void ubx_decode_nak(uint8_t *msg, int len);
    void ubx_decode_rawx(uint8_t *msg, int len);
    void ubx_decode_nav_sat(uint8_t *msg, int len);
    void ubx_decode_cfg_gnss(uint8_t *msg, int len);
    void ubx_decode_mon_ver(uint8_t *msg, int len);
};

// Message classes
#define UBX_CLASS_NAV					0x01
#define UBX_CLASS_RXM					0x02
#define UBX_CLASS_INF					0x04
#define UBX_CLASS_ACK					0x05
#define UBX_CLASS_CFG					0x06
#define UBX_CLASS_UPD					0x09
#define UBX_CLASS_MON					0x0A
#define UBX_CLASS_AID					0x0B
#define UBX_CLASS_TIM					0x0D
#define UBX_CLASS_ESF					0x10
#define UBX_CLASS_MGA					0x13
#define UBX_CLASS_LOG					0x21
#define UBX_CLASS_SEC					0x27
#define UBX_CLASS_HNR					0x28
#define UBX_CLASS_NMEA					0xF0
#define UBX_CLASS_RTCM3					0xF5

// Navigation (NAV) messages
#define UBX_NAV_SOL						0x06
#define UBX_NAV_RELPOSNED				0x3C
#define UBX_NAV_SVIN					0x3B
#define UBX_NAV_SAT 					0x35

// Receiver Manager (RXM) messages
#define UBX_RXM_RAWX					0x15
#define UBX_RXM_SFRBX					0x13

// Ack messages
#define UBX_ACK_ACK						0x01
#define UBX_ACK_NAK						0x00

// Configuration messages
#define UBX_CFG_PRT						0x00
#define UBX_CFG_MSG						0x01
#define UBX_CFG_RATE					0x08
#define UBX_CFG_CFG						0x09
#define UBX_CFG_NAV5					0x24
#define UBX_CFG_TP5						0x31
#define UBX_CFG_TMODE3					0x71
#define UBX_CFG_GNSS                    0x3E
#define UBX_CFG_NMEA                    0x17
#define UBX_CFG_VALSET                  0x8A
#define UBX_CFG_VALGET                  0x8B
#define UBX_CFG_VALDEL                  0x8C

// Monitoring messages
#define UBX_MON_VER                     0x04

// Configuration Keys
#define CFG_SIGNAL_GPS_ENA              0x1031001F // GPS enable
#define CFG_SIGNAL_GPS_L1C_ENA          0x10310001 // GPS L1C/A
#define CFG_SIGNAL_GPS_L2C_ENA          0x10310003 // GPS L2C (only on u-blox F9)
#define CFG_SIGNAL_GAL_ENA              0x10310021 // Galileo enable
#define CFG_SIGNAL_GAL_E1_ENA           0x10310007 // Galileo E1
#define CFG_SIGNAL_GAL_E5B_ENA          0x1031000A // Galileo E5b (only on u-blox F9)
#define CFG_SIGNAL_BDS_ENA              0x10310022 // BeiDou Enable
#define CFG_SIGNAL_BDS_B1_ENA           0x1031000D // BeiDou B1I
#define CFG_SIGNAL_BDS_B2_ENA           0x1031000E // BeiDou B2I (only on u-blox F9)
#define CFG_SIGNAL_GLO_ENA              0x10310025 // GLONASS Enable
#define CFG_SIGNAL_GLO_L1_ENA           0x10310018 // GLONASS L1
#define CFG_SIGNAL_GLO_L2_ENA           0x1031001A // GLONASS L2 (only on u-blox F9)

#define CFG_UART1_BAUDRATE              0x40520001
#define CFG_UART1INPROT_UBX             0x10730001
#define CFG_UART1INPROT_NMEA            0x10730002
#define CFG_UART1INPROT_RTCM3X          0x10730004
#define CFG_UART1OUTPROT_UBX            0x10740001
#define CFG_UART1OUTPROT_NMEA           0x10740002
#define CFG_UART1OUTPROT_RTCM3X         0x10740004

// RTCM3 messages
#define UBX_RTCM3_1005					0x05 // Stationary RTK reference station ARP
#define UBX_RTCM3_1074					0x4A // GPS MSM4
#define UBX_RTCM3_1077					0x4D // GPS MSM7
#define UBX_RTCM3_1084					0x54 // GLONASS MSM4
#define UBX_RTCM3_1087					0x57 // GLONASS MSM7
#define UBX_RTCM3_1094					0x5E // Galileo MSM4
#define UBX_RTCM3_1097					0x61 // Galileo MSM7
#define UBX_RTCM3_1124					0x7C // BeiDou MSM4
#define UBX_RTCM3_1127					0x7F // BeiDou MSM7
#define UBX_RTCM3_1230					0xE6 // GLONASS code-phase biases
#define UBX_RTCM3_4072_0				0xFE // Reference station PVT (u-blox proprietary RTCM Message)
#define UBX_RTCM3_4072_1				0xFD // Additional reference station information (u-blox proprietary RTCM Message)

// GNSS IDs
#define UBX_GNSS_ID_GPS                 0
#define UBX_GNSS_ID_SBAS                1
#define UBX_GNSS_ID_GALILEO             2
#define UBX_GNSS_ID_BEIDOU              3
#define UBX_GNSS_ID_IMES                4
#define UBX_GNSS_ID_QZSS                5
#define UBX_GNSS_ID_GLONASS             6
#define UBX_GNSS_ID_IRNSS               7

// CFG_GNSS flags
#define UBX_CFG_GNSS_GPS_L1C            0x01
#define UBX_CFG_GNSS_GPS_L2C            0x10
#define UBX_CFG_GNSS_SBAS_L1C           0x01
#define UBX_CFG_GNSS_GAL_E1             0x01
#define UBX_CFG_GNSS_GAL_E5B            0x20
#define UBX_CFG_GNSS_BDS_B1L            0x01
#define UBX_CFG_GNSS_BDS_B2L            0x10
#define UBX_CFG_GNSS_IMES_L1            0x01
#define UBX_CFG_GNSS_QZSS_L1C           0x01
#define UBX_CFG_GNSS_QZSS_L1S           0x04
#define UBX_CFG_GNSS_QZSS_L2C           0x10
#define UBX_CFG_GNSS_GLO_L1             0x01
#define UBX_CFG_GNSS_GLO_L2             0x10
#define UBX_CFG_GNSS_IRNSS_L5A          0x01

// NMEA messages
#define UBX_NMEA_GGA                    0x00

#endif // UBLOX_H
