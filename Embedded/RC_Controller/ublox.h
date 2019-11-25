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

#ifndef UBLOX_H_
#define UBLOX_H_

#include "conf_general.h"
#include "ch.h"
#include "hal.h"

void ublox_init(void);
void ublox_send(unsigned char *data, unsigned int len);
void ublox_set_rx_callback_nav_sol(void(*func)(ubx_nav_sol *sol));
void ublox_set_rx_callback_relposned(void(*func)(ubx_nav_relposned *pos));
void ublox_set_rx_callback_rawx(void(*func)(ubx_rxm_rawx *rawx));
void ublox_set_rx_callback_svin(void(*func)(ubx_nav_svin *svin));
void ublox_set_rx_callback_nav_sat(void(*func)(ubx_nav_sat *sat));
void ublox_set_rx_callback_cfg_gnss(void(*func)(ubx_cfg_gnss *gnss));
void ublox_poll(uint8_t msg_class, uint8_t id);
int ublox_cfg_prt_uart(ubx_cfg_prt_uart *cfg);
int ublox_cfg_tmode3(ubx_cfg_tmode3 *cfg);
int ublox_cfg_msg(uint8_t msg_class, uint8_t id, uint8_t rate);
int ublox_cfg_rate(uint16_t meas_rate_ms, uint16_t nav_rate_ms, uint16_t time_ref);
int ublox_cfg_cfg(ubx_cfg_cfg *cfg);
int ublox_cfg_nav5(ubx_cfg_nav5 *cfg);
int ublox_cfg_tp5(ubx_cfg_tp5 *cfg);
int ublox_cfg_gnss(ubx_cfg_gnss *gnss);
int ublox_cfg_nmea(ubx_cfg_nmea *nmea);
int ublox_cfg_valset(unsigned char *values, int len,
		bool ram, bool bbr, bool flash);

void ublox_cfg_append_enable_gps(unsigned char *buffer, int *ind,
		bool en, bool en_l1c, bool en_l2c);
void ublox_cfg_append_enable_gal(unsigned char *buffer, int *ind,
		bool en, bool en_e1, bool en_e5b);
void ublox_cfg_append_enable_bds(unsigned char *buffer, int *ind,
		bool en, bool en_b1, bool en_b2);
void ublox_cfg_append_enable_glo(unsigned char *buffer, int *ind,
		bool en, bool en_l1, bool en_l2);

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
#define UBX_NMEA_GLL                    0x01
#define UBX_NMEA_GSA                    0x02
#define UBX_NMEA_GSV                    0x03
#define UBX_NMEA_RMC                    0x04
#define UBX_NMEA_VTG                    0x05
#define UBX_NMEA_GRS                    0x06
#define UBX_NMEA_GST                    0x07
#define UBX_NMEA_ZDA                    0x08
#define UBX_NMEA_GBS                    0x09
#define UBX_NMEA_DTM                    0x0A

#endif /* UBLOX_H_ */
