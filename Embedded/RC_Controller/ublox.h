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

#ifndef UBLOX_H_
#define UBLOX_H_

#include "conf_general.h"
#include "ch.h"
#include "hal.h"

void ublox_init(void);
void ublox_send(unsigned char *data, unsigned int len);
void ublox_set_rx_callback_relposned(void(*func)(ubx_nav_relposned *pos));
void ublox_set_rx_callback_rawx(void(*func)(ubx_rxm_rawx *pos));
void ublox_set_rx_callback_svin(void(*func)(ubx_nav_svin *pos));
void ublox_poll(uint8_t msg_class, uint8_t id);
int ublox_cfg_prt_uart(ubx_cfg_prt_uart *cfg);
int ublox_cfg_tmode3(ubx_cfg_tmode3 *cfg);
int ublox_cfg_msg(uint8_t msg_class, uint8_t id, uint8_t rate);
int ublox_cfg_rate(uint16_t meas_rate_ms, uint16_t nav_rate_ms, uint16_t time_ref);
int ublox_cfg_nav5(ubx_cfg_nav5 *cfg);
int ublox_cfg_tp5(ubx_cfg_tp5 *cfg);

// Message classes
#define UBX_CLASS_NAV					0x01
#define UBX_CLASS_RXM					0x02
#define UBX_CLASS_INF					0x04
#define UBX_CLASS_ACK					0x05
#define UBX_CLASS_CFG					0x06
#define UBX_CLASS_UPD					0x06
#define UBX_CLASS_MON					0x09
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
#define UBX_NAV_RELPOSNED				0x3C
#define UBX_NAV_SVIN					0x3B

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
#define UBX_CFG_NAV5					0x24
#define UBX_CFG_TP5						0x31
#define UBX_CFG_TMODE3					0x71

// RTCM3 messages
#define UBX_RTCM3_1005					0x05
#define UBX_RTCM3_1077					0x4D
#define UBX_RTCM3_1087					0x57
#define UBX_RTCM3_1127					0x7F

// NMEA messages
#define UBX_NMEA_GGA					0x00

#endif /* UBLOX_H_ */
