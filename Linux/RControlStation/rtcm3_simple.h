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

#ifndef RTCM3_SIMPLE_H
#define RTCM3_SIMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "datatypes.h"

#ifndef D
#define D(x) 						((double)x##L)
#endif

#ifndef D_PI
#define D_PI						D(3.14159265358979323846)
#endif

// Defines
#define RTCM3PREAMB		0xD3                // rtcm ver.3 frame preamble
#define CODE_L1C        1                   // obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS)
#define CODE_L1P        2                   // obs code: L1P,G1P    (GPS,GLO)
#define CODE_L2C        14                  // obs code: L2C/A,G1C/A (GPS,GLO)
#define CODE_L2P        19                  // obs code: L2P,G2P    (GPS,GLO)
#define CODE_L2W        20                  // obs code: L2 Z-track (GPS)

#define SYS_NONE        0x00                // navigation system: none
#define SYS_GPS         0x01                // navigation system: GPS
#define SYS_SBS         0x02                // navigation system: SBAS
#define SYS_GLO         0x04                // navigation system: GLONASS
#define SYS_GAL         0x08                // navigation system: Galileo
#define SYS_QZS         0x10                // navigation system: QZSS
#define SYS_CMP         0x20                // navigation system: BeiDou
#define SYS_LEO         0x40                // navigation system: LEO
#define SYS_ALL         0xFF                // navigation system: all

// Functions
void rtcm3_set_rx_callback_obs(void(*func)(rtcm_obs_header_t *header, rtcm_obs_t *obs, int obs_num), rtcm3_state *state);
void rtcm3_set_rx_callback_1005_1006(void(*func)(rtcm_ref_sta_pos_t *pos), rtcm3_state *state);
void rtcm3_set_rx_callback_1019(void(*func)(rtcm_ephemeris_t *eph), rtcm3_state *state);
void rtcm3_set_rx_callback(void(*func)(uint8_t *data, int len, int type), rtcm3_state *state);
void rtcm3_init_state(rtcm3_state *state);
int rtcm3_input_data(uint8_t data, rtcm3_state *state);
int rtcm3_encode_1002(rtcm_obs_header_t *header, rtcm_obs_t *obs,
                      int obs_num, uint8_t *buffer, int *buffer_len);
int rtcm3_encode_1010(rtcm_obs_header_t *header, rtcm_obs_t *obs,
                      int obs_num, uint8_t *buffer, int *buffer_len);
int rtcm3_encode_1006(rtcm_ref_sta_pos_t pos, uint8_t *buffer, int *buffer_len);
int rtcm3_encode_1019(rtcm_ephemeris_t *eph, uint8_t *buffer, int *buffer_len);

#ifdef __cplusplus
}
#endif

#endif // RTCM3_SIMPLE_H

