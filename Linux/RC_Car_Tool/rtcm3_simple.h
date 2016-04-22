/*
    Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

typedef struct {
    double t_tow;       // Time of week
    double t_wn;        // Week number
    int staid;          // ref station id
    bool sync;          // True if more messages are coming
    int type;           // RTCM Type
} rtcm_obs_header_t;

typedef struct {
    double P[2];        // Pseudorange observation
    double L[2];        // Carrier phase observation
    uint8_t cn0[2];     // Carrier-to-Noise density [dB Hz]
    uint8_t lock[2];   // Lock. Set to 0 when the lock has changed, 127 otherwise. TODO: Really?
    uint8_t prn;        // Sattelite
    uint8_t code[2];    // Code indicator
} rtcm_obs_gps_t;

typedef struct {
    int staid;
    double lat;
    double lon;
    double height;
    double ant_height;
} rtcm_ref_sta_pos_t;

typedef struct {
  double tgd;           // Group delay differential between L1 and L2 [s]
  double c_rs;          // Amplitude of the sine harmonic correction term to the orbit radius [m]
  double c_rc;          // Amplitude of the cosine harmonic correction term to the orbit radius [m]
  double c_uc;          // Amplitude of the cosine harmonic correction term to the argument of latitude [rad]
  double c_us;          // Amplitude of the sine harmonic correction term to the argument of latitude [rad]
  double c_ic;          // Amplitude of the cosine harmonic correction term to the angle of inclination [rad]
  double c_is;          // Amplitude of the sine harmonic correction term to the angle of inclination [rad]
  double dn;            // Mean motion difference [rad/s]
  double m0;            // Mean anomaly at reference time [radians]
  double ecc;           // Eccentricity of satellite orbit
  double sqrta;         // Square root of the semi-major axis of orbit [m^(1/2)]
  double omega0;        // Longitude of ascending node of orbit plane at weekly epoch [rad]
  double omegadot;      // Rate of right ascension [rad/s]
  double w;             // Argument of perigee [rad]
  double inc;           // Inclination [rad]
  double inc_dot;       // Inclination first derivative [rad/s]
  double af0;           // Polynomial clock correction coefficient (clock bias) [s]
  double af1;           // Polynomial clock correction coefficient (clock drift) [s/s]
  double af2;           // Polynomial clock correction coefficient (rate of clock drift) [s/s^2]
  double toe_tow;       // Time of week [s]
  uint16_t toe_wn;      // Week number [week]
  double toc_tow;       // Clock reference time of week [s]
  int sva;              // SV accuracy (URA index)
  int svh;              // SV health (0:ok)
  int code;             // GPS/QZS: code on L2, GAL/CMP: data sources
  int flag;             // GPS/QZS: L2 P data flag, CMP: nav type
  double fit;           // fit interval (h)
  uint8_t prn;          // Sattelite
  uint8_t iode;         // Issue of ephemeris data
  uint16_t iodc;        // Issue of clock data
} rtcm_ephemeris_t;

void rtcm3_set_rx_callback_obs_gps(void(*func)(rtcm_obs_header_t *header, rtcm_obs_gps_t *obs, int obs_num));
void rtcm3_set_rx_callback_1005_1006(void(*func)(rtcm_ref_sta_pos_t *pos));
void rtcm3_set_rx_callback_1019(void(*func)(rtcm_ephemeris_t *eph));
void rtcm3_set_rx_callback(void(*func)(uint8_t *data, int len, int type));
int rtcm3_input_data(uint8_t data);
void rtcm3_get_last_decoded_buffer(const uint8_t **data, int *len, int *type);
int rtcm3_encode_1002(rtcm_obs_header_t *header, rtcm_obs_gps_t *obs,
                       int obs_num, uint8_t *buffer, int *buffer_len);
int rtcm3_encode_1006(rtcm_ref_sta_pos_t pos, uint8_t *buffer, int *buffer_len);
int rtcm3_encode_1019(rtcm_ephemeris_t *eph, uint8_t *buffer, int *buffer_len);

#ifdef __cplusplus
}
#endif

#endif // RTCM3_SIMPLE_H

