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

// This code is based on the implementation found in rtklib by T.TAKASU.
// https://github.com/tomojitakasu/RTKLIB

#include "rtcm3_simple.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// Defines
#define ROUND(x)        ((int)floor((x)+D(0.5)))
#define ROUND_U(x)      ((unsigned int)floor((x)+D(0.5)))
#define CLIGHT          D(299792458.0)      // speed of light (m/s)
#define FREQ1           D(1.57542E9)        // L1/E1  frequency (Hz)
#define FREQ2           D(1.22760E9)        // L2     frequency (Hz)
#define FREQ5           D(1.17645E9)        // L5/E5a frequency (Hz)
#define FREQ6           D(1.27875E9)        // E6/LEX frequency (Hz)
#define FREQ7           D(1.20714E9)        // E5b    frequency (Hz)
#define FREQ8           D(1.191795E9)       // E5a+b  frequency (Hz)
#define FREQ1_GLO       D(1.60200E9)        // GLONASS L1 base frequency (Hz)
#define DFRQ1_GLO       D(0.56250E6)        // GLONASS L1 bias frequency (Hz/n)
#define FREQ2_GLO       D(1.24600E9)        // GLONASS L2 base frequency (Hz)
#define DFRQ2_GLO       D(0.43750E6)        // GLONASS L2 bias frequency (Hz/n)
#define SC2RAD          D(3.1415926535898)  // semi-circle to radian (IS-GPS)
#define PRUNIT_GPS      D(299792.458)       // rtcm 3 unit of gps pseudorange (m)
#define PRUNIT_GLO      D(599584.916)       // rtcm ver.3 unit of glonass pseudorange (m)
#define FE_WGS84        (D(1.0)/D(298.257223563)) // earth flattening (WGS84)
#define RE_WGS84        D(6378137.0)           // earth semimajor axis (WGS84) (m)

#define P2_5        D(0.03125)                 // 2^-5
#define P2_19       D(1.907348632812500E-06)   // 2^-19
#define P2_29       D(1.862645149230957E-09)   // 2^-29
#define P2_31       D(4.656612873077393E-10)   // 2^-31
#define P2_33       D(1.164153218269348E-10)   // 2^-33
#define P2_43       D(1.136868377216160E-13)   // 2^-43
#define P2_55       D(2.775557561562891E-17)   // 2^-55

// Private variables
const double lam_carr[] = { // carrier wave length (m)
                            CLIGHT/FREQ1,
                            CLIGHT/FREQ2,
                            CLIGHT/FREQ5,
                            CLIGHT/FREQ6,
                            CLIGHT/FREQ7,
                            CLIGHT/FREQ8
                          };

// TODO: Fix this properly!!
static int last_wn = 1874;

// Private functions
static int encode_head(rtcm_obs_header_t *header, int nsat, int sys,
                       uint8_t *buffer, double *tadj);
static int decode_head1001(rtcm_obs_header_t *header, int *nsat, uint8_t *buffer);
static int decode_head1009(rtcm_obs_header_t *header, int *nsat, uint8_t *buffer);
static int encode_end(uint8_t *buffer, int nbit);
static int decode_1002(rtcm3_state *state);
static int decode_1004(rtcm3_state *state);
static int decode_1005(rtcm3_state *state);
static int decode_1006(rtcm3_state *state);
static int decode_1010(rtcm3_state *state);
static int decode_1012(rtcm3_state *state);
static int decode_1019(rtcm3_state *state);
static double cp_pr(double cp, double pr_cyc);
static void setbitu(uint8_t *buff, int pos, int len, unsigned int data);
static void setbits(unsigned char *buff, int pos, int len, int data);
static void set38bits(unsigned char *buff, int pos, double value);
static unsigned int getbitu(const unsigned char *buff, int pos, int len);
static int getbits(const unsigned char *buff, int pos, int len);
static double getbits_38(const unsigned char *buff, int pos);
static unsigned int crc24q(const unsigned char *buff, int len);

/**
 * @brief rtcm3_set_rx_callback_obs_gps
 * Set a function to be called when a 1001 - 1004 packet is received.
 */
void rtcm3_set_rx_callback_obs(void(*func)(rtcm_obs_header_t *header, rtcm_obs_t *obs, int obs_num), rtcm3_state *state) {
    state->rx_rtcm_obs = func;
}

/**
 * @brief rtcm3_set_rx_callback_1006
 * Set a function to be called when a 1006 packet is received.
 */
void rtcm3_set_rx_callback_1005_1006(void(*func)(rtcm_ref_sta_pos_t *pos), rtcm3_state *state) {
    state->rx_rtcm_1005_1006 = func;
}

/**
 * @brief rtcm3_set_rx_callback_1019
 * Set a function to be called when a 1019 packet is received.
 */
void rtcm3_set_rx_callback_1019(void(*func)(rtcm_ephemeris_t *eph), rtcm3_state *state) {
    state->rx_rtcm_1019 = func;
}

/**
 * @brief rtcm3_set_rx_callback
 * Set a function to be called when any rtcm packet is received.
 */
void rtcm3_set_rx_callback(void (*func)(uint8_t *, int, int), rtcm3_state *state) {
    state->rx_rtcm = func;
}

/**
 * Initialize the state of the rtcm3 decoder.
 *
 * @param state
 * The state to initialize.
 */
void rtcm3_init_state(rtcm3_state *state) {
    memset(state, 0, sizeof(rtcm3_state));
}

/**
 * @brief rtcm3_input_data
 * Decode RTCM3 data.
 *
 * @param data
 * The byte to put into the state machine.
 *
 * @param state
 * Pointer to the state of the RTCM decoder.
 *
 * @return
 * xxxx: Message xxxx decoded.
 * 0: Byte received.
 * -1: Wrong preamble
 * -2: Wrong crc
 */
int rtcm3_input_data(uint8_t data, rtcm3_state *state) {
    // synchronize frame
    if (state->buffer_ptr == 0) {
        if (data != RTCM3PREAMB) {
            return -1;
        }

        state->buffer[state->buffer_ptr++] = data;
        return 0;
    }

    state->buffer[state->buffer_ptr++]=data;

    if (state->buffer_ptr == 3) {
        state->len = getbitu(state->buffer, 14, 10) + 3; // length without crc
    }

    if (state->buffer_ptr < 3 || state->buffer_ptr < state->len + 3) {
        return 0;
    }

    state->buffer_ptr = 0;

    // check crc
    if (crc24q(state->buffer, state->len) != getbitu(state->buffer, state->len * 8, 24)) {
        return -2;
    }

    // decode rtcm3 message
    int type = getbitu(state->buffer, 24, 12);

    if (state->rx_rtcm) {
        // Send buffer with CRC included
        state->rx_rtcm(state->buffer, state->len + 3, type);
    }

    switch (type) {
    case 1002:
        if (state->rx_rtcm_obs || state->decode_all) {
            decode_1002(state);
        }
        break;

    case 1004:
        if (state->rx_rtcm_obs || state->decode_all) {
            decode_1004(state);
        }
        break;

    case 1005:
        if (state->rx_rtcm_1005_1006 || state->decode_all) {
            decode_1005(state);
        }
        break;

    case 1006:
        if (state->rx_rtcm_1005_1006 || state->decode_all) {
            decode_1006(state);
        }
        break;

    case 1010:
        if (state->rx_rtcm_obs || state->decode_all) {
            decode_1010(state);
        }
        break;

    case 1012:
        if (state->rx_rtcm_obs || state->decode_all) {
            decode_1012(state);
        }
        break;

    case 1019:
        if (state->rx_rtcm_1019 || state->decode_all) {
            decode_1019(state);
        }
        break;

    default:
        // Not supported
        break;
    }

    return type;
}

/**
 * @brief rtcm3_encode_1002
 * Encode RTCM3 GPS L1 observation with extended information.
 *
 * @param header
 * RTCM header.
 *
 * @param obs
 * Observation data.
 *
 * @param obs_num
 * Number of observations.
 *
 * @param buffer
 * Buffer to store the RTCM stream to.
 *
 * @param buffer_len
 * Length of the buffer.
 *
 * @return
 * 1 for success, <= 0 otherwise.
 */
int rtcm3_encode_1002(rtcm_obs_header_t *header, rtcm_obs_t *obs,
                      int obs_num, uint8_t *buffer, int *buffer_len) {
    int i, j, prn;
    int code1, pr1, ppr1, lock1, amb, cnr1;
    double tadj;

    // encode header
    header->type = 1002;
    i = encode_head(header, obs_num, SYS_GPS, buffer, &tadj);

    for (j=0;j < obs_num;j++) {
        double lam1, pr1c = 0.0, ppr;
        double P0, L0;

        lam1 = CLIGHT / FREQ1;
        pr1 = 0;
        amb = 0;
        ppr1 = 0xFFF80000;

        // See https://rtklibexplorer.wordpress.com/2017/02/01/a-fix-for-the-rtcm-time-tag-issue/
        P0 = obs[j].P[0] == D(0.0) ? D(0.0) : obs[j].P[0] - tadj * CLIGHT;
        L0 = obs[j].L[0] == D(0.0) ? D(0.0) : obs[j].L[0] - tadj * FREQ1;

        // L1 peudorange
        amb = (int)floor(P0 / PRUNIT_GPS);
        pr1 = ROUND((P0 - amb * PRUNIT_GPS) / D(0.02));
        pr1c = pr1 * D(0.02) + amb * PRUNIT_GPS;

        // L1 phaserange - L1 pseudorange
        ppr = cp_pr(L0, pr1c / lam1);
        ppr1 = ROUND(ppr * lam1 / D(0.0005));

        lock1 = obs[j].lock[0];
        cnr1 = obs[j].cn0[0] * 4;
        prn = obs[j].prn;

        if (obs[j].code[0] == CODE_L1C) {
            code1 = 0;
        } else {
            code1 = 1;
        }

        setbitu(buffer,i, 6,prn  ); i+= 6;
        setbitu(buffer,i, 1,code1); i+= 1;
        setbitu(buffer,i,24,pr1  ); i+=24;
        setbits(buffer,i,20,ppr1 ); i+=20;
        setbitu(buffer,i, 7,lock1); i+= 7;
        setbitu(buffer,i, 8,amb  ); i+= 8;
        setbitu(buffer,i, 8,cnr1 ); i+= 8;
    }

    *buffer_len = encode_end(buffer, i);

    return *buffer_len > 0;
}

/**
 * @brief rtcm3_encode_1010
 * Encode RTCM3 GLONASS L1 observation with extended information.
 *
 * @param header
 * RTCM header.
 *
 * @param obs
 * Observation data.
 *
 * @param obs_num
 * Number of observations.
 *
 * @param buffer
 * Buffer to store the RTCM stream to.
 *
 * @param buffer_len
 * Length of the buffer.
 *
 * @return
 * 1 for success, <= 0 otherwise.
 */
int rtcm3_encode_1010(rtcm_obs_header_t *header, rtcm_obs_t *obs,
                      int obs_num, uint8_t *buffer, int *buffer_len)
{
    int i, j, prn, fcn;
    int code1, pr1, ppr1, lock1, amb, cnr1;
    double tadj;

    // encode header
    header->type = 1010;
    i = encode_head(header, obs_num, SYS_GLO, buffer, &tadj);

    for (j = 0;j < obs_num;j++) {
        double lam1, pr1c = 0.0, ppr;
        double P0, L0, freq1;

        freq1 = FREQ1_GLO + DFRQ1_GLO * ((double)(obs[j].freq) - D(7.0));
        lam1 = CLIGHT / freq1;
        pr1 = 0;
        amb = 0;
        ppr1 = 0xFFF80000;

        // See https://rtklibexplorer.wordpress.com/2017/02/01/a-fix-for-the-rtcm-time-tag-issue/
        P0 = obs[j].P[0] == D(0.0) ? D(0.0) : obs[j].P[0] - tadj * CLIGHT;
        L0 = obs[j].L[0] == D(0.0) ? D(0.0) : obs[j].L[0] - tadj * freq1;

        // L1 peudorange
        amb = (int)floor(P0 / PRUNIT_GLO);
        pr1 = ROUND((P0 - amb * PRUNIT_GLO) / D(0.02));
        pr1c = pr1 * D(0.02) + amb * PRUNIT_GLO;

        // L1 phaserange - L1 pseudorange
        ppr = cp_pr(L0, pr1c / lam1);
        ppr1 = ROUND(ppr * lam1 / D(0.0005));

        lock1 = obs[j].lock[0];
        cnr1 = obs[j].cn0[0] * 4;
        prn = obs[j].prn;

        if (obs[j].code[0] == CODE_L1C) {
            code1 = 0;
        } else {
            code1 = 1;
        }

        fcn = obs[j].freq;

        setbitu(buffer,i, 6,prn  ); i+= 6;
        setbitu(buffer,i, 1,code1); i+= 1;
        setbitu(buffer,i, 5,fcn  ); i+= 5;
        setbitu(buffer,i,25,pr1  ); i+=25;
        setbits(buffer,i,20,ppr1 ); i+=20;
        setbitu(buffer,i, 7,lock1); i+= 7;
        setbitu(buffer,i, 7,amb  ); i+= 7;
        setbitu(buffer,i, 8,cnr1 ); i+= 8;
    }

    *buffer_len = encode_end(buffer, i);

    return *buffer_len > 0;
}

/**
 * @brief rtcm3_encode_1006
 * Encode RTCM3 reference station position with extended information.
 *
 * @param pos
 * The reference station position.
 *
 * @param buffer
 * Buffer to store the RTCM stream to.
 *
 * @param buffer_len
 * Length of the buffer.
 *
 * @return
 * 1 for success, <= 0 otherwise.
 */
int rtcm3_encode_1006(rtcm_ref_sta_pos_t pos, uint8_t *buffer, int *buffer_len) {
    int i=0;
    int hgt = ROUND(pos.ant_height / D(0.0001));

    // Convert llh to ecef
    double sinp = sin(pos.lat * D_PI / D(180.0));
    double cosp = cos(pos.lat * D_PI / D(180.0));
    double sinl = sin(pos.lon * D_PI / D(180.0));
    double cosl = cos(pos.lon * D_PI / D(180.0));
    double e2 = FE_WGS84 * (D(2.0) - FE_WGS84);
    double v = RE_WGS84 / sqrt(D(1.0) - e2 * sinp * sinp);

    double p0 = (v + pos.height) * cosp * cosl;
    double p1 = (v + pos.height) * cosp * sinl;
    double p2 = (v * (D(1.0) - e2) + pos.height) * sinp;

    // set preamble and reserved
    setbitu(buffer,i, 8, RTCM3PREAMB); i+= 8;
    setbitu(buffer,i, 6, 0); i+= 6;
    setbitu(buffer, i, 10, 0); i+=10;

    setbitu(buffer,i,12,1006       ); i+=12; // message no
    setbitu(buffer,i,12,pos.staid  ); i+=12; // ref station id
    setbitu(buffer,i, 6,0          ); i+= 6; // itrf realization year
    setbitu(buffer,i, 1,1          ); i+= 1; // gps indicator
    setbitu(buffer,i, 1,1          ); i+= 1; // glonass indicator
    setbitu(buffer,i, 1,0          ); i+= 1; // galileo indicator
    setbitu(buffer,i, 1,0          ); i+= 1; // ref station indicator
    set38bits(buffer,i,p0 / D(0.0001)); i+=38; // antenna ref point ecef-x
    setbitu(buffer,i, 1,1          ); i+= 1; // oscillator indicator
    setbitu(buffer,i, 1,0          ); i+= 1; // reserved
    set38bits(buffer,i,p1 / D(0.0001)); i+=38; // antenna ref point ecef-y
    setbitu(buffer,i, 2,0          ); i+= 2; // quarter cycle indicator
    set38bits(buffer,i,p2 / D(0.0001)); i+=38; // antenna ref point ecef-z
    setbitu(buffer,i,16,hgt        ); i+=16; // antenna height

    *buffer_len = encode_end(buffer, i);

    return *buffer_len > 0;
}

/**
 * @brief rtcm3_encode_1019
 * Encode RTCM3 GPS ephemeris.
 *
 * @param eph
 * Ephemeris data.
 *
 * @param buffer
 * Buffer to store the RTCM stream to.
 *
 * @param buffer_len
 * Length of the buffer.
 *
 * @return
 * 1 for success, <= 0 otherwise.
 */
int rtcm3_encode_1019(rtcm_ephemeris_t *eph, uint8_t *buffer, int *buffer_len) {
    int i=0;
    unsigned int sqrtA,e;
    int week,toe,toc,i0,OMG0,omg,M0,deln,idot,OMGd,crs,crc;
    int cus,cuc,cis,cic,af0,af1,af2,tgd;

    // set preamble and reserved
    setbitu(buffer,i, 8, RTCM3PREAMB); i+= 8;
    setbitu(buffer,i, 6, 0); i+= 6;
    setbitu(buffer, i, 10, 0); i+=10;

    week  = eph->toe_wn % 1024;
    toe   = ROUND(eph->toe_tow / D(16.0));
    toc   = ROUND(eph->toc_tow / D(16.0));
    sqrtA = ROUND_U(eph->sqrta / P2_19);
    e     = ROUND_U(eph->ecc / P2_33);
    i0    = ROUND(eph->inc  / P2_31 / SC2RAD);
    OMG0  = ROUND(eph->omega0 / P2_31 / SC2RAD);
    omg   = ROUND(eph->w / P2_31 / SC2RAD);
    M0    = ROUND(eph->m0 / P2_31 / SC2RAD);
    deln  = ROUND(eph->dn / P2_43 / SC2RAD);
    idot  = ROUND(eph->inc_dot / P2_43 / SC2RAD);
    OMGd  = ROUND(eph->omegadot / P2_43 / SC2RAD);
    crs   = ROUND(eph->c_rs / P2_5 );
    crc   = ROUND(eph->c_rc / P2_5 );
    cus   = ROUND(eph->c_us / P2_29);
    cuc   = ROUND(eph->c_uc / P2_29);
    cis   = ROUND(eph->c_is / P2_29);
    cic   = ROUND(eph->c_ic / P2_29);
    af0   = ROUND(eph->af0 / P2_31);
    af1   = ROUND(eph->af1 / P2_43);
    af2   = ROUND(eph->af2 / P2_55);
    tgd   = ROUND(eph->tgd / P2_31);

    setbitu(buffer,i,12,1019     ); i+=12;
    setbitu(buffer,i, 6,eph->prn ); i+= 6;
    setbitu(buffer,i,10,week     ); i+=10;
    setbitu(buffer,i, 4,eph->sva ); i+= 4;
    setbitu(buffer,i, 2,eph->code); i+= 2;
    setbits(buffer,i,14,idot     ); i+=14;
    setbitu(buffer,i, 8,eph->iode); i+= 8;
    setbitu(buffer,i,16,toc      ); i+=16;
    setbits(buffer,i, 8,af2      ); i+= 8;
    setbits(buffer,i,16,af1      ); i+=16;
    setbits(buffer,i,22,af0      ); i+=22;
    setbitu(buffer,i,10,eph->iodc); i+=10;
    setbits(buffer,i,16,crs      ); i+=16;
    setbits(buffer,i,16,deln     ); i+=16;
    setbits(buffer,i,32,M0       ); i+=32;
    setbits(buffer,i,16,cuc      ); i+=16;
    setbitu(buffer,i,32,e        ); i+=32;
    setbits(buffer,i,16,cus      ); i+=16;
    setbitu(buffer,i,32,sqrtA    ); i+=32;
    setbitu(buffer,i,16,toe      ); i+=16;
    setbits(buffer,i,16,cic      ); i+=16;
    setbits(buffer,i,32,OMG0     ); i+=32;
    setbits(buffer,i,16,cis      ); i+=16;
    setbits(buffer,i,32,i0       ); i+=32;
    setbits(buffer,i,16,crc      ); i+=16;
    setbits(buffer,i,32,omg      ); i+=32;
    setbits(buffer,i,24,OMGd     ); i+=24;
    setbits(buffer,i, 8,tgd      ); i+= 8;
    setbitu(buffer,i, 6,eph->svh ); i+= 6;
    setbitu(buffer,i, 1,eph->flag); i+= 1;
    setbitu(buffer,i, 1,eph->fit>D(0.0)?D(0):D(1)); i+=1;

    *buffer_len = encode_end(buffer, i);

    return *buffer_len > 0;
}

static int encode_head(rtcm_obs_header_t *header, int nsat, int sys,
                       uint8_t *buffer, double *tadj) {
    int i=0, epoch;

    // set preamble and reserved
    setbitu(buffer,i, 8, RTCM3PREAMB); i+= 8;
    setbitu(buffer,i, 6, 0); i+= 6;
    setbitu(buffer, i, 10, 0); i+=10;

    setbitu(buffer,i,12,header->type); i+=12; // message type
    setbitu(buffer,i,12,header->staid); i+=12; // ref station id

    if (sys == SYS_GLO) {
        epoch = ROUND(header->t_tod / D(0.001));
        *tadj = (header->t_tod / D(0.001) - epoch) * D(0.001);
        setbitu(buffer,i,27,epoch); i += 27; // glonass epoch time
    } else {
        epoch = ROUND(header->t_tow / D(0.001));
        *tadj = (header->t_tow / D(0.001) - epoch) * D(0.001);
        setbitu(buffer, i, 30, epoch); i += 30; // gps epoch time
    }

    setbitu(buffer, i, 1, header->sync); i+= 1; // synchronous gnss flag
    setbitu(buffer, i, 5, nsat); i += 5; // no of satellites
    setbitu(buffer, i, 1, 0   ); i += 1; // smoothing indicator
    setbitu(buffer, i, 3, 0   ); i += 3; // smoothing interval
    return i;
}

// decode type 1001-1004 message header
static int decode_head1001(rtcm_obs_header_t *header, int *nsat, uint8_t *buffer) {
    int i = 24;

    header->type = getbitu(buffer, i, 12);          i+=12;
    header->staid = getbitu(buffer, i, 12);         i+=12;
    header->t_tow = getbitu(buffer, i, 30) * D(0.001); i+=30;
    header->sync  = getbitu(buffer, i, 1);          i+=1;
    *nsat = getbitu(buffer, i, 5);                  i+=5;

    header->t_wn = last_wn;

    return i;
}

// decode type 1009-1012 message header
static int decode_head1009(rtcm_obs_header_t *header, int *nsat, uint8_t *buffer) {
    int i = 24;

    header->type = getbitu(buffer, i, 12);          i+=12;
    header->staid = getbitu(buffer, i, 12);         i+=12;
    header->t_tod = getbitu(buffer, i, 27) * D(0.001); i+=27;
    header->sync  = getbitu(buffer, i, 1);          i+=1;
    *nsat = getbitu(buffer, i, 5);                  i+=5;

    header->t_wn = last_wn;

    return i;
}

static int encode_end(uint8_t *buffer, int nbit) {
    int len, i, crc;

    // padding to align 8 bit boundary
    for (i = nbit;i % 8;i++) {
        setbitu(buffer, i, 1, 0);
    }

    // message length (header+data) (bytes)
    len = i / 8;
    if (len >= 3+1024) {
        return 0;
    }

    // message length without header and parity
    setbitu(buffer, 14, 10, len - 3);

    // crc-24q
    crc=crc24q(buffer, len);
    setbitu(buffer, i, 24, crc);

    // Return length total (bytes)
    return len + 3;
}

static int decode_1002(rtcm3_state *state) {
    double pr1,cnr1,cp1;
    int i=24+64,j,nsat,prn,code,ppr1,lock1,amb;

    decode_head1001(&state->header, &nsat, state->buffer);

    for (j=0;j < nsat && i + 74 <= state->len * 8;j++) {
        prn  =getbitu(state->buffer,i, 6); i+= 6;
        code =getbitu(state->buffer,i, 1); i+= 1;
        pr1  =getbitu(state->buffer,i,24); i+=24;
        ppr1 =getbits(state->buffer,i,20); i+=20;
        lock1=getbitu(state->buffer,i, 7); i+= 7;
        amb  =getbitu(state->buffer,i, 8); i+= 8;
        cnr1 =getbitu(state->buffer,i, 8); i+= 8;

        pr1 = pr1 * D(0.02) + amb * PRUNIT_GPS;

        if (ppr1 != (int)0xFFF80000) {
            state->obs[j].P[0] = pr1;
            cp1 = ppr1 * D(0.0005) / lam_carr[0];
            state->obs[j].L[0]=pr1/lam_carr[0] + cp1;
        }

        state->obs[j].prn = prn;
        state->obs[j].lock[0] = lock1;
        state->obs[j].cn0[0] = cnr1 * D(0.25);
        state->obs[j].code[0] = code ? CODE_L1P : CODE_L1C;
    }

    // Call callback if it is set
    if (state->rx_rtcm_obs) {
        state->rx_rtcm_obs(&state->header, state->obs, nsat);
    }

    return 1004;
}

static int decode_1004(rtcm3_state *state) {
    const int L2codes[]={CODE_L2C,CODE_L2P,CODE_L2W,CODE_L2W};

    double pr1, cnr1, cnr2, cp1, cp2;
    int i=24+64, j, nsat, prn, code1, code2, pr21, ppr1, ppr2;
    int lock1, lock2, amb;

    decode_head1001(&state->header, &nsat, state->buffer);

    for (j = 0;j < nsat && i + 125 <= state->len * 8;j++) {
        prn   = getbitu(state->buffer,i, 6); i+= 6;
        code1 = getbitu(state->buffer,i, 1); i+= 1;
        pr1   = getbitu(state->buffer,i,24); i+=24;
        ppr1  = getbits(state->buffer,i,20); i+=20;
        lock1 = getbitu(state->buffer,i, 7); i+= 7;
        amb   = getbitu(state->buffer,i, 8); i+= 8;
        cnr1  = getbitu(state->buffer,i, 8); i+= 8;
        code2 = getbitu(state->buffer,i, 2); i+= 2;
        pr21  = getbits(state->buffer,i,14); i+=14;
        ppr2  = getbits(state->buffer,i,20); i+=20;
        lock2 = getbitu(state->buffer,i, 7); i+= 7;
        cnr2  = getbitu(state->buffer,i, 8); i+= 8;

        pr1 = pr1 * D(0.02) + amb * PRUNIT_GPS;

        if (ppr1!=(int)0xFFF80000) {
            state->obs[j].P[0] = pr1;
            cp1 = ppr1 * D(0.0005) / lam_carr[0];
            state->obs[j].L[0] = pr1 / lam_carr[0] + cp1;
        }

        state->obs[j].prn = prn;
        state->obs[j].lock[0] = lock1;
        state->obs[j].cn0[0] = cnr1 * D(0.25);
        state->obs[j].code[0] = code1 ? CODE_L1P : CODE_L1C;

        if (pr21 != (int)0xFFFFE000) {
            state->obs[j].P[1] = pr1 + pr21 * D(0.02);
        }

        if (ppr2!=(int)0xFFF80000) {
            cp2 = ppr2 * D(0.0005) / lam_carr[1];
            state->obs[j].L[1] = pr1 / lam_carr[1] + cp2;
        }

        state->obs[j].lock[1] = lock2;
        state->obs[j].cn0[1] = cnr2 * D(0.25);
        state->obs[j].code[1] = L2codes[code2];
    }

    // Call callback if it is set
    if (state->rx_rtcm_obs) {
        state->rx_rtcm_obs(&state->header, state->obs, nsat);
    }

    return 1004;
}

static int decode_1005(rtcm3_state *state) {
    double p0 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    int i = 24 + 12;
    int staid;
    int itrf;

    if (i + 140 <= state->len * 8) {
        staid = getbitu(state->buffer, i, 12); i+=12;
        itrf  = getbitu(state->buffer, i, 6);  i+= 6+4;
        p0    = getbits_38(state->buffer, i);  i+=38+2;
        p1    = getbits_38(state->buffer, i);  i+=38+2;
        p2    = getbits_38(state->buffer, i);

        p0 *= D(0.0001);
        p1 *= D(0.0001);
        p2 *= D(0.0001);

        (void)itrf;
        state->pos.ant_height = 0.0;
        state->pos.staid = staid;

        // Convert ecef to llh
        double e2 = FE_WGS84 * (D(2.0) - FE_WGS84);
        double r2 = p0 * p0 + p1 * p1;
        double z = p2;
        double zk = 0.0;
        double sinp = 0.0;
        double v = RE_WGS84;

        while (fabs(z - zk) >= D(1E-4)) {
            zk = z;
            sinp = z / sqrt(r2 + z * z);
            v = RE_WGS84 / sqrt(D(1.0) - e2 * sinp * sinp);
            z = p2 + v * e2 * sinp;
        }

        state->pos.lat = (r2 > D(1E-12) ? atan(z / sqrt(r2)) : (p2 > D(0.0) ? D_PI / D(2.0) : -D_PI / D(2.0))) * D(180.0) / D_PI;
        state->pos.lon = (r2 > D(1E-12) ? atan2(p1, p0) : D(0.0)) * D(180.0) / D_PI;
        state->pos.height = sqrt(r2 + z * z) - v;

        if (state->rx_rtcm_1005_1006) {
            state->rx_rtcm_1005_1006(&state->pos);
        }
    }

    return 1005;
}

static int decode_1006(rtcm3_state *state) {
    double p0 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    double anth;
    int i = 24 + 12;
    int staid;
    int itrf;

    if (i + 156 <= state->len * 8) {
        staid = getbitu(state->buffer, i, 12); i+=12;
        itrf  = getbitu(state->buffer, i, 6);  i+= 6+4;
        p0    = getbits_38(state->buffer, i);  i+=38+2;
        p1    = getbits_38(state->buffer, i);  i+=38+2;
        p2    = getbits_38(state->buffer, i);  i+=38;
        anth  = getbitu(state->buffer, i, 16);

        p0 *= D(0.0001);
        p1 *= D(0.0001);
        p2 *= D(0.0001);

        (void)itrf;
        state->pos.ant_height = anth * D(0.0001);
        state->pos.staid = staid;

        // Convert ecef to llh
        double e2 = FE_WGS84 * (D(2.0) - FE_WGS84);
        double r2 = p0 * p0 + p1 * p1;
        double z = p2;
        double zk = 0.0;
        double sinp = 0.0;
        double v = RE_WGS84;

        while (fabs(z - zk) >= D(1E-4)) {
            zk = z;
            sinp = z / sqrt(r2 + z * z);
            v = RE_WGS84 / sqrt(D(1.0) - e2 * sinp * sinp);
            z = p2 + v * e2 * sinp;
        }

        state->pos.lat = (r2 > D(1E-12) ? atan(z / sqrt(r2)) : (p2 > D(0.0) ? D_PI / D(2.0) : -D_PI / D(2.0))) * D(180.0) / D_PI;
        state->pos.lon = (r2 > D(1E-12) ? atan2(p1, p0) : D(0.0)) * D(180.0) / D_PI;
        state->pos.height = sqrt(r2 + z * z) - v;

        if (state->rx_rtcm_1005_1006) {
            state->rx_rtcm_1005_1006(&state->pos);
        }
    }

    return 1006;
}

static int decode_1010(rtcm3_state *state) {
    double pr1,cnr1,cp1,lam1;
    int i=24+61,j,nsat,prn,code,freq,ppr1,lock1,amb;

    decode_head1009(&state->header, &nsat, state->buffer);

    for (j=0;j < nsat && i + 79 <= state->len * 8;j++) {
        prn  =getbitu(state->buffer,i, 6); i+= 6;
        code =getbitu(state->buffer,i, 1); i+= 1;
        freq =getbitu(state->buffer,i, 5); i+= 5;
        pr1  =getbitu(state->buffer,i,25); i+=25;
        ppr1 =getbits(state->buffer,i,20); i+=20;
        lock1=getbitu(state->buffer,i, 7); i+= 7;
        amb  =getbitu(state->buffer,i, 7); i+= 7;
        cnr1 =getbitu(state->buffer,i, 8); i+= 8;

        pr1 = pr1 * D(0.02) + amb * PRUNIT_GLO;

        if (ppr1 != (int)0xFFF80000) {
            state->obs[j].P[0] = pr1;
            lam1 = CLIGHT / (FREQ1_GLO + DFRQ1_GLO * (freq - 7));
            cp1 = ppr1 * D(0.0005) / lam1;
            state->obs[j].L[0] = pr1 / lam1 + cp1;
        }

        state->obs[j].prn = prn;
        state->obs[j].lock[0] = lock1;
        state->obs[j].cn0[0] = cnr1 * D(0.25);
        state->obs[j].code[0] = code ? CODE_L1P : CODE_L1C;
        state->obs[j].freq = freq;
    }

    // Call callback if it is set
    if (state->rx_rtcm_obs) {
        state->rx_rtcm_obs(&state->header, state->obs, nsat);
    }

    return 1010;
}

static int decode_1012(rtcm3_state *state) {
    double pr1, cnr1, cnr2, cp1, cp2, lam1, lam2;
    int i=24+61, j, nsat, prn, freq, code1, code2, pr21, ppr1, ppr2;
    int lock1, lock2, amb;

    decode_head1009(&state->header, &nsat, state->buffer);

    for (j = 0;j < nsat && i + 130 <= state->len * 8;j++) {
        prn   = getbitu(state->buffer,i, 6); i+= 6;
        code1 = getbitu(state->buffer,i, 1); i+= 1;
        freq  = getbitu(state->buffer,i, 5); i+= 5;
        pr1   = getbitu(state->buffer,i,25); i+=25;
        ppr1  = getbits(state->buffer,i,20); i+=20;
        lock1 = getbitu(state->buffer,i, 7); i+= 7;
        amb   = getbitu(state->buffer,i, 7); i+= 7;
        cnr1  = getbitu(state->buffer,i, 8); i+= 8;
        code2 = getbitu(state->buffer,i, 2); i+= 2;
        pr21  = getbits(state->buffer,i,14); i+=14;
        ppr2  = getbits(state->buffer,i,20); i+=20;
        lock2 = getbitu(state->buffer,i, 7); i+= 7;
        cnr2  = getbitu(state->buffer,i, 8); i+= 8;

        pr1 = pr1 * D(0.02) + amb * PRUNIT_GLO;

        if (ppr1 != (int)0xFFF80000) {
            state->obs[j].P[0] = pr1;
            lam1 = CLIGHT / (FREQ1_GLO + DFRQ1_GLO * (freq - 7));
            cp1 = ppr1 * D(0.0005) / lam1;
            state->obs[j].L[0] = pr1 / lam1 + cp1;
        }

        state->obs[j].prn = prn;
        state->obs[j].lock[0] = lock1;
        state->obs[j].cn0[0] = cnr1 * D(0.25);
        state->obs[j].code[0] = code1 ? CODE_L1P : CODE_L1C;

        if (pr21 != (int)0xFFFFE000) {
            state->obs[j].P[1] = pr1 + pr21 * D(0.02);
        }

        if (ppr2 != (int)0xFFF80000) {
            lam2 = CLIGHT / (FREQ2_GLO + DFRQ2_GLO * (freq - 7));
            cp2 = ppr2 * D(0.0005) / lam2;
            state->obs[j].L[1] = pr1 / lam2 + cp2;
        }

        state->obs[j].lock[1] = lock2;
        state->obs[j].cn0[1] = cnr2 * D(0.25);
        state->obs[j].code[1] = code2 ? CODE_L2P : CODE_L2C;
        state->obs[j].freq = freq;
    }

    // Call callback if it is set
    if (state->rx_rtcm_obs) {
        state->rx_rtcm_obs(&state->header, state->obs, nsat);
    }

    return 1012;
}

static int decode_1019(rtcm3_state *state) {
    int i = 24 + 12;
    int week;

    if (i + 476 <= state->len * 8) {
        state->eph.prn   =getbitu(state->buffer, i, 6);              i+= 6;
        week      =getbitu(state->buffer, i,10);              i+=10;
        state->eph.sva   =getbitu(state->buffer, i, 4);              i+= 4;
        state->eph.code  =getbitu(state->buffer, i, 2);              i+= 2;
        state->eph.inc_dot=getbits(state->buffer, i,14)*P2_43*SC2RAD;i+=14;
        state->eph.iode  =getbitu(state->buffer, i, 8);              i+= 8;
        state->eph.toc_tow=getbitu(state->buffer, i,16)*16.0;        i+=16;
        state->eph.af2   =getbits(state->buffer, i, 8)*P2_55;        i+= 8;
        state->eph.af1   =getbits(state->buffer, i,16)*P2_43;        i+=16;
        state->eph.af0   =getbits(state->buffer, i,22)*P2_31;        i+=22;
        state->eph.iodc  =getbitu(state->buffer, i,10);              i+=10;
        state->eph.c_rs  =getbits(state->buffer, i,16)*P2_5;         i+=16;
        state->eph.dn    =getbits(state->buffer, i,16)*P2_43*SC2RAD; i+=16;
        state->eph.m0    =getbits(state->buffer, i,32)*P2_31*SC2RAD; i+=32;
        state->eph.c_uc  =getbits(state->buffer, i,16)*P2_29;        i+=16;
        state->eph.ecc   =getbitu(state->buffer, i,32)*P2_33;        i+=32;
        state->eph.c_us  =getbits(state->buffer, i,16)*P2_29;        i+=16;
        state->eph.sqrta =getbitu(state->buffer, i,32)*P2_19;        i+=32;
        state->eph.toe_tow=getbitu(state->buffer, i,16)*16.0;        i+=16;
        state->eph.c_ic  =getbits(state->buffer, i,16)*P2_29;        i+=16;
        state->eph.omega0=getbits(state->buffer, i,32)*P2_31*SC2RAD; i+=32;
        state->eph.c_is  =getbits(state->buffer, i,16)*P2_29;        i+=16;
        state->eph.inc   =getbits(state->buffer, i,32)*P2_31*SC2RAD; i+=32;
        state->eph.c_rc  =getbits(state->buffer, i,16)*P2_5;         i+=16;
        state->eph.w     =getbits(state->buffer, i,32)*P2_31*SC2RAD; i+=32;
        state->eph.omegadot=getbits(state->buffer, i,24)*P2_43*SC2RAD;i+=24;
        state->eph.tgd   =getbits(state->buffer, i, 8)*P2_31;        i+= 8;
        state->eph.svh   =getbitu(state->buffer, i, 6);              i+= 6;
        state->eph.flag  =getbitu(state->buffer, i, 1);              i+= 1;
        state->eph.fit   =getbitu(state->buffer, i, 1) ? 0.0 : 4.0; // 0:4hr,1:>4hr

        // TODO: Is this correct??
        week += (1760 - week + 512) / 1024 * 1024;
        state->eph.toe_wn = week;

        last_wn = week;

        if (state->rx_rtcm_1019) {
            state->rx_rtcm_1019(&state->eph);
        }
    }

    return 1019;
}

// carrier-phase - pseudorange in cycle
static double cp_pr(double cp, double pr_cyc) {
    return fmod(cp - pr_cyc + D(1500.0), D(3000.0)) - D(1500.0);
}

static void setbitu(uint8_t *buff, int pos, int len, unsigned int data) {
    unsigned int mask = 1u << (len - 1);
    int i;

    if (len <= 0 || len > 32) {
        return;
    }

    for (i=pos;i < pos+len;i++,mask >>= 1) {
        if (data & mask) {
            buff[i / 8] |= 1u << (7 - i % 8);
        } else {
            buff[i / 8] &= ~(1u << (7 - i % 8));
        }
    }
}

static void setbits(unsigned char *buff, int pos, int len, int data) {
    if (data < 0) {
        data |= 1 << (len - 1);
    } else {
        data &= ~(1 << (len - 1)); // set sign bit
    }

    setbitu(buff,pos,len,(unsigned int)data);
}

// set signed 38 bit field
static void set38bits(unsigned char *buff, int pos, double value) {
    int word_h = (int)floor(value / D(64.0));
    unsigned int word_l = (unsigned int)(value - word_h * D(64.0));
    setbits(buff, pos  , 32, word_h);
    setbitu(buff, pos + 32, 6, word_l);
}

static unsigned int getbitu(const unsigned char *buff, int pos, int len) {
    unsigned int bits=0;
    int i;

    for (i = pos;i < pos + len;i++) {
        bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
    }

    return bits;
}

static int getbits(const unsigned char *buff, int pos, int len) {
    unsigned int bits = getbitu(buff, pos, len);

    if (len <= 0 || 32 <= len || !(bits & (1u << (len - 1)))) {
        return (int)bits;
    }

    return (int)(bits | (~0u << len)); // extend sign
}

// get signed 38bit field
static double getbits_38(const unsigned char *buff, int pos) {
    return (double)getbits(buff, pos, 32) * D(64.0) + getbitu(buff, pos + 32, 6);
}

static const unsigned int tbl_CRC24Q[]={
    0x000000,0x864CFB,0x8AD50D,0x0C99F6,0x93E6E1,0x15AA1A,0x1933EC,0x9F7F17,
    0xA18139,0x27CDC2,0x2B5434,0xAD18CF,0x3267D8,0xB42B23,0xB8B2D5,0x3EFE2E,
    0xC54E89,0x430272,0x4F9B84,0xC9D77F,0x56A868,0xD0E493,0xDC7D65,0x5A319E,
    0x64CFB0,0xE2834B,0xEE1ABD,0x685646,0xF72951,0x7165AA,0x7DFC5C,0xFBB0A7,
    0x0CD1E9,0x8A9D12,0x8604E4,0x00481F,0x9F3708,0x197BF3,0x15E205,0x93AEFE,
    0xAD50D0,0x2B1C2B,0x2785DD,0xA1C926,0x3EB631,0xB8FACA,0xB4633C,0x322FC7,
    0xC99F60,0x4FD39B,0x434A6D,0xC50696,0x5A7981,0xDC357A,0xD0AC8C,0x56E077,
    0x681E59,0xEE52A2,0xE2CB54,0x6487AF,0xFBF8B8,0x7DB443,0x712DB5,0xF7614E,
    0x19A3D2,0x9FEF29,0x9376DF,0x153A24,0x8A4533,0x0C09C8,0x00903E,0x86DCC5,
    0xB822EB,0x3E6E10,0x32F7E6,0xB4BB1D,0x2BC40A,0xAD88F1,0xA11107,0x275DFC,
    0xDCED5B,0x5AA1A0,0x563856,0xD074AD,0x4F0BBA,0xC94741,0xC5DEB7,0x43924C,
    0x7D6C62,0xFB2099,0xF7B96F,0x71F594,0xEE8A83,0x68C678,0x645F8E,0xE21375,
    0x15723B,0x933EC0,0x9FA736,0x19EBCD,0x8694DA,0x00D821,0x0C41D7,0x8A0D2C,
    0xB4F302,0x32BFF9,0x3E260F,0xB86AF4,0x2715E3,0xA15918,0xADC0EE,0x2B8C15,
    0xD03CB2,0x567049,0x5AE9BF,0xDCA544,0x43DA53,0xC596A8,0xC90F5E,0x4F43A5,
    0x71BD8B,0xF7F170,0xFB6886,0x7D247D,0xE25B6A,0x641791,0x688E67,0xEEC29C,
    0x3347A4,0xB50B5F,0xB992A9,0x3FDE52,0xA0A145,0x26EDBE,0x2A7448,0xAC38B3,
    0x92C69D,0x148A66,0x181390,0x9E5F6B,0x01207C,0x876C87,0x8BF571,0x0DB98A,
    0xF6092D,0x7045D6,0x7CDC20,0xFA90DB,0x65EFCC,0xE3A337,0xEF3AC1,0x69763A,
    0x578814,0xD1C4EF,0xDD5D19,0x5B11E2,0xC46EF5,0x42220E,0x4EBBF8,0xC8F703,
    0x3F964D,0xB9DAB6,0xB54340,0x330FBB,0xAC70AC,0x2A3C57,0x26A5A1,0xA0E95A,
    0x9E1774,0x185B8F,0x14C279,0x928E82,0x0DF195,0x8BBD6E,0x872498,0x016863,
    0xFAD8C4,0x7C943F,0x700DC9,0xF64132,0x693E25,0xEF72DE,0xE3EB28,0x65A7D3,
    0x5B59FD,0xDD1506,0xD18CF0,0x57C00B,0xC8BF1C,0x4EF3E7,0x426A11,0xC426EA,
    0x2AE476,0xACA88D,0xA0317B,0x267D80,0xB90297,0x3F4E6C,0x33D79A,0xB59B61,
    0x8B654F,0x0D29B4,0x01B042,0x87FCB9,0x1883AE,0x9ECF55,0x9256A3,0x141A58,
    0xEFAAFF,0x69E604,0x657FF2,0xE33309,0x7C4C1E,0xFA00E5,0xF69913,0x70D5E8,
    0x4E2BC6,0xC8673D,0xC4FECB,0x42B230,0xDDCD27,0x5B81DC,0x57182A,0xD154D1,
    0x26359F,0xA07964,0xACE092,0x2AAC69,0xB5D37E,0x339F85,0x3F0673,0xB94A88,
    0x87B4A6,0x01F85D,0x0D61AB,0x8B2D50,0x145247,0x921EBC,0x9E874A,0x18CBB1,
    0xE37B16,0x6537ED,0x69AE1B,0xEFE2E0,0x709DF7,0xF6D10C,0xFA48FA,0x7C0401,
    0x42FA2F,0xC4B6D4,0xC82F22,0x4E63D9,0xD11CCE,0x575035,0x5BC9C3,0xDD8538
};

static unsigned int crc24q(const unsigned char *buff, int len) {
    unsigned int crc=0;
    int i;

    for (i=0;i < len;i++) {
        crc= ((crc << 8) & 0xFFFFFF) ^ tbl_CRC24Q[(crc >> 16) ^ buff[i]];
    }

    return crc;
}
