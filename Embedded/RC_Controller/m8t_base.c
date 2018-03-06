/*
	Copyright 2018 Benjamin Vedder	benjamin@vedder.se

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

#include "m8t_base.h"
#include "ublox.h"
#include "rtcm3_simple.h"
#include "utils.h"
#include "comm_cc2520.h"
#include "comm_cc1120.h"
#include "terminal.h"
#include "commands.h"

#include <math.h>

#if MAIN_MODE_IS_BASE || MAIN_MODE_IS_MOTE

// Private variables
static double m_x_avg = D(0.0);
static double m_y_avg = D(0.0);
static double m_z_avg = D(0.0);
static double m_avg_samples = D(0.0);
static float m_min_acc;
static int m_min_samples;
static ubx_nav_sol m_last_sol;

// Private functions
static void ubx_rx_nav_sol(ubx_nav_sol *sol);
static void ubx_rx_rawx(ubx_rxm_rawx *rawx);
static void terminal_cmd_status(int argc, const char **argv);
static void terminal_cmd_reset(int argc, const char **argv);

void m8t_base_init(void) {
	m_x_avg = D(0.0);
	m_y_avg = D(0.0);
	m_z_avg = D(0.0);
	m_avg_samples = D(0.0);

	m_min_acc = 10.0;
	m_min_samples = 60;

	// Register terminal callbacks
	terminal_register_command_callback(
			"base_status",
			"Print the base station status",
			0,
			terminal_cmd_status);

	terminal_register_command_callback(
			"base_reset",
			"Restart the base station survey-in",
			0,
			terminal_cmd_reset);
}

/**
 * Start base station. This will start a survey-in if not enough samples are collected.
 */
void m8t_base_start(void) {
	ublox_set_rx_callback_nav_sol(ubx_rx_nav_sol);
	ublox_set_rx_callback_rawx(ubx_rx_rawx);

	// Set rate to 1 Hz and time reference to UTC
	ublox_cfg_rate(1000, 1, 0);

	// Stationary dynamic model
	ubx_cfg_nav5 nav5;
	memset(&nav5, 0, sizeof(ubx_cfg_nav5));
	nav5.apply_dyn = true;
	nav5.dyn_model = 2;
	ublox_cfg_nav5(&nav5);

	// Switch on SOL and RAWX messages
	ublox_cfg_msg(UBX_CLASS_NAV, UBX_NAV_SOL, 1);
	ublox_cfg_msg(UBX_CLASS_RXM, UBX_RXM_RAWX, 1);
}

/**
 * Stop M8T base station mode.
 */
void m8t_base_stop(void) {
	ublox_set_rx_callback_nav_sol(0);
	ublox_set_rx_callback_rawx(0);

	// Set rate to 5 Hz and time reference to UTC
	ublox_cfg_rate(200, 1, 0);

	// Automotive dynamic model
	ubx_cfg_nav5 nav5;
	memset(&nav5, 0, sizeof(ubx_cfg_nav5));
	nav5.apply_dyn = true;
	nav5.dyn_model = 4;
	ublox_cfg_nav5(&nav5);

	// Switch on SOL and RAWX messages
	ublox_cfg_msg(UBX_CLASS_NAV, UBX_NAV_SOL, 0);
	ublox_cfg_msg(UBX_CLASS_RXM, UBX_RXM_RAWX, 0);

	m8t_base_reset_pos();
}

/**
 * Set base station position manually, instead of doing survey-in.
 *
 * @param lat
 * Base station latitude.
 *
 * @param lon
 * Base station longitude.
 *
 * @param height
 * Base station height.
 *
 */
void m8t_base_set_pos(double lat, double lon, double height) {
	double x, y, z;
	utils_llh_to_xyz(lat, lon, height, &x, &y, &z);

	m_avg_samples = (double)m_min_samples + D(1.0);
	m_x_avg = x * m_avg_samples;
	m_y_avg = y * m_avg_samples;
	m_z_avg = z * m_avg_samples;
}

/**
 * Reset the base station position and start doing a survey in.
 */
void m8t_base_reset_pos(void) {
	m_x_avg = D(0.0);
	m_y_avg = D(0.0);
	m_z_avg = D(0.0);
	m_avg_samples = D(0.0);
}

/**
 * Set the minimum accuracy and number of samples for survey-in.
 *
 * @param acc
 * The minimum accuracy of each sample.
 *
 * @param samples
 * The minimum amount of samples
 */
void m8t_base_set_min_acc_samples(float acc, int samples) {
	m_min_acc = acc;
	m_min_samples = samples;
}

// Private functions
static void ubx_rx_nav_sol(ubx_nav_sol *sol) {
	m_last_sol = *sol;

	if (sol->p_acc < m_min_acc && m_avg_samples <= (double)m_min_samples) {
		m_x_avg += sol->ecef_x;
		m_y_avg += sol->ecef_y;
		m_z_avg += sol->ecef_z;
		m_avg_samples += D(1.0);
	}
}

static void ubx_rx_rawx(ubx_rxm_rawx *rawx) {
	if (m_avg_samples < (double)m_min_samples) {
		return;
	}

	static uint8_t data_gps[512];
	static uint8_t data_glo[512];
	static uint8_t data_ref[256];

	int gps_len = 0;
	int glo_len = 0;
	int ref_len = 0;

	rtcm_obs_header_t header;
	rtcm_obs_t obs[rawx->num_meas];

	header.staid = 0;
	header.t_wn = rawx->week;
	header.t_tow = rawx->rcv_tow;
	header.t_tod = fmod(rawx->rcv_tow - (double)rawx->leaps + D(10800.0), D(86400.0));

	bool has_gps = false;
	bool has_glo = false;
	bool has_ref = false;

	for (int i = 0;i < rawx->num_meas;i++) {
		ubx_rxm_rawx_obs *raw_obs = &rawx->obs[i];

		if (raw_obs->gnss_id == 0) {
			has_gps = true;
		} else if (raw_obs->gnss_id == 6) {
			has_glo = true;
		}
	}

	// GPS
	if (has_gps) {
		int obs_ind = 0;
		for (int i = 0;i < rawx->num_meas;i++) {
			ubx_rxm_rawx_obs *raw_obs = &rawx->obs[i];

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
		for (int i = 0;i < rawx->num_meas;i++) {
			ubx_rxm_rawx_obs *raw_obs = &rawx->obs[i];

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
	double lat, lon, height;
	utils_xyz_to_llh(
			m_x_avg / m_avg_samples,
			m_y_avg / m_avg_samples,
			m_z_avg / m_avg_samples,
			&lat, &lon, &height);

	rtcm_ref_sta_pos_t pos;
	pos.ant_height = 0.0;
	pos.height = height;
	pos.lat = lat;
	pos.lon = lon;
	pos.staid = 0;
	rtcm3_encode_1006(pos, data_ref, &ref_len);
	has_ref = true;

	// Send over radios
	void (*send_func)(uint8_t *data, unsigned int len) =
			MAIN_MODE == MAIN_MODE_M8T_BASE_400 ||
			MAIN_MODE == MAIN_MODE_MOTE_400 ||
			MAIN_MODE == MAIN_MODE_MOTE_HYBRID ?
					comm_cc1120_send_buffer : comm_cc2520_send_buffer;

	if (has_gps) {
		send_func(data_gps, gps_len);
	}

	if (has_glo) {
		send_func(data_glo, glo_len);
	}

	// Send base station position every 5 cycles to save bandwidth.
	static int base_pos_cnt = 0;
	base_pos_cnt++;
	if (base_pos_cnt >= 5) {
		base_pos_cnt = 0;

		if (has_ref) {
			send_func(data_ref, ref_len);
		}
	}
}

static void terminal_cmd_status(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	double lat = 0.0;
	double lon = 0.0;
	double height = 0.0;

	if (m_avg_samples > D(0.0)) {
		utils_xyz_to_llh(
				m_x_avg / m_avg_samples,
				m_y_avg / m_avg_samples,
				m_z_avg / m_avg_samples,
				&lat, &lon, &height);
	}

	commands_printf(
			"SVIN Done    : %s\n"
			"Samples      : %.0f / %d\n"
			"Accuracy now : %.2f m (min %.2f m)\n"
			"Latitude     : %.8f\n"
			"Longitude    : %.8f\n"
			"Height       : %.2f\n",
			m_avg_samples < (double)m_min_samples ? "False" : "True",
			m_avg_samples, m_min_samples,
			(double)m_last_sol.p_acc, (double)m_min_acc,
			lat, lon, height);
}

static void terminal_cmd_reset(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	m_x_avg = D(0.0);
	m_y_avg = D(0.0);
	m_z_avg = D(0.0);
	m_avg_samples = D(0.0);

	commands_printf(
			"OK\n"
			"New survey-in started. Correction will be sent again once this"
			"is done.\n");
}

#endif

