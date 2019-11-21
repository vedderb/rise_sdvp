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

#include <string.h>
#include <math.h>
#include <stdio.h>

#include "pos_uwb.h"
#include "comm_can.h"
#include "terminal.h"
#include "commands.h"
#include "utils.h"
#include "fi.h"
#include "pos.h"

// Defines
#define MAX_ANCHORS				20
#define MAX_POINT_DISTANCE		80
#define AVG_SAMPLES				5
#define CORR_STEP				0.2
#define OFFSET_X				0.085
#define OFFSET_Y				0.0
#define YAW_CLAMP_THRESHOLD		0.5
#define YAW_HISTORY_SIZE		50

// Private datatypes
typedef struct {
	float px_prev;
	float py_prev;
	float px;
	float py;
	float yaw;
	float speed;
	float traveldist_prev;
	float gyro_offset;
	float angle_rad;
	float history_for_yaw[YAW_HISTORY_SIZE];
	int history_for_yaw_ptr;
	float yaw_drift;
	int is_clamped;
	float clamp_last_value;
	float px_old;
	float py_old;
	systime_t last_update_time;
	float uwb_sample_now;
} uwb_state_t;

// Private variables
static UWB_ANCHOR m_anchors[MAX_ANCHORS];
static int m_anchor_last;
static mutex_t m_mutex_pos;
static uwb_state_t m_uwb_state;

// Threads
static THD_WORKING_AREA(uwb_thread_wa, 512);
static THD_FUNCTION(uwb_thread, arg);

// Private functions
static void dw_range(uint8_t id, uint8_t dest, float range);
static void cmd_terminal_list_anchors(int argc, const char **argv);
static void cmd_terminal_reset_pos(int argc, const char **argv);

void pos_uwb_init(void) {
	m_anchor_last = 0;
	memset(&m_anchors, 0, sizeof(m_anchors));
	memset(&m_uwb_state, 0, sizeof(m_uwb_state));
	chMtxObjectInit(&m_mutex_pos);

	chThdCreateStatic(uwb_thread_wa, sizeof(uwb_thread_wa),
			NORMALPRIO, uwb_thread, NULL);

	terminal_register_command_callback(
			"pos_uwb_anchors",
			"List UWB anchors.",
			0,
			cmd_terminal_list_anchors);

	terminal_register_command_callback(
			"pos_uwb_reset_pos",
			"Reset position to the sensor fusion position.",
			0,
			cmd_terminal_reset_pos);
}

void pos_uwb_update_dr(float imu_yaw, float rtk_yaw, float travel_dist,
		float turn_rad, float speed) {
	(void)turn_rad;
	(void)rtk_yaw;

	fi_inject_fault_float("uwb_travel_dist", &travel_dist);
	fi_inject_fault_float("uwb_yaw", &imu_yaw);

	chMtxLock(&m_mutex_pos);

	float dt = (float)chVTTimeElapsedSinceX(m_uwb_state.last_update_time) / (float)CH_CFG_ST_FREQUENCY;
	m_uwb_state.last_update_time = chVTGetSystemTimeX();

	m_uwb_state.yaw = -imu_yaw;
	m_uwb_state.is_clamped = fabsf(speed * 3.6) < YAW_CLAMP_THRESHOLD;
	
	if (!main_config.car.clamp_imu_yaw_stationary) {
		m_uwb_state.is_clamped = false;
	}

	if (m_uwb_state.is_clamped) {
		m_uwb_state.gyro_offset = m_uwb_state.clamp_last_value - m_uwb_state.yaw;
	} else {
		m_uwb_state.clamp_last_value = m_uwb_state.yaw + m_uwb_state.gyro_offset;
	}

	m_uwb_state.px += cosf((m_uwb_state.yaw + m_uwb_state.gyro_offset)
			* M_PI / 180) * (travel_dist );
	m_uwb_state.py += sinf((m_uwb_state.yaw + m_uwb_state.gyro_offset)
			* M_PI / 180) * (travel_dist);

	if (!m_uwb_state.is_clamped) {
		m_uwb_state.gyro_offset += m_uwb_state.yaw_drift * dt;
		m_uwb_state.history_for_yaw[m_uwb_state.history_for_yaw_ptr++] = m_uwb_state.yaw;

		if (m_uwb_state.history_for_yaw_ptr >= YAW_HISTORY_SIZE) {
			m_uwb_state.history_for_yaw_ptr = 0;
		}
	}

	utils_norm_angle(&m_uwb_state.gyro_offset);

	chMtxUnlock(&m_mutex_pos);
}

void pos_uwb_add_anchor(UWB_ANCHOR a) {
	// If an anchor with the same ID already exists update it.
	for (int i = 0;i < m_anchor_last;i++) {
		if (m_anchors[i].id == a.id) {
			m_anchors[i] = a;
			return;
		}
	}

	if (m_anchor_last < MAX_ANCHORS) {
		m_anchors[m_anchor_last++] = a;
	}
}

void pos_uwb_clear_anchors(void) {
	m_anchor_last = 0;
}

void pos_uwb_get_pos(POS_STATE *p) {
	chMtxLock(&m_mutex_pos);
	p->px = m_uwb_state.px;
	p->py = m_uwb_state.py;
	p->speed = m_uwb_state.speed;
	p->yaw = -(m_uwb_state.yaw + m_uwb_state.gyro_offset);
	chMtxUnlock(&m_mutex_pos);
}

void pos_uwb_set_xya(float x, float y, float angle) {
	chMtxLock(&m_mutex_pos);

	m_uwb_state.px = x;
	m_uwb_state.py = y;
	m_uwb_state.gyro_offset = -angle - m_uwb_state.yaw;
	m_uwb_state.clamp_last_value = -angle;

	chMtxUnlock(&m_mutex_pos);
}

static void dw_range(uint8_t id, uint8_t dest, float range) {
	(void)id;

	// Fault injection on the measured range
	// fi_is_active is not necessary, but it minimizes runtime overhead
	// when fault injection is disabled (sprintf).
	if (fi_is_active()) {
		char id_str[20];
		sprintf(id_str, "uwb_range_%d", dest);
		fi_inject_fault_float(id_str, &range);
	}

	float dt = 1.0;

	UWB_ANCHOR *a = 0;
	for (int i = 0;i < m_anchor_last;i++) {
		if (m_anchors[i].id == dest) {
			a = &m_anchors[i];
			a->dist_last = range;

			dt = (float)(chVTGetSystemTimeX() - a->timestamp) /
					(float)CH_CFG_ST_FREQUENCY;

			a->timestamp = chVTGetSystemTimeX();
			break;
		}
	}

	// Unused
	(void)dt;

	if (a) {
		chMtxLock(&m_mutex_pos);

		// Apply antenna offset
		const float s_yaw = sinf(-m_uwb_state.yaw * M_PI / 180.0);
		const float c_yaw = cosf(-m_uwb_state.yaw * M_PI / 180.0);
		const float px = m_uwb_state.px + (c_yaw * OFFSET_X - s_yaw * OFFSET_Y);
		const float py = m_uwb_state.py + (s_yaw * OFFSET_X + c_yaw * OFFSET_Y);

		float c = 0.0;
		float d_estimated=sqrtf((px - a->px) * (px - a->px) +
				(py - a->py) * (py - a->py));
		float vuwb_x = (a->px - px) / d_estimated;
		float vuwb_y = (a->py - py) / d_estimated;
		float diff = d_estimated - a->dist_last;

		if (fabsf(diff) < main_config.uwb_max_corr){
			c = diff;
		} else{
			c = 0.2 * diff / fabsf(diff);
		}

		m_uwb_state.px += vuwb_x * c;
		m_uwb_state.py += vuwb_y * c;

		float corrDiff = sqrtf((m_uwb_state.py - m_uwb_state.py_old) *
				(m_uwb_state.py - m_uwb_state.py_old) +
				(m_uwb_state.px - m_uwb_state.px_old) * (m_uwb_state.px -
						m_uwb_state.px_old));


		if (m_uwb_state.uwb_sample_now < 2) {
			corrDiff = 0.0;
			m_uwb_state.px_old = m_uwb_state.px;
			m_uwb_state.py_old = m_uwb_state.py;
		}

		if (corrDiff > 0.1 && m_uwb_state.history_for_yaw_ptr > 0 && !m_uwb_state.is_clamped) {
			float yaw_uwb = atan2f((m_uwb_state.py - m_uwb_state.py_old),
					(m_uwb_state.px - m_uwb_state.px_old)) * 180.0 / M_PI;

			if (m_uwb_state.speed < 0.0) {
				yaw_uwb += 180.0;
				while (yaw_uwb > 180.0) yaw_uwb -= 2.0 * 180.0;
			}

			float yaw_history_avg = 0.0;

			for (int i = 0;i < m_uwb_state.history_for_yaw_ptr;i++) {
				yaw_history_avg += m_uwb_state.history_for_yaw[i];
			}

			yaw_history_avg /= (float)m_uwb_state.history_for_yaw_ptr;

			float gyroOffsetEst = yaw_uwb - yaw_history_avg;
			utils_norm_angle(&gyroOffsetEst);

			// Difference between currrent and last correction
			float diffOffset = gyroOffsetEst - m_uwb_state.gyro_offset;
			utils_norm_angle(&diffOffset);

			// Tuning parameter, proportional term
			float step = 0.25;
			if (fabsf(diffOffset) < step) {
				m_uwb_state.gyro_offset = gyroOffsetEst;
			} else {
				if (gyroOffsetEst < m_uwb_state.gyro_offset) {
					m_uwb_state.gyro_offset -= step;
				} else {
					m_uwb_state.gyro_offset += step;
				}
			}

			// Tuning parameter, integral term
			float driftGain = 0.5;
			// Maximum contribution from integral term each step
			float driftMax = 3.5;

			m_uwb_state.yaw_drift += driftGain * diffOffset;
			utils_truncate_number_abs(&m_uwb_state.yaw_drift, driftMax);

			m_uwb_state.px_old = m_uwb_state.px;
			m_uwb_state.py_old = m_uwb_state.py;
			m_uwb_state.history_for_yaw_ptr = 0;
		}

		m_uwb_state.uwb_sample_now++;

		chMtxUnlock(&m_mutex_pos);
	}
}

static THD_FUNCTION(uwb_thread, arg) {
	(void)arg;

	chRegSetThreadName("UWB");

	int anchor_index = 0;

	for (;;) {
		int next = (anchor_index + 1);
		if (next >= m_anchor_last) {
			next = 0;
		}

		chMtxLock(&m_mutex_pos);
		while (next != anchor_index) {
			if (utils_point_distance(m_anchors[next].px, m_anchors[next].py,
					m_uwb_state.px, m_uwb_state.py) < MAX_POINT_DISTANCE) {
				anchor_index = next;
				comm_can_set_range_func(dw_range);
				comm_can_dw_range(255, m_anchors[next].id, AVG_SAMPLES);
				break;
			}

			next++;
			if (next >= m_anchor_last) {
				next = 0;
			}
		}
		chMtxUnlock(&m_mutex_pos);

		chThdSleepMilliseconds(100);
	}
}

static void cmd_terminal_list_anchors(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("UWB anchor list:");

	for (int i = 0;i < m_anchor_last;i++) {
		float age = (float)(chVTGetSystemTimeX() - m_anchors[i].timestamp) /
				(float)CH_CFG_ST_FREQUENCY;

		commands_printf(
				"ID       : %d\n"
				"PX       : %.2f\n"
				"PY       : %.2f\n"
				"Height   : %.2f m\n"
				"Last dist: %.2f m\n"
				"Age      : %.2f s\n\n",
				m_anchors[i].id,
				(double)m_anchors[i].px,
				(double)m_anchors[i].py,
				(double)m_anchors[i].height,
				(double)m_anchors[i].dist_last,
				(double)age);
	}
}

static void cmd_terminal_reset_pos(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	POS_STATE p;
	pos_get_pos(&p);

	chMtxLock(&m_mutex_pos);
	m_uwb_state.yaw = p.yaw;
	m_uwb_state.px = p.px;
	m_uwb_state.py = p.py;
	chMtxUnlock(&m_mutex_pos);

	commands_printf("UWB Position Reset\n");
}
