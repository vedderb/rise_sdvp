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

#include "pos_uwb.h"
#include "comm_can.h"
#include "terminal.h"
#include "commands.h"
#include "utils.h"

// Defines
#define MAX_ANCHORS				20
#define MAX_POINT_DISTANCE		80
#define AVG_SAMPLES				5
#define CORR_STEP				0.2
#define OFFSET_X				0.085
#define OFFSET_Y				0.0

// Private variables
static UWB_ANCHOR m_anchors[MAX_ANCHORS];
static int m_anchor_last;
static POS_STATE m_pos;
static mutex_t m_mutex_pos;

// Threads
static THD_WORKING_AREA(uwb_thread_wa, 512);
static THD_FUNCTION(uwb_thread, arg);

// Private functions
static void dw_range(uint8_t id, uint8_t dest, float range);
static void cmd_terminal_list_anchors(int argc, const char **argv);

void pos_uwb_init(void) {
	m_anchor_last = 0;
	memset(&m_pos, 0, sizeof(m_pos));
	memset(&m_anchors, 0, sizeof(m_anchors));
	chMtxObjectInit(&m_mutex_pos);

	chThdCreateStatic(uwb_thread_wa, sizeof(uwb_thread_wa),
			NORMALPRIO, uwb_thread, NULL);

	terminal_register_command_callback(
			"pos_uwb_anchors",
			"List UWB anchors.",
			0,
			cmd_terminal_list_anchors);
}

void pos_uwb_update_dr(float imu_yaw, float travel_dist,
		float steering_angle, float speed) {
	(void)steering_angle;

	// TODO: Implement yaw correction

	m_pos.yaw = imu_yaw;
	m_pos.speed = speed;
	float angle_rad = -m_pos.yaw * M_PI / 180.0;

	m_pos.px += cosf(angle_rad) * travel_dist;
	m_pos.py += sinf(angle_rad) * travel_dist;
}

void pos_uwb_add_anchor(UWB_ANCHOR a) {
	if (m_anchor_last < MAX_ANCHORS) {
		m_anchors[m_anchor_last++] = a;
	}
}

void pos_uwb_clear_anchors(void) {
	m_anchor_last = 0;
}

void pos_uwb_get_pos(POS_STATE *p) {
	chMtxLock(&m_mutex_pos);
	*p = m_pos;
	chMtxUnlock(&m_mutex_pos);
}

void pos_uwb_set_xya(float x, float y, float angle) {
	chMtxLock(&m_mutex_pos);

	m_pos.px = x;
	m_pos.py = y;
	m_pos.yaw = angle;

	chMtxUnlock(&m_mutex_pos);
}

static void dw_range(uint8_t id, uint8_t dest, float range) {
	(void)id;

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
		const float s_yaw = sinf(-m_pos.yaw * M_PI / 180.0);
		const float c_yaw = cosf(-m_pos.yaw * M_PI / 180.0);
		const float px = m_pos.px + (c_yaw * OFFSET_X - s_yaw * OFFSET_Y);
		const float py = m_pos.py + (s_yaw * OFFSET_X + c_yaw * OFFSET_Y);

		const float dx = a->px - px;
		const float dy = a->py - py;
		float d = sqrtf(SQ(dx) + SQ(dy));
		d = d < 0.01 ? 0.01 : d; // Avoid divide by 0

		const float diff = d - a->dist_last;
		if (diff < 8.0) {
			const float ca = dx / d;
			const float sa = dy / d;
			const float corr_step = SIGN(diff) * MIN(fabsf(diff), CORR_STEP);

			m_pos.px += corr_step * ca;
			m_pos.py += corr_step * sa;
		}

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
					m_pos.px, m_pos.py) < MAX_POINT_DISTANCE) {
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
