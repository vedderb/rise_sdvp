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

#include "pos_uwb.h"
#include "comm_can.h"
#include "terminal.h"
#include "commands.h"

// Defines
#define UWB_MAX_ANCHORS			20

// Private variables
static UWB_ANCHOR m_anchors[UWB_MAX_ANCHORS];
static int m_anchor_last;

// Threads
static THD_WORKING_AREA(uwb_thread_wa, 512);
static THD_FUNCTION(uwb_thread, arg);

// Private functions
static void dw_range(uint8_t id, uint8_t dest, float range);
static void cmd_terminal_list_anchors(int argc, const char **argv);

void pos_uwb_init(void) {
	m_anchor_last = 0;

	chThdCreateStatic(uwb_thread_wa, sizeof(uwb_thread_wa),
			NORMALPRIO, uwb_thread, NULL);

	comm_can_set_range_func(dw_range);

	terminal_register_command_callback(
			"pos_uwb_anchors",
			"List UWB anchors.",
			0,
			cmd_terminal_list_anchors);
}

void pos_uwb_update_dr(float imu_yaw, float travel_dist, float steering_angle) {
	(void)imu_yaw;
	(void)travel_dist;
	(void)steering_angle;
}

void pos_uwb_add_anchor(UWB_ANCHOR a) {
	if (m_anchor_last < UWB_MAX_ANCHORS) {
		m_anchors[m_anchor_last++] = a;
	}
}

void pos_uwb_clear_anchors(void) {
	m_anchor_last = 0;
}

static void dw_range(uint8_t id, uint8_t dest, float range) {
	(void)id;
	(void)dest;
	(void)range;
}

static THD_FUNCTION(uwb_thread, arg) {
	(void)arg;

	chRegSetThreadName("UWB");

	for (;;) {
		chThdSleepMilliseconds(100);
	}

}

static void cmd_terminal_list_anchors(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("UWB anchor list:");

	for (int i = 0;i < m_anchor_last;i++) {
		commands_printf(
				"ID       : %d\n"
				"PX       : %.2f\n"
				"PY       : %.2f\n"
				"Height   : %.2f\n"
				"Last dist: %.2f\n\n",
				m_anchors[i].id,
				(double)m_anchors[i].px,
				(double)m_anchors[i].py,
				(double)m_anchors[i].height,
				(double)m_anchors[i].dist_last);
	}
}
