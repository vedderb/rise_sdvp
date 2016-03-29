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

#include "autopilot.h"
#include <math.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "servo_simple.h"

// Defines
#define AP_HZ				100 // Hz

// Private variables
static THD_WORKING_AREA(ap_thread_wa, 512);
static ROUTE_POINT m_route[AP_ROUTE_SIZE];
static bool m_is_active;
static int m_point_last;
static int m_point_now;

// Private functions
static THD_FUNCTION(ap_thread, arg);

void autopilot_init(void) {
	memset(m_route, 0, sizeof(m_route));
	m_is_active = false;
	m_point_now = 0;
	m_point_last = 0;

	chThdCreateStatic(ap_thread_wa, sizeof(ap_thread_wa),
			NORMALPRIO, ap_thread, NULL);
}

void autopilot_add_point(ROUTE_POINT *p) {
	m_route[m_point_last++] = *p;

	if (m_point_last >= AP_ROUTE_SIZE) {
		m_point_last = 0;
	}
}

void autopilot_clear_route(void) {
	m_is_active = false;
	m_point_now = 0;
	m_point_last = 0;
}

void autopilot_set_active(bool active) {
	m_is_active = active;
}

bool autopilot_is_active(void) {
	return m_is_active;
}

static THD_FUNCTION(ap_thread, arg) {
	(void)arg;

	chRegSetThreadName("Autopilot");

	for(;;) {
		// TODO: Implement autopilot!

		chThdSleep(CH_CFG_ST_FREQUENCY / AP_HZ);
	}
}
