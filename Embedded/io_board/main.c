/*
	Copyright 2020 Benjamin Vedder	benjamin@vedder.se

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

#include "ch.h"
#include "hal.h"
#include "comm_can.h"
#include "adc_read.h"
#include "as5047.h"

int main(void) {
	halInit();
	chSysInit();

	palSetLineMode(LINE_LED_RED, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_LED_GREEN, PAL_MODE_OUTPUT_PUSHPULL);

	LED_OFF(LINE_LED_RED);
	LED_OFF(LINE_LED_GREEN);

	palSetLineMode(LINE_VALVE_1, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_VALVE_2, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_VALVE_3, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_VALVE_4, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_VALVE_5, PAL_MODE_OUTPUT_PUSHPULL);
	palSetLineMode(LINE_VALVE_6, PAL_MODE_OUTPUT_PUSHPULL);

	VALVE_OFF(LINE_VALVE_1);
	VALVE_OFF(LINE_VALVE_2);
	VALVE_OFF(LINE_VALVE_3);
	VALVE_OFF(LINE_VALVE_4);
	VALVE_OFF(LINE_VALVE_5);
	VALVE_OFF(LINE_VALVE_6);

	palSetLineMode(LINE_LIM_SW_1, PAL_MODE_INPUT_PULLUP);
	palSetLineMode(LINE_LIM_SW_2, PAL_MODE_INPUT_PULLUP);
	palSetLineMode(LINE_LIM_SW_3, PAL_MODE_INPUT_PULLUP);
	palSetLineMode(LINE_LIM_SW_4, PAL_MODE_INPUT_PULLUP);

	comm_can_init();
	adc_read_init();
	as5047_init();

	for (;;) {
		LED_TOGGLE(LINE_LED_GREEN);
		chThdSleepMilliseconds(1000);
	}

	return 0;
}
