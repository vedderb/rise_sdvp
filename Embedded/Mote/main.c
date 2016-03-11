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

#include "comm_cc2520.h"
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "comm_usb_serial.h"
#include "stm32f4xx_conf.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "utils.h"
#include "basic_rf.h"
#include "ext_cb.h"
#include "led.h"
#include "comm_usb.h"
#include "packet.h"

int main(void) {
	halInit();
	chSysInit();

	led_init();
	ext_cb_init();
	comm_usb_init();
	comm_cc2520_init();

	for(;;) {
		chThdSleepMilliseconds(2);
		packet_timerfunc();
	}
}
