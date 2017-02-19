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

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "conf_general.h"
#include "comm_cc2520.h"
#include "comm_usb.h"
#include "commands.h"
#include "utils.h"
#include "mpu9150.h"
#include "basic_rf.h"
#include "ext_cb.h"
#include "adconv.h"
#include "led.h"
#include "packet.h"
#include "pos.h"
#include "comm_can.h"
#include "servo_simple.h"
#include "autopilot.h"
#include "timeout.h"
#include "log.h"
#include "radar.h"
#include "comm_cc1120.h"
#include "ublox.h"

/*
 * Timers used:
 * TIM6: Pos
 * TIM3: servo_simple
 *
 * DMA/Stream	Device		Usage
 * 2, 4			ADC1		adconv
 * 1, 0			I2C1		I2C Port
 * 1, 6			I2C1		I2C Port
 * 1, 2			I2C2		MPU9150
 * 1, 7			I2C2		MPU9150
 * 1, 3			SPI2		CC2520
 * 1, 4			SPI2		CC2520
 * 2, 0			SPI2		CC1120
 * 2, 3			SPI2		CC1120
 * 2, 2			UART6		UBLOX
 * 2, 7			UART6		UBLOX
 *
 */

int main(void) {
	halInit();
	chSysInit();

	led_init();
	ext_cb_init();

#if MAIN_MODE == MAIN_MODE_CAR
	conf_general_init();
	adconv_init();
	servo_simple_init();
	pos_init();
	comm_can_init();
	autopilot_init();
	timeout_init();
	log_init();
#if RADAR_EN
	radar_init();
	radar_setup_measurement_default();
#endif
#endif

	comm_usb_init();
	comm_cc2520_init();
	comm_cc1120_init();
	commands_init();

#if MAIN_MODE == MAIN_MODE_CAR
	commands_set_send_func(comm_cc2520_send_buffer);
#endif

#if UBLOX_EN
	ublox_init();
#endif

	timeout_configure(2000, 20.0);
	log_set_enabled(main_config.log_en);
	log_set_name(main_config.log_name);

	for(;;) {
		chThdSleepMilliseconds(2);
		packet_timerfunc();
	}
}
