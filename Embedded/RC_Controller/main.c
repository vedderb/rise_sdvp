/*
	Copyright 2017 Benjamin Vedder	benjamin@vedder.se

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
#include "comm_cc1120.h"
#include "ublox.h"
#include "srf10.h"
#include "pwm_esc.h"
#include "mr_control.h"
#include "motor_sim.h"
#include "m8t_base.h"
#include "pos_uwb.h"
#include "fi.h"
#include "hydraulic.h"
#include "timer.h"

/*
 * Timers used:
 * TIM6: Pos
 * TIM3: servo_simple and pwm_esc
 * TIM9: pwm_esc
 * TIM5: timer.c
 *
 * DMA/Stream	Device		Usage
 * 2, 4			ADC1		adconv
 * 1, 0			I2C1		I2C EXT (Overlap with SPI EXT)
 * 1, 6			I2C1		I2C EXT
 * 1, 2			I2C2		MPU9150
 * 1, 7			I2C2		MPU9150
 * 1, 3			SPI2		CC2520
 * 1, 4			SPI2		CC2520
 * 2, 0			SPI1		CC1120
 * 2, 3			SPI1		CC1120
 * 2, 1			UART6		UBLOX
 * 2, 6			UART6		UBLOX
 * 2, 5			UART1		UART EXT
 * 2, 7			UART1		UART EXT
 * 1, 0			SPI3		SPI EXT (Overlap with I2C EXT)
 * 1, 5			SPI3		SPI EXT
 *
 */

int main(void) {
	halInit();
	chSysInit();

	timer_init();
	led_init();
	ext_cb_init();

#if MAIN_MODE == MAIN_MODE_CAR
	conf_general_init();
	adconv_init();
	servo_simple_init();
	pos_init();
	pos_uwb_init();
	comm_can_init();
	autopilot_init();
	timeout_init();
	log_init();
	motor_sim_init();
#if HAS_HYDRAULIC_DRIVE
	hydraulic_init();
#endif
#endif

#if MAIN_MODE == MAIN_MODE_MULTIROTOR
	conf_general_init();
	adconv_init();
	srf10_init();
	pwm_esc_init();
	pos_init();
	pos_uwb_init();
	mr_control_init();
#endif

	comm_usb_init();

#if HAS_CC2520
	comm_cc2520_init();
#endif
#if HAS_CC1120
	comm_cc1120_init();
#endif

	commands_init();

#if MAIN_MODE_IS_VEHICLE && HAS_CC2520
	commands_set_send_func(comm_cc2520_send_buffer);
#endif

#if UBLOX_EN
	ublox_init();
#endif

#if MAIN_MODE_IS_BASE || MAIN_MODE_IS_MOTE
	m8t_base_init();
#endif

#if MAIN_MODE_IS_BASE
	m8t_base_start();
#endif

#if MAIN_MODE == MAIN_MODE_CAR
	motor_sim_set_running(main_config.car.simulate_motor);
#endif

#if MAIN_MODE_IS_VEHICLE
	rtcm3_set_rx_callback_obs(pos_base_rtcm_obs, commands_get_rtcm3_state());
#endif

	fi_init();

	timeout_configure(2000, 40.0);
	log_set_rate(main_config.log_rate_hz);
	log_set_enabled(main_config.log_en);
	log_set_name(main_config.log_name);
	log_set_ext(main_config.log_mode_ext, main_config.log_uart_baud);

	for(;;) {
		chThdSleepMilliseconds(10);
		packet_timerfunc();
	}
}
