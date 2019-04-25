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

#include "pwm_esc.h"
#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "stm32f4xx_conf.h"
#include "pwm_esc.h"

// Settings
#define ESC_UPDATE_RATE			200	// Hz
#define TIM_CLOCK				5e6 // Hz
#define ALL_CHANNELS			0xFF

// Pins
#define SERVO1_GPIO				GPIOB
#define SERVO1_PIN				0
#define SERVO2_GPIO				GPIOB
#define SERVO2_PIN				1
#define SERVO3_GPIO				GPIOE
#define SERVO3_PIN				5
#define SERVO4_GPIO				GPIOE
#define SERVO4_PIN				6

static PWMConfig pwmcfg3 = {
		TIM_CLOCK,
		(uint16_t)((uint32_t)TIM_CLOCK / (uint32_t)ESC_UPDATE_RATE),
		NULL,
		{
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_ACTIVE_HIGH, NULL},
				{PWM_OUTPUT_ACTIVE_HIGH, NULL}
		},
		0,
		0,
#if STM32_PWM_USE_ADVANCED
		0
#endif
};

static PWMConfig pwmcfg9 = {
		TIM_CLOCK,
		(uint16_t)((uint32_t)TIM_CLOCK / (uint32_t)ESC_UPDATE_RATE),
		NULL,
		{
				{PWM_OUTPUT_ACTIVE_HIGH, NULL},
				{PWM_OUTPUT_ACTIVE_HIGH, NULL},
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_DISABLED, NULL}
		},
		0,
		0,
#if STM32_PWM_USE_ADVANCED
		0
#endif
};

void pwm_esc_init(void) {
	pwmStart(&PWMD3, &pwmcfg3);
	pwmStart(&PWMD9, &pwmcfg9);

	palSetPadMode(SERVO1_GPIO, SERVO1_PIN,
			PAL_MODE_ALTERNATE(GPIO_AF_TIM3) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);
	palSetPadMode(SERVO2_GPIO, SERVO2_PIN,
			PAL_MODE_ALTERNATE(GPIO_AF_TIM3) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);
	palSetPadMode(SERVO3_GPIO, SERVO3_PIN,
			PAL_MODE_ALTERNATE(GPIO_AF_TIM9) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);
	palSetPadMode(SERVO4_GPIO, SERVO4_PIN,
			PAL_MODE_ALTERNATE(GPIO_AF_TIM9) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);

	pwm_esc_set_all(0);
}

void pwm_esc_set_all(float pulse_width) {
	pwm_esc_set(ALL_CHANNELS, pulse_width);
}

/**
 * Set output pulsewidth.
 *
 * @param channel
 * Channel to use
 * Range: [0 - 3]
 * 0xFF: All Channels
 *
 * @param pulse_width
 * Pulsewidth to use
 * Range: [0.0 - 1.0]
 *
 */
void pwm_esc_set(uint8_t channel, float pulse_width) {
	uint32_t cnt_val;

	if (0) {
		// Always set zero if emergency stop is set.
		// TODO: Implement this
		cnt_val = ((TIM_CLOCK / 1e3) * (uint32_t)main_config.mr.motor_pwm_min_us) / 1e3;
	} else {
		cnt_val = ((TIM_CLOCK / 1e3) * (uint32_t)main_config.mr.motor_pwm_min_us) / 1e3 +
				((TIM_CLOCK / 1e3) * (uint32_t)(pulse_width * (float)(main_config.mr.motor_pwm_max_us -
						main_config.mr.motor_pwm_min_us))) / 1e3;
	}

	switch(channel) {
	case 0:
		pwmEnableChannel(&PWMD3, 2, cnt_val);
		break;

	case 1:
		pwmEnableChannel(&PWMD3, 3, cnt_val);
		break;

	case 2:
		pwmEnableChannel(&PWMD9, 0, cnt_val);
		break;

	case 3:
		pwmEnableChannel(&PWMD9, 1, cnt_val);
		break;

	case ALL_CHANNELS:
		pwmEnableChannel(&PWMD3, 2, cnt_val);
		pwmEnableChannel(&PWMD3, 3, cnt_val);
		pwmEnableChannel(&PWMD9, 0, cnt_val);
		pwmEnableChannel(&PWMD9, 1, cnt_val);
		break;

	default:
		break;
	}
}
