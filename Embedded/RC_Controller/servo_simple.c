/*
	Copyright 2012 - 2019 Benjamin Vedder	benjamin@vedder.se

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

#include "servo_simple.h"
#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "stm32f4xx_conf.h"
#include "utils.h"
#include "servo_vesc.h"

#if SERVO_VESC_ID < 0
// Settings
#define TIM_CLOCK				1000000 // Hz
#define RAMP_LOOP_HZ			100 // Hz

// Private variables
static float m_pos_now;
static float m_pos_set;
static THD_WORKING_AREA(ramp_thread_wa, 128);

// Private functions
static THD_FUNCTION(ramp_thread, arg);
#endif

void servo_simple_init(void) {
#if SERVO_VESC_ID >= 0
	servo_vesc_init();
#else
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	m_pos_now = 0.5;
	m_pos_set = 0.5;

	palSetPadMode(GPIOB, 0, PAL_MODE_ALTERNATE(GPIO_AF_TIM3) |
			PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = (uint16_t)((uint32_t)TIM_CLOCK / (uint32_t)SERVO_OUT_RATE_HZ);
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)((168000000 / 2) / TIM_CLOCK) - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	servo_simple_set_pos(0.5);

	TIM_Cmd(TIM3, ENABLE);

	chThdCreateStatic(ramp_thread_wa, sizeof(ramp_thread_wa),
			NORMALPRIO, ramp_thread, NULL);
#endif
}

void servo_simple_set_pos(float pos) {
	utils_truncate_number(&pos, 0.0, 1.0);
#if SERVO_VESC_ID < 0
	m_pos_now = pos;
	m_pos_set = pos;

	float us = (float)SERVO_OUT_PULSE_MIN_US + pos * (float)(SERVO_OUT_PULSE_MAX_US - SERVO_OUT_PULSE_MIN_US);
	us *= (float)TIM_CLOCK / 1000000.0;
	TIM3->CCR3 = (uint32_t)us;
#else
	servo_vesc_set_pos(pos);
#endif
}

void servo_simple_set_pos_ramp(float pos) {
	utils_truncate_number(&pos, 0.0, 1.0);
#if SERVO_VESC_ID < 0
	m_pos_set = pos;
#else
	servo_vesc_set_pos(pos);
#endif
}

float servo_simple_get_pos_now(void) {
#if SERVO_VESC_ID < 0
	return m_pos_now;
#else
	return servo_vesc_get_pos();
#endif
}

float servo_simple_get_pos_set(void) {
#if SERVO_VESC_ID < 0
	return m_pos_set;
#else
	return servo_vesc_get_pos_set();
#endif
}

#if SERVO_VESC_ID < 0
static THD_FUNCTION(ramp_thread, arg) {
	(void)arg;

	chRegSetThreadName("Servo ramp");

	for(;;) {
		float pos_prev = m_pos_now;
		utils_step_towards(&m_pos_now, m_pos_set, 1.0 / ((float)RAMP_LOOP_HZ * main_config.car.steering_ramp_time));

		if (m_pos_now != pos_prev) {
			float us = (float)SERVO_OUT_PULSE_MIN_US + m_pos_now * (float)(SERVO_OUT_PULSE_MAX_US - SERVO_OUT_PULSE_MIN_US);
			us *= (float)TIM_CLOCK / 1000000.0;
			TIM3->CCR3 = (uint32_t)us;
		}

		chThdSleep(CH_CFG_ST_FREQUENCY / RAMP_LOOP_HZ);
	}
}
#endif
