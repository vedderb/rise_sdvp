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

#include "adc_read.h"
#include <string.h>

// Settings
#define ADC_CHANNELS				9
#define NUM_SAMP					4

static const ADCConversionGroup adcgrpcfg = {
		FALSE,
		ADC_CHANNELS,
		NULL,
		NULL,
		0,                        /* CR1 */
		ADC_CR2_SWSTART,          /* CR2 */
		ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_144) | ADC_SMPR1_SMP_VREF(ADC_SAMPLE_144),
		ADC_SMPR2_SMP_AN0(ADC_SAMPLE_56) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_56) |
		ADC_SMPR2_SMP_AN2(ADC_SAMPLE_56) | ADC_SMPR2_SMP_AN3(ADC_SAMPLE_56) |
		ADC_SMPR2_SMP_AN4(ADC_SAMPLE_56) | ADC_SMPR2_SMP_AN5(ADC_SAMPLE_56) |
		ADC_SMPR2_SMP_AN6(ADC_SAMPLE_56) | ADC_SMPR2_SMP_AN7(ADC_SAMPLE_56) |
		ADC_SMPR2_SMP_AN8(ADC_SAMPLE_56),
		0,                        /* HTR */
		0,                        /* LTR */
		0,                        /* SQR1 */
		ADC_SQR2_SQ8_N(ADC_CHANNEL_SENSOR) |
		ADC_SQR2_SQ8_N(ADC_CHANNEL_VREFINT) | ADC_SQR2_SQ7_N(ADC_CHANNEL_IN6),
		ADC_SQR3_SQ6_N(ADC_CHANNEL_IN5)   | ADC_SQR3_SQ5_N(ADC_CHANNEL_IN4) |
		ADC_SQR3_SQ4_N(ADC_CHANNEL_IN3)   | ADC_SQR3_SQ3_N(ADC_CHANNEL_IN2) |
		ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1)   | ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0)
};

// Variables
static volatile float m_voltages[ADC_CHANNELS];
static volatile ADC_CNT_t m_adc_cnt[ADC_CHANNELS] = {0};

// Threads
static THD_WORKING_AREA(adc_read_thread_wa, 512);
static THD_FUNCTION(adc_read_thread, arg);

void adc_read_init(void) {
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);

	adcStart(&ADCD1, NULL);
	adcSTM32EnableTSVREFE();

	chThdCreateStatic(adc_read_thread_wa, sizeof(adc_read_thread_wa), NORMALPRIO, adc_read_thread, NULL);
}

float adc_read_get_voltage(int channel) {
	if (channel < 0 || channel >= ADC_CHANNELS) {
		return -1.0;
	} else {
		return m_voltages[channel];
	}
}

volatile ADC_CNT_t* adc_read_get_counter(int channel) {
	if (channel < 0 || channel >= ADC_CHANNELS) {
		return 0;
	} else {
		return &m_adc_cnt[channel];
	}
}

static THD_FUNCTION(adc_read_thread, arg) {
	(void)arg;
	chRegSetThreadName("ADC read");

	systime_t time_last = chVTGetSystemTimeX();

	for(;;) {
		adcsample_t samples[ADC_CHANNELS * NUM_SAMP];
		float voltages[ADC_CHANNELS];
		memset(voltages, 0, sizeof(voltages));

		adcConvert(&ADCD1, &adcgrpcfg, samples, NUM_SAMP);
		for (int i = 0;i < NUM_SAMP;i++) {
			for (int j = 0;j < ADC_CHANNELS;j++) {
				voltages[j] += (float)samples[ADC_CHANNELS * i + j];
			}
		}

		for (int i = 0;i < ADC_CHANNELS;i++) {
			m_voltages[i] = (voltages[i] / (float)NUM_SAMP / 4095.0) * 3.3;
		}

		float dt = (float)chVTTimeElapsedSinceX(time_last) / (float)CH_CFG_ST_FREQUENCY;
		time_last = chVTGetSystemTimeX();

		for (int i = 0;i < ADC_CHANNELS;i++) {
			bool was_high = m_adc_cnt[i].is_high;

			if (m_adc_cnt[i].is_high && m_voltages[i] < 0.3) {
				m_adc_cnt[i].is_high = false;
			} else if (!m_adc_cnt[i].is_high && m_voltages[i] >= 0.9) {
				m_adc_cnt[i].is_high = true;
			}

			if (was_high != m_adc_cnt[i].is_high) {
				if (was_high) {
					m_adc_cnt[i].high_time_last = m_adc_cnt[i].high_time_current + dt / 2.0;
					m_adc_cnt[i].high_time_current = dt / 2.0;
					m_adc_cnt[i].toggle_low_cnt++;
				} else {
					m_adc_cnt[i].low_time_last = m_adc_cnt[i].low_time_current + dt / 2.0;
					m_adc_cnt[i].low_time_current = dt / 2.0;
					m_adc_cnt[i].toggle_high_cnt++;
				}
			} else {
				if (was_high) {
					m_adc_cnt[i].high_time_current += dt;
				} else {
					m_adc_cnt[i].low_time_current += dt;
				}
			}
		}

		chThdSleepMicroseconds(500);
	}
}
