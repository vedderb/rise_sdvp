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

#include "srf10.h"
#include "stm32f4xx_conf.h"

#include <math.h>

// Settings
#define SRF10_I2C_TIMEOUT		1
#define ITERATION_TIME_US		100000
#define MAX_WRONG_SAMPLES		3
#define SAMPLE_MAX_DIFF			0.5

// HW Settings
#define SCL_GPIO				GPIOB
#define SCL_PAD					8
#define SDA_GPIO				GPIOB
#define SDA_PAD					9
#define I2C_DEV					I2CD1

// Addresses
#define SRF10_DEFAULT_ADDR		0x70
#define SRF10_MEAS_REG			0x0
#define SRF10_GAIN_REG			0x1
#define SRF10_RANGE_REG			0x2

// Private functions
static void delay_short(void);

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		100000,
		STD_DUTY_CYCLE
};

// Threads
static THD_WORKING_AREA(sampling_thread_wa, 2048);
static THD_FUNCTION(sampling_thread, arg);

// Function pointers
static void(*sample_callback)(float distance) = 0;

void srf10_init(void) {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	// I2C configuration
	palSetPadMode(SCL_GPIO, SCL_PAD,
			PAL_MODE_ALTERNATE(GPIO_AF_I2C1) |
			PAL_STM32_OTYPE_OPENDRAIN |
			PAL_STM32_OSPEED_MID1);
	palSetPadMode(SDA_GPIO, SDA_PAD,
			PAL_MODE_ALTERNATE(GPIO_AF_I2C1) |
			PAL_STM32_OTYPE_OPENDRAIN |
			PAL_STM32_OSPEED_MID1);
	chThdSleepMilliseconds(10);

	I2C_DEV.state = I2C_STOP;
	i2cStart(&I2C_DEV, &i2cfg);

	srf10_reset_i2c();

	chThdCreateStatic(sampling_thread_wa, sizeof(sampling_thread_wa), NORMALPRIO, sampling_thread, NULL );
}

/**
 * Set function to be called when ranging is done with the measured distance. This
 * will also activate the ranging thread.
 *
 * @param func
 * The function to be called with the ranging result.
 */
void srf10_set_sample_callback(void (*func)(float distance)) {
	sample_callback = func;
}

msg_t srf10_set_gain(uint8_t gain) {
	unsigned char tx_buf[2];

	tx_buf[0] = SRF10_GAIN_REG;
	tx_buf[1] = gain;

	i2cAcquireBus(&I2C_DEV);
	msg_t res = i2cMasterTransmitTimeout(&I2C_DEV, SRF10_DEFAULT_ADDR,
			tx_buf,2, 0, 0, MS2ST(SRF10_I2C_TIMEOUT));
	i2cReleaseBus(&I2C_DEV);

	return res;
}

msg_t srf10_set_range(uint8_t range) {
	unsigned char tx_buf[2];

	tx_buf[0] = SRF10_RANGE_REG;
	tx_buf[1] = range;

	i2cAcquireBus(&I2C_DEV);
	msg_t res = i2cMasterTransmitTimeout(&I2C_DEV, SRF10_DEFAULT_ADDR,
			tx_buf, 2, 0, 0, MS2ST(SRF10_I2C_TIMEOUT));
	i2cReleaseBus(&I2C_DEV);

	return res;
}

msg_t srf10_start_ranging(void) {
	unsigned char tx_buf[2];

	tx_buf[0] = SRF10_MEAS_REG;
	tx_buf[1] = 0x51;

	i2cAcquireBus(&I2C_DEV);
	msg_t res = i2cMasterTransmitTimeout(&I2C_DEV, SRF10_DEFAULT_ADDR,
			tx_buf, 2, 0, 0, MS2ST(SRF10_I2C_TIMEOUT));
	i2cReleaseBus(&I2C_DEV);

	return res;
}

msg_t srf10_read_range(float *range) {
	unsigned char rx_buf[2];
	unsigned char tx_buf[1];

	// Read measurement
	tx_buf[0] = SRF10_RANGE_REG;

	i2cAcquireBus(&I2C_DEV);
	msg_t res = i2cMasterTransmitTimeout(&I2C_DEV, SRF10_DEFAULT_ADDR,
			tx_buf, 1, rx_buf, 2, MS2ST(SRF10_I2C_TIMEOUT));
	i2cReleaseBus(&I2C_DEV);

	if (res == MSG_OK) {
				uint16_t tmp = rx_buf[0] << 8 | rx_buf[1];
				*range = (float)tmp / 100.0;
	} else {
		*range = -1.0;
	}

	return res;
}

msg_t srf10_reset_i2c(void) {
	msg_t res = MSG_OK;

	palSetPadMode(SCL_GPIO, SCL_PAD,
			PAL_STM32_OTYPE_OPENDRAIN |
			PAL_STM32_OSPEED_MID1);

	for(int i = 0;i < 16;i++) {
		palClearPad(SCL_GPIO, SCL_PAD);
		delay_short();
		palSetPad(SCL_GPIO, SCL_PAD);
		delay_short();
	}

	palSetPadMode(SCL_GPIO, SCL_PAD,
			PAL_MODE_ALTERNATE(GPIO_AF_I2C1) |
			PAL_STM32_OTYPE_OPENDRAIN |
			PAL_STM32_OSPEED_MID1);

	I2C_DEV.state = I2C_STOP;
	i2cStart(&I2C_DEV, &i2cfg);
	chThdSleepMicroseconds(1000);

	return res;
}

static THD_FUNCTION(sampling_thread, arg) {
	(void)arg;
	chRegSetThreadName("SRF10 Sampling");

	systime_t iteration_timer = chVTGetSystemTime();
	float ultrasonic_range = 0.0;
	int wrong_sample_cnt = 0;
	systime_t time_start;

	for(;;) {
		if (!sample_callback) {
			chThdSleepMicroseconds(ITERATION_TIME_US);
			time_start = chVTGetSystemTime();
			continue;
		}

		float range = 0;

		msg_t res = srf10_set_gain(5);

		if (res == MSG_OK) {
			chThdSleepMilliseconds(1);
			res = srf10_set_range(92); // Max 4 m
		}

		if (res == MSG_OK) {
			chThdSleepMilliseconds(1);
			res = srf10_start_ranging();
		}

		if (res == MSG_OK) {
			chThdSleepMilliseconds(50);
		}

		if (srf10_read_range(&range) == MSG_OK) {
			if (range > 0.01 && range < 2.5) { // Only trust the sensor up to 2.5 meters
				if (fabsf(ultrasonic_range - range) < SAMPLE_MAX_DIFF ||
						wrong_sample_cnt >= MAX_WRONG_SAMPLES) {
					ultrasonic_range = range;
					wrong_sample_cnt = 0;

					if (sample_callback) {
						sample_callback(range);
					}
				} else {
					wrong_sample_cnt++;
				}
			}
		} else {
			srf10_reset_i2c();
		}

		iteration_timer += US2ST(ITERATION_TIME_US);
		time_start = chVTGetSystemTime();
		if (iteration_timer > time_start) {
			chThdSleep(iteration_timer - time_start);
		} else {
			chThdSleepMicroseconds(ITERATION_TIME_US);
			iteration_timer = chVTGetSystemTime();
		}
	}
}

static void delay_short(void) {
	for (volatile int i = 0;i < 100;i++) {
		__NOP();
	}
}
