/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "bmi160_wrapper.h"
#include "conf_general.h"

#include <stdio.h>
#include <string.h>

#if HAS_BMI160

// Threads
static THD_FUNCTION(bmi_thread, arg);
static THD_WORKING_AREA(bmi_thread_wa, 2048);

// Private functions
static bool reset_init_bmi(void);
void user_delay_ms(uint32_t ms);
static int8_t user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
static int8_t user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

// Private
static i2c_bb_state m_i2c_bb;
static void(*read_callback)(float *accel, float *gyro, float *mag) = 0;
static struct bmi160_dev sensor;
static int rate_hz;

void bmi160_wrapper_init(int samp_rate_hz) {
	rate_hz = samp_rate_hz;

	m_i2c_bb.sda_gpio = GPIOB;
	m_i2c_bb.sda_pin = 11;
	m_i2c_bb.scl_gpio = GPIOB;
	m_i2c_bb.scl_pin = 10;
	i2c_bb_init(&m_i2c_bb);

	sensor.id = BMI160_I2C_ADDR;
	sensor.interface = BMI160_I2C_INTF;
	sensor.read = user_i2c_read;
	sensor.write = user_i2c_write;

	if (reset_init_bmi()) {
		chThdCreateStatic(bmi_thread_wa, sizeof(bmi_thread_wa),
				NORMALPRIO, bmi_thread, NULL);
	}
}

void bmi160_wrapper_set_read_callback(
		void(*func)(float *accel, float *gyro, float *mag)) {
	read_callback = func;
}

static bool reset_init_bmi(void) {
	sensor.delay_ms = user_delay_ms;

	bmi160_init(&sensor);

	sensor.accel_cfg.odr = BMI160_ACCEL_ODR_200HZ;
	sensor.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
	sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
	sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

	sensor.gyro_cfg.odr = BMI160_GYRO_ODR_200HZ;
	sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
	sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
	sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

	int8_t res = bmi160_set_sens_conf(&sensor);

	return res == BMI160_OK;
}

void user_delay_ms(uint32_t ms) {
	chThdSleepMilliseconds(ms);
}

static THD_FUNCTION(bmi_thread, arg) {
	(void)arg;

	chRegSetThreadName("BMI Sampling");

	for(;;) {
		struct bmi160_sensor_data accel;
		struct bmi160_sensor_data gyro;

		int8_t res = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL),
				&accel, &gyro, &sensor);

		if (res != BMI160_OK) {
			chThdSleepMilliseconds(5);
			continue;
		}

		float tmp_accel[3], tmp_gyro[3], tmp_mag[3];

		tmp_accel[0] = (float)accel.x * 16.0 / 32768.0;
		tmp_accel[1] = (float)accel.y * 16.0 / 32768.0;
		tmp_accel[2] = (float)accel.z * 16.0 / 32768.0;

		tmp_gyro[0] = (float)gyro.x * 2000.0 / 32768.0;
		tmp_gyro[1] = (float)gyro.y * 2000.0 / 32768.0;
		tmp_gyro[2] = (float)gyro.z * 2000.0 / 32768.0;

		memset(tmp_mag, 0, sizeof(tmp_mag));

		if (read_callback) {
			read_callback(tmp_accel, tmp_gyro, tmp_mag);
		}

		chThdSleepMicroseconds(1000000 / rate_hz);
	}
}

static int8_t user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	m_i2c_bb.has_error = 0;
	uint8_t txbuf[1];
	txbuf[0] = reg_addr;
	return i2c_bb_tx_rx(&m_i2c_bb, dev_addr, txbuf, 1, data, len) ? BMI160_OK : BMI160_E_COM_FAIL;
}

static int8_t user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
	m_i2c_bb.has_error = 0;
	uint8_t txbuf[len + 1];
	txbuf[0] = reg_addr;
	memcpy(txbuf + 1, data, len);
	return i2c_bb_tx_rx(&m_i2c_bb, dev_addr, txbuf, len + 1, 0, 0) ? BMI160_OK : BMI160_E_COM_FAIL;
}

#endif
