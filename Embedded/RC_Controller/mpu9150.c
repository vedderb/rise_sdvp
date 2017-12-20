/*
	Copyright 2013-2015 Benjamin Vedder	benjamin@vedder.se

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

/*
 * Note: This driver also works for the MPU9250
 */

#include "mpu9150.h"
#include "chprintf.h"
#include "utils.h"
#include "stm32f4xx_conf.h"
#include <string.h>
#include <math.h>

// Settings
#define USE_MAGNETOMETER		1
#define MPU_I2C_TIMEOUT			10
#define MAG_DIV 				10
#define ITERATION_TIME_US		1000
#define FAIL_DELAY_US			1000
#define MIN_ITERATION_DELAY_US	500
#define MAX_IDENTICAL_READS		5
#define MPU_ADDR1				0x68
#define MPU_ADDR2				0x69

// HW Settings
#define SCL_GPIO				GPIOB
#define SCL_PAD					10
#define SDA_GPIO				GPIOB
#define SDA_PAD					11
#define I2C_DEV					I2CD2

// Private variables
static unsigned char rx_buf[100];
static unsigned char tx_buf[100];
static THD_WORKING_AREA(mpu_thread_wa, 2048);
static volatile int16_t raw_accel_gyro_mag[9];
static volatile int16_t raw_accel_gyro_mag_no_offset[9];
static volatile int failed_reads;
static volatile int failed_mag_reads;
static volatile systime_t last_update_time;
static volatile systime_t update_time_diff;
static volatile int mag_updated;
static volatile uint16_t mpu_addr;
static volatile bool is_mpu9250;

// Public variables
volatile int16_t mpu9150_gyro_offsets[3];

// I2C configuration
static const I2CConfig i2cfg = {
		OPMODE_I2C,
		400000,
		FAST_DUTY_CYCLE_2
};

// Private functions
static int reset_init_mpu(void);
static int get_raw_accel_gyro(int16_t* accel_gyro);
static uint8_t read_single_reg(uint8_t reg);
#if USE_MAGNETOMETER
static int get_raw_mag(int16_t* mag);
#endif
static void delay_short(void);
static THD_FUNCTION(mpu_thread, arg);

// Function pointers
static void(*read_callback)(void) = 0;

void mpu9150_init(void) {
	failed_reads = 0;
	failed_mag_reads = 0;
	read_callback = 0;
	last_update_time = 0;
	update_time_diff = 0;
	mag_updated = 0;
	mpu_addr = MPU_ADDR1;
	is_mpu9250 = 0;

	memset((void*)mpu9150_gyro_offsets, 0, sizeof(mpu9150_gyro_offsets));

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	// I2C configuration
	palSetPadMode(SCL_GPIO, SCL_PAD,
			PAL_MODE_ALTERNATE(GPIO_AF_I2C2) |
			PAL_STM32_OTYPE_OPENDRAIN |
			PAL_STM32_OSPEED_MID1);
	palSetPadMode(SDA_GPIO, SDA_PAD,
			PAL_MODE_ALTERNATE(GPIO_AF_I2C2) |
			PAL_STM32_OTYPE_OPENDRAIN |
			PAL_STM32_OSPEED_MID1);
	chThdSleepMilliseconds(10);

	I2C_DEV.state = I2C_STOP;
	i2cStart(&I2C_DEV, &i2cfg);

	reset_init_mpu();

	chThdCreateStatic(mpu_thread_wa, sizeof(mpu_thread_wa), NORMALPRIO + 1,
			mpu_thread, NULL );
}

/**
 * Determine wether this is a MPU9150 or a MPU9250.
 *
 * @return
 * false: This is a MPU9150
 * true: This is a MPU9250
 */
bool mpu9150_is_mpu9250(void) {
	return is_mpu9250;
}

void mpu9150_set_read_callback(void(*func)(void)) {
	read_callback = func;
}

/**
 * Get the amount of milliseconds that have passed since
 * IMU values were received the last time.
 */
uint32_t mpu9150_get_time_since_update(void) {
	return (systime_t)((float)chVTTimeElapsedSinceX(last_update_time) /
			((float)CH_CFG_ST_FREQUENCY / 1000.0));
}

/*
 * Get the amount of milliseconds it took to read one sample of every axis.
 */
float mpu9150_get_last_sample_duration(void) {
	return (float)update_time_diff / (float)CH_CFG_ST_FREQUENCY * 1000.0;
}

int mpu9150_get_failed_reads(void) {
	return failed_reads;
}

int mpu9150_get_failed_mag_reads(void) {
	return failed_mag_reads;
}

/*
 * Returns 1 if the magnetometer was updated in the latest iteration, 0 otherwise.
 */
int mpu9150_mag_updated(void) {
	return mag_updated;
}

void mpu9150_cmd_print(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	(void)argc;

	float accel[3];
	float gyro[3];
	float mag[3];

	for(int i = 0;i < 500; i++) {
		mpu9150_get_accel_gyro_mag(accel, gyro, mag);

		chprintf(chp,
				"Acc X : %.2f\r\n"
				"Acc Y : %.2f\r\n"
				"Acc Z : %.2f\r\n"
				"Rate X: %.2f\r\n"
				"Rate Y: %.2f\r\n"
				"Rate Z: %.2f\r\n"
				"Mag X: %.2f\r\n"
				"Mag Y: %.2f\r\n"
				"Mag Z: %.2f\r\n"
				"Failed reads: %d\r\n"
				"Failed mag reads: %d\r\n"
				"Sample duration: %.2f ms / %.2f Hz\r\n\r\n",
				(double)accel[0], (double)accel[1], (double)accel[2],
				(double)gyro[0], (double)gyro[1], (double)gyro[2],
				(double)mag[0], (double)mag[1], (double)mag[2],
				failed_reads,
				failed_mag_reads,
				(double)mpu9150_get_last_sample_duration(),
				(double)(1000.0 / mpu9150_get_last_sample_duration()));

		chThdSleepMilliseconds(20);
	}
}

void mpu9150_cmd_sample_offsets(BaseSequentialStream *chp, int argc, char *argv[]) {
	(void)argv;
	(void)argc;

	chprintf(chp, "Sampling the gyro offsets for 100 iterations...\r\n");

	mpu9150_sample_gyro_offsets(100);

	chprintf(chp,
			"Offset X: %d\r\n"
			"Offset Y: %d\r\n"
			"Offset Z: %d\r\n\r\n",
			mpu9150_gyro_offsets[0], mpu9150_gyro_offsets[1], mpu9150_gyro_offsets[2]);
}

void mpu9150_sample_gyro_offsets(uint32_t iteratons) {
	int32_t offsets[3];
	memset(offsets, 0, sizeof(offsets));

	for(uint32_t i = 0;i < iteratons;i++) {
		offsets[0] += raw_accel_gyro_mag_no_offset[3];
		offsets[1] += raw_accel_gyro_mag_no_offset[4];
		offsets[2] += raw_accel_gyro_mag_no_offset[5];
		chThdSleepMilliseconds(10);
	}

	offsets[0] /= (int32_t)iteratons;
	offsets[1] /= (int32_t)iteratons;
	offsets[2] /= (int32_t)iteratons;

	mpu9150_gyro_offsets[0] = offsets[0];
	mpu9150_gyro_offsets[1] = offsets[1];
	mpu9150_gyro_offsets[2] = offsets[2];
}

void mpu9150_get_raw_accel_gyro_mag(int16_t *accel_gyro_mag) {
	memcpy(accel_gyro_mag, (int16_t*)raw_accel_gyro_mag, sizeof(raw_accel_gyro_mag));
}

void mpu9150_get_accel_gyro_mag(float *accel, float *gyro, float *mag) {
	accel[0] = (float)raw_accel_gyro_mag[0] * 16.0 / 32768.0;
	accel[1] = (float)raw_accel_gyro_mag[1] * 16.0 / 32768.0;
	accel[2] = (float)raw_accel_gyro_mag[2] * 16.0 / 32768.0;

	gyro[0] = (float)raw_accel_gyro_mag[3] * 2000.0 / 32768.0;
	gyro[1] = (float)raw_accel_gyro_mag[4] * 2000.0 / 32768.0;
	gyro[2] = (float)raw_accel_gyro_mag[5] * 2000.0 / 32768.0;

#if USE_MAGNETOMETER
	mag[0] = (float)raw_accel_gyro_mag[6] * 1200.0 / 4096.0;
	mag[1] = (float)raw_accel_gyro_mag[7] * 1200.0 / 4096.0;
	mag[2] = (float)raw_accel_gyro_mag[8] * 1200.0 / 4096.0;
#else
	mag[0] = 0.0;
	mag[1] = 0.0;
	mag[2] = 0.0;
#endif
}

static THD_FUNCTION(mpu_thread, arg) {
	(void)arg;
	chRegSetThreadName("MPU Sampling");

	static int16_t raw_accel_gyro_mag_tmp[9];
#if USE_MAGNETOMETER
	static int mag_cnt = MAG_DIV;
#endif
	static systime_t iteration_timer = 0;
	static int identical_reads = 0;

	iteration_timer = chVTGetSystemTime();

	for(;;) {
		if (get_raw_accel_gyro(raw_accel_gyro_mag_tmp)) {
			int is_identical = 1;
			for (int i = 0;i < 6;i++) {
				if (raw_accel_gyro_mag_tmp[i] != raw_accel_gyro_mag_no_offset[i]) {
					is_identical = 0;
					break;
				}
			}

			if (is_identical) {
				identical_reads++;
			} else {
				identical_reads = 0;
			}

			if (identical_reads >= MAX_IDENTICAL_READS) {
				failed_reads++;
				chThdSleepMicroseconds(FAIL_DELAY_US);
				reset_init_mpu();
				iteration_timer = chVTGetSystemTime();
			} else {
				memcpy((uint16_t*)raw_accel_gyro_mag_no_offset, raw_accel_gyro_mag_tmp, sizeof(raw_accel_gyro_mag));
				raw_accel_gyro_mag_tmp[3] -= mpu9150_gyro_offsets[0];
				raw_accel_gyro_mag_tmp[4] -= mpu9150_gyro_offsets[1];
				raw_accel_gyro_mag_tmp[5] -= mpu9150_gyro_offsets[2];
				memcpy((uint16_t*)raw_accel_gyro_mag, raw_accel_gyro_mag_tmp, sizeof(raw_accel_gyro_mag));

				update_time_diff = chVTGetSystemTime() - last_update_time;
				last_update_time = chVTGetSystemTime();

				if (read_callback) {
					read_callback();
				}

#if USE_MAGNETOMETER
				mag_cnt++;
				if (mag_cnt >= MAG_DIV) {
					mag_cnt = 0;
					mag_updated = 1;

					int16_t raw_mag_tmp[3];

					if (get_raw_mag(raw_mag_tmp)) {
						memcpy((uint16_t*)raw_accel_gyro_mag_tmp + 6, raw_mag_tmp, sizeof(raw_mag_tmp));
					} else {
						failed_mag_reads++;
						chThdSleepMicroseconds(FAIL_DELAY_US);
						reset_init_mpu();
						iteration_timer = chVTGetSystemTime();
					}
				} else {
					mag_updated = 0;
				}
#endif
			}
		} else {
			failed_reads++;
			chThdSleepMicroseconds(FAIL_DELAY_US);
			reset_init_mpu();
			iteration_timer = chVTGetSystemTime();
		}

		iteration_timer += US2ST(ITERATION_TIME_US);
		systime_t time_start = chVTGetSystemTime();
		if (iteration_timer > time_start) {
			chThdSleep(iteration_timer - time_start);
		} else {
			chThdSleepMicroseconds(MIN_ITERATION_DELAY_US);
			iteration_timer = chVTGetSystemTime();
		}
	}
}

static int reset_init_mpu(void) {
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

	// Set clock source to gyro x
	i2cAcquireBus(&I2C_DEV);
	tx_buf[0] = MPU9150_PWR_MGMT_1;
	tx_buf[1] = 0x01;
	res = i2cMasterTransmitTimeout(&I2C_DEV, mpu_addr, tx_buf, 2, rx_buf, 0, MPU_I2C_TIMEOUT);
	i2cReleaseBus(&I2C_DEV);

	// Try the other address
	if (res != MSG_OK) {
		if (mpu_addr == MPU_ADDR1) {
			mpu_addr = MPU_ADDR2;
		} else {
			mpu_addr = MPU_ADDR1;
		}

		// Set clock source to gyro x
		i2cAcquireBus(&I2C_DEV);
		tx_buf[0] = MPU9150_PWR_MGMT_1;
		tx_buf[1] = 0x01;
		res = i2cMasterTransmitTimeout(&I2C_DEV, mpu_addr, tx_buf, 2, rx_buf, 0, MPU_I2C_TIMEOUT);
		i2cReleaseBus(&I2C_DEV);

		if (res != MSG_OK) {
			return 0;
		}
	}

	// Set accelerometer full-scale range to +/- 16g
	i2cAcquireBus(&I2C_DEV);
	tx_buf[0] = MPU9150_ACCEL_CONFIG;
	tx_buf[1] = MPU9150_ACCEL_FS_16 << MPU9150_ACONFIG_AFS_SEL_BIT;
	res = i2cMasterTransmitTimeout(&I2C_DEV, mpu_addr, tx_buf, 2, rx_buf, 0, MPU_I2C_TIMEOUT);
	i2cReleaseBus(&I2C_DEV);

	if (res != MSG_OK) {
		return 0;
	}

	// Set gyroscope full-scale range to +/- 2000 deg/s
	i2cAcquireBus(&I2C_DEV);
	tx_buf[0] = MPU9150_GYRO_CONFIG;
	tx_buf[1] = MPU9150_GYRO_FS_2000 << MPU9150_GCONFIG_FS_SEL_BIT;
	res = i2cMasterTransmitTimeout(&I2C_DEV, mpu_addr, tx_buf, 2, rx_buf, 0, MPU_I2C_TIMEOUT);
	i2cReleaseBus(&I2C_DEV);

	if (res != MSG_OK) {
		return 0;
	}

	// Set low pass filter to 256Hz (1ms delay)
	i2cAcquireBus(&I2C_DEV);
	tx_buf[0] = MPU9150_CONFIG;
	tx_buf[1] = MPU9150_DLPF_BW_256;
	res = i2cMasterTransmitTimeout(&I2C_DEV, mpu_addr, tx_buf, 2, rx_buf, 0, MPU_I2C_TIMEOUT);
	i2cReleaseBus(&I2C_DEV);

	if (res != MSG_OK) {
		return 0;
	}

#if USE_MAGNETOMETER
	// Set the i2c bypass enable pin to true to access the magnetometer
	i2cAcquireBus(&I2C_DEV);
	tx_buf[0] = MPU9150_INT_PIN_CFG;
	tx_buf[1] = 0x02;
	res = i2cMasterTransmitTimeout(&I2C_DEV, mpu_addr, tx_buf, 2, rx_buf, 0, MPU_I2C_TIMEOUT);
	i2cReleaseBus(&I2C_DEV);

	if (res != MSG_OK) {
		return 0;
	}
#endif

	is_mpu9250 = read_single_reg(MPU9150_WHO_AM_I) == 0x71;

	return 1;
}

static int get_raw_accel_gyro(int16_t* accel_gyro) {
	msg_t res = MSG_OK;

	tx_buf[0] = MPU9150_ACCEL_XOUT_H;
	i2cAcquireBus(&I2C_DEV);
	res = i2cMasterTransmitTimeout(&I2C_DEV, mpu_addr, tx_buf, 1, rx_buf, 14, MPU_I2C_TIMEOUT);
	i2cReleaseBus(&I2C_DEV);

	if (res != MSG_OK) {
		return 0;
	}

	// Acceleration
	for (int i = 0;i < 3; i++) {
		accel_gyro[i] = ((int16_t) ((uint16_t) rx_buf[2 * i] << 8)
				+ rx_buf[2 * i + 1]);
	}

	// Angular rate
	for (int i = 4;i < 7; i++) {
		accel_gyro[i - 1] = ((int16_t) ((uint16_t) rx_buf[2 * i] << 8)
				+ rx_buf[2 * i + 1]);
	}

	return 1;
}

static uint8_t read_single_reg(uint8_t reg) {
	msg_t res = MSG_OK;

	uint8_t rxb[2];
	uint8_t txb[2];

	txb[0] = reg;
	i2cAcquireBus(&I2C_DEV);
	res = i2cMasterTransmitTimeout(&I2C_DEV, mpu_addr, txb, 1, rxb, 1, MPU_I2C_TIMEOUT);
	i2cReleaseBus(&I2C_DEV);

	if (res != MSG_OK) {
		return 0;
	} else {
		return rxb[0];
	}
}

#if USE_MAGNETOMETER
static int get_raw_mag(int16_t* mag) {
	msg_t res = MSG_OK;

	tx_buf[0] = MPU9150_HXL;
	i2cAcquireBus(&I2C_DEV);
	res = i2cMasterTransmitTimeout(&I2C_DEV, 0x0C, tx_buf, 1, rx_buf, 6, MPU_I2C_TIMEOUT);
	i2cReleaseBus(&I2C_DEV);

	if (res != MSG_OK) {
		return 0;
	}

	for (int i = 0; i < 3; i++) {
		mag[i] = ((int16_t) ((uint16_t) rx_buf[2 * i + 1] << 8) + rx_buf[2 * i]);
	}

	// Start the measurement for the next iteration
	i2cAcquireBus(&I2C_DEV);
	tx_buf[0] = MPU9150_CNTL;
	tx_buf[1] = 0x01;
	res = i2cMasterTransmitTimeout(&I2C_DEV, 0x0C, tx_buf, 2, rx_buf, 0, MPU_I2C_TIMEOUT);
	i2cReleaseBus(&I2C_DEV);

	if (res != MSG_OK) {
		return 0;
	}

	return 1;
}
#endif

static void delay_short(void) {
	for (volatile int i = 0;i < 100;i++) {
		__NOP();
	}
}
