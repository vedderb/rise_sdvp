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

#include "utils.h"
#include "packet.h"
#include "comm_can.h"
#include "conf_general.h"
#include "led.h"
#include "comm_usb.h"
#include "deca_port.h"
#include "deca_device_api.h"
#include "ext_cb.h"
#include "deca_range.h"
#include "comm_uart.h"
#include "mpu9150.h"

// Settings
#define TEST_MODE		0

#if TEST_MODE
static void range_func(float dist, uint8_t id) {
	led_toggle(LED_GREEN);
	comm_usb_printf("Dist to %u: %.0f cm\r\n", id, (double)(dist * 100.0));
}

static void data_func(uint8_t sender, uint8_t *buffer, int len) {
	comm_usb_printf("Data from %d\r\n", sender);
	for (int i = 0;i < len;i++) {
		comm_usb_printf("%d\r\n", buffer[i]);
	}

	comm_usb_printf("\r\n");
}

static void mpu_read(void) {
	static int a = 0;
	a++;
	if (a > 100) {
		a = 0;
		float accel[3], gyro[3], mag[3];
		mpu9150_get_accel_gyro_mag(accel, gyro, mag);

		comm_usb_printf("%.2f %.2f %.2f\r\n",
				(double)mag[0], (double)mag[1], (double)mag[2]);
	}
}
#endif

int main(void) {
	halInit();
	chSysInit();

	conf_general_init();
	comm_can_init();
	led_init();
	comm_usb_init();
	deca_port_init();
	ext_cb_init();
	comm_uart_init();
	mpu9150_init();

	deca_port_reset();
	deca_port_set_spi_slow();
	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
        for (;;) {
        	comm_usb_printf("Deca init failed\r\n");
        	chThdSleepMilliseconds(1000);
        }
    } else {
//    	comm_usb_printf("Deca init OK\r\n");
    }
	deca_port_set_spi_fast();

	/*
	 * IDs
	 * Module 1: 38
	 * Module 2: 131
	 * Module 3: 35
	 * Module 4: 189
	 */

	deca_range_configure(main_id);

#if TEST_MODE
	deca_range_set_range_func(range_func);
	deca_range_set_data_func(data_func);

	mpu9150_set_read_callback(mpu_read);

	for(;;) {
		if (main_id == 35) {
			led_toggle(LED_RED);
			deca_range_measure(234, 6);
			chThdSleepMilliseconds(100);
		} else {
			uint8_t buffer[5];
			buffer[0] = 12;
			buffer[1] = 0;
			buffer[2] = 44;
			buffer[3] = 9;
			buffer[4] = 31;
			deca_range_send_data(50, buffer, 5);
			chThdSleepMilliseconds(1000);
		}

		chThdSleepMilliseconds(100);
		comm_usb_printf("ID: %d\r\n", main_id);
	}
#else
	for(;;) {
		packet_timerfunc();
		chThdSleepMilliseconds(1);
	}
#endif
}
