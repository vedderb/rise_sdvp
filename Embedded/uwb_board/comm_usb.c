/*
 * comm_usb.c
 *
 *  Created on: 18 jan. 2017
 *      Author: benjamin
 */

#include "comm_usb.h"
#include "ch.h"
#include "hal.h"
#include "comm_usb_serial.h"
#include "deca_range.h"
#include "led.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

// Private variables
static mutex_t m_print;
static THD_WORKING_AREA(serial_read_thread_wa, 2048);
static THD_FUNCTION(serial_read_thread, arg);

void comm_usb_init(void) {
	chMtxObjectInit(&m_print);
	comm_usb_serial_init();
	chThdCreateStatic(serial_read_thread_wa, sizeof(serial_read_thread_wa),
			NORMALPRIO, serial_read_thread, NULL);
}

void comm_usb_printf(char* format, ...) {
	chMtxLock(&m_print);

	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[1024];

	len = vsnprintf(print_buffer, 1020, format, arg);
	va_end (arg);

	if(len > 0) {
		chSequentialStreamWrite(&SDU1, (unsigned char*)print_buffer, len);
	}

	chMtxUnlock(&m_print);
}

static void range_func(float dist, uint8_t id) {
	led_toggle(LED_GREEN);
	comm_usb_printf("%d %.2f\r\n", id, (double)dist);
}

static THD_FUNCTION(serial_read_thread, arg) {
	(void)arg;

	chRegSetThreadName("USB-Serial read");

	uint8_t buffer[128];
	static uint8_t buffer_acc[512];
	int write_ptr = 0;

	for(;;) {
		int len = chSequentialStreamRead(&SDU1, (uint8_t*)buffer, 1);

//		// Echo text
//		if (len > 0) {
//			chSequentialStreamWrite(&SDU1, (unsigned char*)buffer, len);
//		}

		for (int i = 0;i < len;i++) {
			if (buffer[i] == '\n') {
				continue;
			}

			buffer_acc[write_ptr] = buffer[i];

			if (buffer_acc[write_ptr] == '\r') {
				buffer_acc[write_ptr] = '\0';
				write_ptr = -1;

				enum { kMaxArgs = 8 };
				int argc = 0;
				char *argv[kMaxArgs];
				char *p2 = strtok((char*)buffer_acc, " ");
				while (p2 && argc < kMaxArgs) {
					argv[argc++] = p2;
					p2 = strtok(0, " ");
				}

				if (argc == 3 && strcmp(argv[0], "range") == 0) {
					int id = -1;
					int samples = -1;
					sscanf(argv[1], "%d", &id);
					sscanf(argv[2], "%d", &samples);

					if (id >= 0 && id <= 255 && samples >= 1 && samples < 500) {
						deca_range_set_range_func(range_func);
						deca_range_measure(id, samples);
					} else {
						comm_usb_printf("Invalid command arguments\r\n");
					}
				} else {
					comm_usb_printf("Invalid command: %s\r\n", buffer_acc);
				}
			}

			write_ptr++;
			write_ptr %= sizeof(buffer_acc);
		}
	}
}
