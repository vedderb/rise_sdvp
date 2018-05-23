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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

// Private variables
static mutex_t m_print;

void comm_usb_init(void) {
	chMtxObjectInit(&m_print);
	comm_usb_serial_init();
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
