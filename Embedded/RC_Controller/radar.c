/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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

#include "radar.h"
#include "conf_general.h"
#include "commands.h"
#include "ch.h"
#include "hal.h"
#include "digital_filter.h"
#include "pos.h"
#include "utils.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// Settings
#define UART_DEV		UARTD1
#define DATA_RAW		1

#if RADAR_EN

// Private variables
static radar_settings_t m_settings;

static uint8_t m_rx_buf[1536 * 6];
static char m_print_buf[256];
static unsigned int m_print_buf_w = 0;
static unsigned int m_print_buf_r = 0;
static bool m_wait_radar = false;
static bool m_sampling_done = false;
static bool m_print_enabled = true;
static mutex_t m_mutex_radar;

// Private functions
static void write_blocking(unsigned char *data, unsigned int len);
static void printf_blocking(char* format, ...);
static void radar_wait(void);

// Threads
static THD_WORKING_AREA(radar_thread_wa, 2048);
static THD_FUNCTION(radar_thread, arg);
static thread_t *radar_tp;

static THD_WORKING_AREA(radar_log_thread_wa, 2048);
static THD_FUNCTION(radar_log_thread, arg);

/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {
	(void)uartp;

}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {
	(void)uartp;
	(void)e;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
	(void)uartp;
	(void)c;

	m_print_buf[m_print_buf_w++] = c;

	if (m_print_buf_w >= sizeof(m_print_buf)) {
		m_print_buf_w = 0;
	}

	if (c == '\n') {
		chSysLockFromISR();
		chEvtSignalI(radar_tp, (eventmask_t) 1);
		chSysUnlockFromISR();
	}
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
	(void)uartp;
	m_sampling_done = true;
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg_1 = {
		txend1,
		txend2,
		rxend,
		rxchar,
		rxerr,
		115200,
		0,
		USART_CR2_LINEN,
		0
};

void radar_init(void) {
	uartStart(&UART_DEV, &uart_cfg_1);
	palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(7));
	palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(7));

	chMtxObjectInit(&m_mutex_radar);

	m_settings.f_center = RADAR_CENTER_FREQ;
	m_settings.f_span = RADAR_FREQ_SPAN;
	m_settings.points = RADAR_FREQ_PONTS;
	m_settings.t_sweep = RADAR_SWEEP_TIME;
	m_settings.map_plot_avg_factor = RADAR_MAP_PLOT_AVG_FACTOR;
	m_settings.map_plot_max_div = RADAR_MAP_PLOT_MAX_DIV;
	m_settings.plot_mode = RADAR_PLOT_MODE;
	m_settings.map_plot_start = RADAR_MAP_PLOT_START;
	m_settings.map_plot_end = RADAR_MAP_PLOT_END;
	m_settings.cc_x = 0.0;
	m_settings.cc_y = 0.0;
	m_settings.cc_rad = 10.0;
	m_settings.log_en = false;
	m_settings.log_rate_ms = 1000;

	chThdCreateStatic(radar_thread_wa, sizeof(radar_thread_wa), NORMALPRIO, radar_thread, NULL);
	chThdCreateStatic(radar_log_thread_wa, sizeof(radar_log_thread_wa), NORMALPRIO, radar_log_thread, NULL);
}

void radar_setup_measurement(radar_settings_t *settings) {
	if (memcmp(&m_settings, settings, sizeof(radar_settings_t)) == 0) {
		return;
	}

	m_settings = *settings;

	int hh, mm, ss;
	utils_ms_to_hhmmss(pos_get_ms_today(), &hh, &mm, &ss);

	commands_printf_log_usb("RcCar. UTC Time:  %02d:%02d:%02d\n", hh, mm, ss);
	commands_printf_log_usb("//FreqPoints: %d\n", m_settings.points);
	commands_printf_log_usb("//FreqSpan: %.1f\n", (double)m_settings.f_span);
	commands_printf_log_usb("//CenterFreq: %.1f\n", (double)m_settings.f_center);
	commands_printf_log_usb("//SweepNumbers: %d\n", 1);
	commands_printf_log_usb("//SweepTime: %.3f\n", (double)m_settings.t_sweep);
	commands_printf_log_usb("//SweepType: circular\n");
	commands_printf_log_usb("//StartAngle: 0.0\n");
	commands_printf_log_usb("//StopAngle: 360.0\n");
	commands_printf_log_usb("//StartRadius %.3f m\n", (double)m_settings.cc_rad);
	commands_printf_log_usb("//StopRadius %.3f m\n", (double)m_settings.cc_rad);
	commands_printf_log_usb("//RadarHeight: 1.1\n");
	commands_printf_log_usb("//RadarElevationAngle: 0.0\n");
	commands_printf_log_usb("//RadarAngle: 0.0\n");
	commands_printf_log_usb("//RadarPolarization: v\n");
	commands_printf_log_usb("//Comment: RcCar\n");
	commands_printf_log_usb(" --------------------------------- \n");
	commands_printf_log_usb("//utc_time_today_ms cc_x cc_y c_rad px py yaw RawData \n");
	commands_printf_log_usb(" --------------------------------- \n");

	radar_reset_setup();
}

void radar_reset_setup(void) {
	chMtxLock(&m_mutex_radar);
	uartStopReceive(&UART_DEV);
	printf_blocking("\r");
	radar_wait();
	printf_blocking("HARD:SYST W\r");
	radar_wait();
	printf_blocking("INIT\r");
	radar_wait();
	printf_blocking("SWEEP:MEAS ON\r");
	radar_wait();
	printf_blocking("TRIG:SOURCE:IMMEDIATE\r");
	radar_wait();
	printf_blocking("FREQ:CENTER %e\r", (double)m_settings.f_center);
	radar_wait();
	printf_blocking("FREQ:SPAN %e\r", (double)m_settings.f_span);
	radar_wait();
	printf_blocking("FREQ:POINTS %i\r", m_settings.points);
	radar_wait();
	printf_blocking("SWEEP:NUMBERS 1\r");
	radar_wait();
	printf_blocking("SWEEP:TIME %e\r", (double)m_settings.t_sweep);
	radar_wait();

	chThdSleepMilliseconds(1000);

	chMtxUnlock(&m_mutex_radar);
}

const radar_settings_t *radar_get_settings(void) {
	return &m_settings;
}

void radar_setup_measurement_default(void) {
	radar_settings_t s = m_settings;
	s.f_center = RADAR_CENTER_FREQ;
	s.f_span = RADAR_FREQ_SPAN;
	s.points = RADAR_FREQ_PONTS;
	s.t_sweep = RADAR_SWEEP_TIME;

	radar_setup_measurement(&s);
}

void radar_sample(void) {
	uartStopReceive(&UART_DEV);
	printf_blocking("TRIG:ARM\r");
	radar_wait();
#if DATA_RAW
	uartStartReceive(&UART_DEV, m_settings.points * 2, m_rx_buf);
	printf_blocking("TRACE:RAW ?\r");
#else
	uartStartReceive(&UART_DEV, m_freq_points * 6, m_rx_buf);
	printf_blocking("TRACE:DATA ?\r");
#endif
}

void radar_cmd(char *cmd) {
	printf_blocking("%s\r", cmd);
}

static void write_blocking(unsigned char *data, unsigned int len) {
	// Wait for the previous transmission to finish.
	while (UART_DEV.txstate == UART_TX_ACTIVE) {
		chThdSleep(1);
	}

	// Copy this data to a new buffer in case the provided one is re-used
	// after this function returns.
	static uint8_t buffer[256];
	memcpy(buffer, data, len);

	uartStartSend(&UART_DEV, len, buffer);
}

static void printf_blocking(char* format, ...) {
	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[256];

	len = vsnprintf(print_buffer, 255, format, arg);
	va_end (arg);

	print_buffer[len] = '\0';

	if(len > 0) {
		m_wait_radar = true;
		write_blocking((unsigned char*)print_buffer, (len < 255) ? len : 255);
	}

	if (m_print_enabled) {
		commands_printf("%d To radar:%s", ST2MS(chVTGetSystemTimeX()), print_buffer);
	}
}

static void radar_wait(void) {
	for (int i = 0;i < 2000;i++) {
		if (!m_wait_radar) {
			return;
		}

		chThdSleepMilliseconds(1);
	}

	commands_printf("radar_wait timed out");
}

static THD_FUNCTION(radar_thread, arg) {
	(void)arg;

	chRegSetThreadName("Radar");
	radar_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		if (m_sampling_done) {
			m_sampling_done = false;

			POS_STATE pos;
			pos_get_pos(&pos);

			static float samples[1536];
			static float samples_fft[1024];
			static float im[1024];

			for (int i = 0;i < m_settings.points;i++) {
#if DATA_RAW == 1
				uint32_t value = (uint32_t)(((uint32_t)m_rx_buf[2 * i + 1] << 8) + m_rx_buf[2 * i]);
#else
				int value;
				sscanf((char*)m_rx_buf + i * 6, "%d", &value);
#endif
				samples[i] = (float)value;
			}

			float avg = 0.0;
			for (int i = 0;i < 1024;i++) {
				avg += samples[i];
			}
			avg /= 1024;

			for (int i = 0;i < 1024;i++) {
				samples_fft[i] = samples[i] - avg;
				im[i] = 0;
			}

			filter_fft(0, 10, samples_fft, im);

			for (int i = 0;i < 1024;i++) {
				samples_fft[i] = sqrtf(samples_fft[i] * samples_fft[i] + im[i] * im[i]);
			}

			// Range of interest
			const int sample_first = m_settings.map_plot_start;
			const int sample_last = m_settings.map_plot_end;

			// Look for maximum and minimum values in range of interest
			float max = 0;
			float min = 1e20;
			float avg_fft = 0.0;
			for (int i = sample_first;i < sample_last;i++) {
				if (samples_fft[i] > max) {
					max = samples_fft[i];
				}

				if (samples_fft[i] < min) {
					min = samples_fft[i];
				}

				avg_fft += samples_fft[i];
			}

			avg_fft /= (float)(sample_last - sample_first);

//			commands_printf("Max: %f Min: %f Avg: %f\n", (double)max, (double)min, (double)avg_fft);

			// Send samples that are over threshold if there is a significant
			// difference between max and min.
			// TODO: Look for absolute threshold
			if (max > (avg_fft * m_settings.map_plot_avg_factor)) {
				float vec[24];
				int vec_ind = 0;
				for (int i = sample_first;i < sample_last;i++) {
					const float val = samples_fft[i];

					if (val > (max / m_settings.map_plot_max_div)) {
						const float c = 299792458.0;
						const float deltaD = c / (2.0 * m_settings.f_span);
						const float dist = (float)i * deltaD;

						vec[vec_ind++] = dist;
						vec[vec_ind++] = val;

						if (vec_ind >= 24) {
							break;
						}
					}
				}

				if (vec_ind > 0) {
					commands_send_radar_samples(vec, vec_ind);
				}
			}

			if (m_settings.plot_mode == 1) {
				commands_init_plot("Sample", "Amplitude");
				for (int i = 0;i < m_settings.points;i++) {
					commands_send_plot_points(i, samples[i]);
					chThdSleepMilliseconds(1);
				}
			} else if (m_settings.plot_mode == 2) {
				commands_init_plot("Distance [m]", "Power");
				for (int i = 0;i < 512;i++) {
					const float c = 3e8; // Speed of light
					const float deltaD = c / (2.0 * m_settings.f_span); // Resolution of FFT
					commands_send_plot_points((float)i * deltaD, samples_fft[i]);
					//				commands_send_plot_points((float)i * deltaD, 20.0 * log10f(samples_fft[i]));
					chThdSleepMilliseconds(1);
				}
			}

			if (m_settings.log_en) {
				commands_printf_log_usb(
						"%d " // utc_time_today_ms
						"%.3f " // cc_x
						"%.3f " // cc_y
						"%.3f " // c_rad
						"%.3f " // px
						"%.3f " // py
						"%.2f", // yaw
						pos_get_ms_today(),
						(double)m_settings.cc_x,
						(double)m_settings.cc_y,
						(double)m_settings.cc_rad,
						(double)pos.px,
						(double)pos.py,
						(double)pos.yaw);
				for (int i = 0;i < m_settings.points;i++) {
					commands_printf_log_usb(" %d", (int)samples[i]);
				}

				commands_printf_log_usb("\r\n");
			}

			if (m_print_enabled) {
				commands_printf("%d RX Done!\n", ST2MS(chVTGetSystemTimeX()));
			}

			continue;
		}

		char buf[100];
		unsigned int ptr = 0;

		while (ptr < (sizeof(buf) - 1)) {
			if (m_print_buf[m_print_buf_r] == '\n') {
				m_print_buf_r++;
				if (m_print_buf_r >= sizeof(m_print_buf)) {
					m_print_buf_r = 0;
				}
				break;
			}

			buf[ptr++] = m_print_buf[m_print_buf_r++];
			if (m_print_buf_r >= sizeof(m_print_buf)) {
				m_print_buf_r = 0;
			}
		}

		if (m_print_enabled) {
			buf[ptr] = '\0';
			commands_printf("%d From radar:%s", ST2MS(chVTGetSystemTimeX()), buf);
		}

		m_wait_radar = false;
	}
}

static THD_FUNCTION(radar_log_thread, arg) {
	(void)arg;

	chRegSetThreadName("RadarLog");

	systime_t time_p = chVTGetSystemTimeX(); // T0

	for (;;) {
		if (m_settings.log_en) {
			chMtxLock(&m_mutex_radar);
			radar_sample();
			chMtxUnlock(&m_mutex_radar);
		}

		time_p += MS2ST(m_settings.log_rate_ms);
		systime_t time = chVTGetSystemTimeX();

		if (time_p >= time + 5) {
			chThdSleepUntil(time_p);
		} else {
			chThdSleepMilliseconds(m_settings.log_rate_ms);
			time_p = chVTGetSystemTimeX();
		}
	}
}

#endif
