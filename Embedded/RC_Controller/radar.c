#include "conf_general.h"
#include "commands.h"
#include "ch.h"
#include "hal.h"
#include "digital_filter.h"
#include "pos.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

// Settings
#define UART_DEV		UARTD1
#define DATA_RAW		1
#define PLOT_MODE		0 // 0 = off, 1 = sample, 2 = fft

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
		chEvtSignal(radar_tp, (eventmask_t) 1);
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

	m_settings.f_center = RADAR_CENTER_FREQ;
	m_settings.f_span = RADAR_FREQ_SPAN;
	m_settings.points = RADAR_FREQ_PONTS;
	m_settings.t_sweep = RADAR_SWEEP_TIME;
	m_settings.cc_x = 0.0;
	m_settings.cc_y = 0.0;
	m_settings.cc_rad = 10.0;
	m_settings.log_en = true;
	m_settings.log_rate_ms = 1000;

	chThdCreateStatic(radar_thread_wa, sizeof(radar_thread_wa), NORMALPRIO, radar_thread, NULL);
	chThdCreateStatic(radar_log_thread_wa, sizeof(radar_log_thread_wa), NORMALPRIO, radar_log_thread, NULL);
}

void radar_setup_measurement(radar_settings_t *settings) {
	if (memcmp(&m_settings, settings, sizeof(radar_settings_t)) == 0) {
		return;
	}

	m_settings = *settings;
	commands_printf_log_usb("// Radius %.3f m\n", (double)m_settings.cc_rad);

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
		commands_printf("To radar:%s", print_buffer);
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

#if PLOT_MODE == 1
			commands_init_plot("Sample", "Amplitude");
			for (int i = 0;i < m_settings.points;i++) {
				commands_send_plot_points(i, samples[i]);
				chThdSleepMilliseconds(1);
			}
#elif PLOT_MODE == 2
			commands_init_plot("Distance [m]", "Power");
			for (int i = 0;i < 512;i++) {
				const float c = 3e8; // Speed of light
				const float deltaD = c / (2.0 * m_settings.f_span); // Resolution of FFT
				commands_send_plot_points((float)i * deltaD, samples_fft[i]);
//				commands_send_plot_points((float)i * deltaD, 20.0 * log10f(samples_fft[i]));
				chThdSleepMilliseconds(1);
			}
#endif

			if (m_settings.log_en) {
				commands_printf_log_usb(
						"%.3f " // cc_x
						"%.3f " // cc_y
						"%.3f " // c_rad
						"%.3f " // px
						"%.3f " // py
						"%.2f", // yaw
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
				commands_printf("RX Done!");
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
			commands_printf("From radar:%s", buf);
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
			radar_sample();
		}

		time_p += MS2ST(m_settings.log_rate_ms);
		systime_t time = chVTGetSystemTimeX();

		if (time_p >= time + 5) {
			chThdSleepUntil(time_p);
		} else {
			chThdSleepMilliseconds(1);
		}
	}
}

#endif
