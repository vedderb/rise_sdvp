/*
	Copyright 2016-2018 Benjamin Vedder	benjamin@vedder.se

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
 * bldc_interface.c
 *
 * Compatible Firmware Versions
 * 3.39
 * 3.40
 *
 */

#include "bldc_interface.h"
#include "buffer.h"
#include <string.h>

// Private variables
static unsigned char send_buffer[1024];

// Private variables for received data
static mc_values values;
static int fw_major;
static int fw_minor;
static float rotor_pos;
static mc_configuration mcconf;
static app_configuration appconf;
static float detect_cycle_int_limit;
static float detect_coupling_k;
static signed char detect_hall_table[8];
static signed char detect_hall_res;
static float dec_ppm;
static float dec_ppm_len;
static float dec_adc;
static float dec_adc_voltage;
static float dec_chuk;

// Private functions
void send_packet_no_fwd(unsigned char *data, unsigned int len);

// Function pointers
static void(*send_func)(unsigned char *data, unsigned int len) = 0;
static void(*forward_func)(unsigned char *data, unsigned int len) = 0;

// Function pointers for received data
static void(*rx_value_func)(mc_values *values) = 0;
static void(*rx_printf_func)(char *str) = 0;
static void(*rx_fw_func)(int major, int minor) = 0;
static void(*rx_rotor_pos_func)(float pos) = 0;
static void(*rx_mcconf_func)(mc_configuration *conf) = 0;
static void(*rx_appconf_func)(app_configuration *conf) = 0;
static void(*rx_detect_func)(float cycle_int_limit, float coupling_k,
		const signed char *hall_table, signed char hall_res) = 0;
static void(*rx_dec_ppm_func)(float val, float ms) = 0;
static void(*rx_dec_adc_func)(float val, float voltage) = 0;
static void(*rx_dec_chuk_func)(float val) = 0;
static void(*rx_mcconf_received_func)(void) = 0;
static void(*rx_appconf_received_func)(void) = 0;
static void(*motor_control_set_func)(motor_control_mode mode, float value) = 0;
static void(*values_requested_func)(void) = 0;

void bldc_interface_init(void(*func)(unsigned char *data, unsigned int len)) {
	send_func = func;
}

void bldc_interface_set_forward_func(void(*func)(unsigned char *data, unsigned int len)) {
	forward_func = func;
}

/**
 * Send a packet using the set send function.
 *
 * @param data
 * The packet data.
 *
 * @param len
 * The data length.
 */
void bldc_interface_send_packet(unsigned char *data, unsigned int len) {
	if (send_func) {
		send_func(data, len);
	}
}

/**
 * Process a received buffer with commands and data.
 *
 * @param data
 * The buffer to process.
 *
 * @param len
 * The length of the buffer.
 */
void bldc_interface_process_packet(unsigned char *data, unsigned int len) {
	if (!len) {
		return;
	}

	if (forward_func) {
		forward_func(data, len);
		return;
	}

	int32_t ind = 0;
	int i = 0;
	unsigned char id = data[0];
	data++;
	len--;

	switch (id) {
	case COMM_FW_VERSION:
		if (len == 2) {
			ind = 0;
			fw_major = data[ind++];
			fw_minor = data[ind++];
		} else {
			fw_major = -1;
			fw_minor = -1;
		}
		break;

	case COMM_ERASE_NEW_APP:
	case COMM_WRITE_NEW_APP_DATA:
		// TODO
		break;

	case COMM_GET_VALUES:
		ind = 0;
		values.temp_mos = buffer_get_float16(data, 1e1, &ind);
		values.temp_motor = buffer_get_float16(data, 1e1, &ind);
		values.current_motor = buffer_get_float32(data, 1e2, &ind);
		values.current_in = buffer_get_float32(data, 1e2, &ind);
		values.id = buffer_get_float32(data, 1e2, &ind);
		values.iq = buffer_get_float32(data, 1e2, &ind);
		values.duty_now = buffer_get_float16(data, 1e3, &ind);
		values.rpm = buffer_get_float32(data, 1e0, &ind);
		values.v_in = buffer_get_float16(data, 1e1, &ind);
		values.amp_hours = buffer_get_float32(data, 1e4, &ind);
		values.amp_hours_charged = buffer_get_float32(data, 1e4, &ind);
		values.watt_hours = buffer_get_float32(data, 1e4, &ind);
		values.watt_hours_charged = buffer_get_float32(data, 1e4, &ind);
		values.tachometer = buffer_get_int32(data, &ind);
		values.tachometer_abs = buffer_get_int32(data, &ind);
		values.fault_code = (mc_fault_code)data[ind++];

		if (ind < (int)len) {
			values.pid_pos = buffer_get_float32(data, 1e6, &ind);
		} else {
			values.pid_pos = 0.0;
		}

		if (ind < (int)len) {
			values.vesc_id = data[ind++];
		} else {
			values.vesc_id = 255;
		}

		if (rx_value_func) {
			rx_value_func(&values);
		}
		break;

	case COMM_PRINT:
		if (rx_printf_func) {
			data[len] = '\0';
			rx_printf_func((char*)data);
		}
		break;

	case COMM_SAMPLE_PRINT:
		// TODO
		break;

	case COMM_ROTOR_POSITION:
		ind = 0;
		rotor_pos = buffer_get_float32(data, 100000.0, &ind);

		if (rx_rotor_pos_func) {
			rx_rotor_pos_func(rotor_pos);
		}
		break;

	case COMM_EXPERIMENT_SAMPLE:
		// TODO
		break;

	case COMM_GET_MCCONF:
	case COMM_GET_MCCONF_DEFAULT:
		ind = 0;
		mcconf.pwm_mode = data[ind++];
		mcconf.comm_mode = data[ind++];
		mcconf.motor_type = data[ind++];
		mcconf.sensor_mode = data[ind++];

		mcconf.l_current_max = buffer_get_float32_auto(data, &ind);
		mcconf.l_current_min = buffer_get_float32_auto(data, &ind);
		mcconf.l_in_current_max = buffer_get_float32_auto(data, &ind);
		mcconf.l_in_current_min = buffer_get_float32_auto(data, &ind);
		mcconf.l_abs_current_max = buffer_get_float32_auto(data, &ind);
		mcconf.l_min_erpm = buffer_get_float32_auto(data, &ind);
		mcconf.l_max_erpm = buffer_get_float32_auto(data, &ind);
		mcconf.l_erpm_start = buffer_get_float32_auto(data, &ind);
		mcconf.l_max_erpm_fbrake = buffer_get_float32_auto(data, &ind);
		mcconf.l_max_erpm_fbrake_cc = buffer_get_float32_auto(data, &ind);
		mcconf.l_min_vin = buffer_get_float32_auto(data, &ind);
		mcconf.l_max_vin = buffer_get_float32_auto(data, &ind);
		mcconf.l_battery_cut_start = buffer_get_float32_auto(data, &ind);
		mcconf.l_battery_cut_end = buffer_get_float32_auto(data, &ind);
		mcconf.l_slow_abs_current = data[ind++];
		mcconf.l_temp_fet_start = buffer_get_float32_auto(data, &ind);
		mcconf.l_temp_fet_end = buffer_get_float32_auto(data, &ind);
		mcconf.l_temp_motor_start = buffer_get_float32_auto(data, &ind);
		mcconf.l_temp_motor_end = buffer_get_float32_auto(data, &ind);
		mcconf.l_temp_accel_dec = buffer_get_float32_auto(data, &ind);
		mcconf.l_min_duty = buffer_get_float32_auto(data, &ind);
		mcconf.l_max_duty = buffer_get_float32_auto(data, &ind);
		mcconf.l_watt_max = buffer_get_float32_auto(data, &ind);
		mcconf.l_watt_min = buffer_get_float32_auto(data, &ind);

		mcconf.lo_current_max = mcconf.l_current_max;
		mcconf.lo_current_min = mcconf.l_current_min;
		mcconf.lo_in_current_max = mcconf.l_in_current_max;
		mcconf.lo_in_current_min = mcconf.l_in_current_min;
		mcconf.lo_current_motor_max_now = mcconf.l_current_max;
		mcconf.lo_current_motor_min_now = mcconf.l_current_min;

		mcconf.sl_min_erpm = buffer_get_float32_auto(data, &ind);
		mcconf.sl_min_erpm_cycle_int_limit = buffer_get_float32_auto(data, &ind);
		mcconf.sl_max_fullbreak_current_dir_change = buffer_get_float32_auto(data, &ind);
		mcconf.sl_cycle_int_limit = buffer_get_float32_auto(data, &ind);
		mcconf.sl_phase_advance_at_br = buffer_get_float32_auto(data, &ind);
		mcconf.sl_cycle_int_rpm_br = buffer_get_float32_auto(data, &ind);
		mcconf.sl_bemf_coupling_k = buffer_get_float32_auto(data, &ind);

		memcpy(mcconf.hall_table, data + ind, 8);
		ind += 8;
		mcconf.hall_sl_erpm = buffer_get_float32_auto(data, &ind);

		mcconf.foc_current_kp = buffer_get_float32_auto(data, &ind);
		mcconf.foc_current_ki = buffer_get_float32_auto(data, &ind);
		mcconf.foc_f_sw = buffer_get_float32_auto(data, &ind);
		mcconf.foc_dt_us = buffer_get_float32_auto(data, &ind);
		mcconf.foc_encoder_inverted = data[ind++];
		mcconf.foc_encoder_offset = buffer_get_float32_auto(data, &ind);
		mcconf.foc_encoder_ratio = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sensor_mode = data[ind++];
		mcconf.foc_pll_kp = buffer_get_float32_auto(data, &ind);
		mcconf.foc_pll_ki = buffer_get_float32_auto(data, &ind);
		mcconf.foc_motor_l = buffer_get_float32_auto(data, &ind);
		mcconf.foc_motor_r = buffer_get_float32_auto(data, &ind);
		mcconf.foc_motor_flux_linkage = buffer_get_float32_auto(data, &ind);
		mcconf.foc_observer_gain = buffer_get_float32_auto(data, &ind);
		mcconf.foc_observer_gain_slow = buffer_get_float32_auto(data, &ind);
		mcconf.foc_duty_dowmramp_kp = buffer_get_float32_auto(data, &ind);
		mcconf.foc_duty_dowmramp_ki = buffer_get_float32_auto(data, &ind);
		mcconf.foc_openloop_rpm = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sl_openloop_hyst = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sl_openloop_time = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sl_d_current_duty = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sl_d_current_factor = buffer_get_float32_auto(data, &ind);
		memcpy(mcconf.foc_hall_table, data + ind, 8);
		ind += 8;
		mcconf.foc_sl_erpm = buffer_get_float32_auto(data, &ind);
		mcconf.foc_sample_v0_v7 = data[ind++];
		mcconf.foc_sample_high_current = data[ind++];
		mcconf.foc_sat_comp = buffer_get_float32_auto(data, &ind);
		mcconf.foc_temp_comp = data[ind++];
		mcconf.foc_temp_comp_base_temp = buffer_get_float32_auto(data, &ind);
		mcconf.foc_current_filter_const = buffer_get_float32_auto(data, &ind);

		mcconf.s_pid_kp = buffer_get_float32_auto(data, &ind);
		mcconf.s_pid_ki = buffer_get_float32_auto(data, &ind);
		mcconf.s_pid_kd = buffer_get_float32_auto(data, &ind);
		mcconf.s_pid_kd_filter = buffer_get_float32_auto(data, &ind);
		mcconf.s_pid_min_erpm = buffer_get_float32_auto(data, &ind);
		mcconf.s_pid_allow_braking = data[ind++];

		mcconf.p_pid_kp = buffer_get_float32_auto(data, &ind);
		mcconf.p_pid_ki = buffer_get_float32_auto(data, &ind);
		mcconf.p_pid_kd = buffer_get_float32_auto(data, &ind);
		mcconf.p_pid_kd_filter = buffer_get_float32_auto(data, &ind);
		mcconf.p_pid_ang_div = buffer_get_float32_auto(data, &ind);

		mcconf.cc_startup_boost_duty = buffer_get_float32_auto(data, &ind);
		mcconf.cc_min_current = buffer_get_float32_auto(data, &ind);
		mcconf.cc_gain = buffer_get_float32_auto(data, &ind);
		mcconf.cc_ramp_step_max = buffer_get_float32_auto(data, &ind);

		mcconf.m_fault_stop_time_ms = buffer_get_int32(data, &ind);
		mcconf.m_duty_ramp_step = buffer_get_float32_auto(data, &ind);
		mcconf.m_current_backoff_gain = buffer_get_float32_auto(data, &ind);
		mcconf.m_encoder_counts = buffer_get_uint32(data, &ind);
		mcconf.m_sensor_port_mode = data[ind++];
		mcconf.m_invert_direction = data[ind++];
		mcconf.m_drv8301_oc_mode = data[ind++];
		mcconf.m_drv8301_oc_adj = data[ind++];
		mcconf.m_bldc_f_sw_min = buffer_get_float32_auto(data, &ind);
		mcconf.m_bldc_f_sw_max = buffer_get_float32_auto(data, &ind);
		mcconf.m_dc_f_sw = buffer_get_float32_auto(data, &ind);
		mcconf.m_ntc_motor_beta = buffer_get_float32_auto(data, &ind);
		mcconf.m_out_aux_mode = data[ind++];

		if (rx_mcconf_func) {
			rx_mcconf_func(&mcconf);
		}
		break;

	case COMM_GET_APPCONF:
	case COMM_GET_APPCONF_DEFAULT:
		ind = 0;
		appconf.controller_id = data[ind++];
		appconf.timeout_msec = buffer_get_uint32(data, &ind);
		appconf.timeout_brake_current = buffer_get_float32_auto(data, &ind);
		appconf.send_can_status = data[ind++];
		appconf.send_can_status_rate_hz = buffer_get_uint16(data, &ind);
		appconf.can_baud_rate = data[ind++];

		appconf.app_to_use = data[ind++];

		appconf.app_ppm_conf.ctrl_type = data[ind++];
		appconf.app_ppm_conf.pid_max_erpm = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.hyst = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.pulse_start = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.pulse_end = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.pulse_center = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.median_filter = data[ind++];
		appconf.app_ppm_conf.safe_start = data[ind++];
		appconf.app_ppm_conf.throttle_exp = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.throttle_exp_brake = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.throttle_exp_mode = data[ind++];
		appconf.app_ppm_conf.ramp_time_pos = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.ramp_time_neg = buffer_get_float32_auto(data, &ind);
		appconf.app_ppm_conf.multi_esc = data[ind++];
		appconf.app_ppm_conf.tc = data[ind++];
		appconf.app_ppm_conf.tc_max_diff = buffer_get_float32_auto(data, &ind);

		appconf.app_adc_conf.ctrl_type = data[ind++];
		appconf.app_adc_conf.hyst = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.voltage_start = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.voltage_end = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.voltage_center = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.voltage2_start = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.voltage2_end = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.use_filter = data[ind++];
		appconf.app_adc_conf.safe_start = data[ind++];
		appconf.app_adc_conf.cc_button_inverted = data[ind++];
		appconf.app_adc_conf.rev_button_inverted = data[ind++];
		appconf.app_adc_conf.voltage_inverted = data[ind++];
		appconf.app_adc_conf.voltage2_inverted = data[ind++];
		appconf.app_adc_conf.throttle_exp = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.throttle_exp_brake = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.throttle_exp_mode = data[ind++];
		appconf.app_adc_conf.ramp_time_pos = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.ramp_time_neg = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.multi_esc = data[ind++];
		appconf.app_adc_conf.tc = data[ind++];
		appconf.app_adc_conf.tc_max_diff = buffer_get_float32_auto(data, &ind);
		appconf.app_adc_conf.update_rate_hz = buffer_get_uint16(data, &ind);

		appconf.app_uart_baudrate = buffer_get_uint32(data, &ind);

		appconf.app_chuk_conf.ctrl_type = data[ind++];
		appconf.app_chuk_conf.hyst = buffer_get_float32_auto(data, &ind);
		appconf.app_chuk_conf.ramp_time_pos = buffer_get_float32_auto(data, &ind);
		appconf.app_chuk_conf.ramp_time_neg = buffer_get_float32_auto(data, &ind);
		appconf.app_chuk_conf.stick_erpm_per_s_in_cc = buffer_get_float32_auto(data, &ind);
		appconf.app_chuk_conf.throttle_exp = buffer_get_float32_auto(data, &ind);
		appconf.app_chuk_conf.throttle_exp_brake = buffer_get_float32_auto(data, &ind);
		appconf.app_chuk_conf.throttle_exp_mode = data[ind++];
		appconf.app_chuk_conf.multi_esc = data[ind++];
		appconf.app_chuk_conf.tc = data[ind++];
		appconf.app_chuk_conf.tc_max_diff = buffer_get_float32_auto(data, &ind);

		appconf.app_nrf_conf.speed = data[ind++];
		appconf.app_nrf_conf.power = data[ind++];
		appconf.app_nrf_conf.crc_type = data[ind++];
		appconf.app_nrf_conf.retry_delay = data[ind++];
		appconf.app_nrf_conf.retries = data[ind++];
		appconf.app_nrf_conf.channel = data[ind++];
		memcpy(appconf.app_nrf_conf.address, data + ind, 3);
		ind += 3;
		appconf.app_nrf_conf.send_crc_ack = data[ind++];

		if (rx_appconf_func) {
			rx_appconf_func(&appconf);
		}
		break;

	case COMM_DETECT_MOTOR_PARAM:
		ind = 0;
		detect_cycle_int_limit = buffer_get_float32(data, 1000.0, &ind);
		detect_coupling_k = buffer_get_float32(data, 1000.0, &ind);
		for (i = 0;i < 8;i++) {
			detect_hall_table[i] = (const signed char)(data[ind++]);
		}
		detect_hall_res = (const signed char)(data[ind++]);

		if (rx_detect_func) {
			rx_detect_func(detect_cycle_int_limit, detect_coupling_k,
					detect_hall_table, detect_hall_res);
		}
		break;

	case COMM_DETECT_MOTOR_R_L: {
		// TODO!
	} break;

	case COMM_DETECT_MOTOR_FLUX_LINKAGE: {
		// TODO!
	} break;

	case COMM_DETECT_ENCODER: {
		// TODO!
	} break;

	case COMM_DETECT_HALL_FOC: {
		// TODO!
	} break;

	case COMM_GET_DECODED_PPM:
		ind = 0;
		dec_ppm = buffer_get_float32(data, 1000000.0, &ind);
		dec_ppm_len = buffer_get_float32(data, 1000000.0, &ind);

		if (rx_dec_ppm_func) {
			rx_dec_ppm_func(dec_ppm, dec_ppm_len);
		}
		break;

	case COMM_GET_DECODED_ADC:
		ind = 0;
		dec_adc = buffer_get_float32(data, 1000000.0, &ind);
		dec_adc_voltage = buffer_get_float32(data, 1000000.0, &ind);
		// TODO for adc2

		if (rx_dec_adc_func) {
			rx_dec_adc_func(dec_adc, dec_adc_voltage);
		}
		break;

	case COMM_GET_DECODED_CHUK:
		ind = 0;
		dec_chuk = buffer_get_float32(data, 1000000.0, &ind);

		if (rx_dec_chuk_func) {
			rx_dec_chuk_func(dec_chuk);
		}
		break;

	case COMM_SET_MCCONF:
		// This is a confirmation that the new mcconf is received.
		if (rx_mcconf_received_func) {
			rx_mcconf_received_func();
		}
		break;

	case COMM_SET_APPCONF:
		// This is a confirmation that the new appconf is received.
		if (rx_appconf_received_func) {
			rx_appconf_received_func();
		}
		break;

	default:
		break;
	}
}

/**
 * Function pointer setters. When data that is requested with the get functions
 * is received, the corresponding function pointer will be called with the
 * received data.
 *
 * @param func
 * A function to be called when the corresponding data is received.
 */

void bldc_interface_set_rx_value_func(void(*func)(mc_values *values)) {
	rx_value_func = func;
}

void bldc_interface_set_rx_printf_func(void(*func)(char *str)) {
	rx_printf_func = func;
}

void bldc_interface_set_rx_fw_func(void(*func)(int major, int minor)) {
	rx_fw_func = func;
}

void bldc_interface_set_rx_rotor_pos_func(void(*func)(float pos)) {
	rx_rotor_pos_func = func;
}

void bldc_interface_set_rx_mcconf_func(void(*func)(mc_configuration *conf)) {
	rx_mcconf_func = func;
}

void bldc_interface_set_rx_appconf_func(void(*func)(app_configuration *conf)) {
	rx_appconf_func = func;
}

void bldc_interface_set_rx_detect_func(void(*func)(float cycle_int_limit, float coupling_k,
		const signed char *hall_table, signed char hall_res)) {
	rx_detect_func = func;
}

void bldc_interface_set_rx_dec_ppm_func(void(*func)(float val, float ms)) {
	rx_dec_ppm_func = func;
}

void bldc_interface_set_rx_dec_adc_func(void(*func)(float val, float voltage)) {
	rx_dec_adc_func = func;
}

void bldc_interface_set_rx_dec_chuk_func(void(*func)(float val)) {
	rx_dec_chuk_func = func;
}

void bldc_interface_set_rx_mcconf_received_func(void(*func)(void)) {
	rx_mcconf_received_func = func;
}

void bldc_interface_set_rx_appconf_received_func(void(*func)(void)) {
	rx_appconf_received_func = func;
}

void bldc_interface_set_sim_control_function(void(*func)(motor_control_mode mode, float value)) {
	motor_control_set_func = func;
}

void bldc_interface_set_sim_values_func(void(*func)(void)) {
	values_requested_func = func;
}

// Setters
void bldc_interface_terminal_cmd(char* cmd) {
	int len = strlen(cmd);
	send_buffer[0] = COMM_TERMINAL_CMD;
	memcpy(send_buffer + 1, cmd, len);
	send_packet_no_fwd(send_buffer, len + 1);
}

void bldc_interface_set_duty_cycle(float dutyCycle) {
	if (motor_control_set_func) {
		motor_control_set_func(MOTOR_CONTROL_DUTY, dutyCycle);
		return;
	}
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_SET_DUTY;
	buffer_append_float32(send_buffer, dutyCycle, 100000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_current(float current) {
	if (motor_control_set_func) {
		motor_control_set_func(MOTOR_CONTROL_CURRENT, current);
		return;
	}
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_SET_CURRENT;
	buffer_append_float32(send_buffer, current, 1000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_current_brake(float current) {
	if (motor_control_set_func) {
		motor_control_set_func(MOTOR_CONTROL_CURRENT_BRAKE, current);
		return;
	}
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_float32(send_buffer, current, 1000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_rpm(int rpm) {
	if (motor_control_set_func) {
		motor_control_set_func(MOTOR_CONTROL_RPM, rpm);
		return;
	}
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_SET_RPM;
	buffer_append_int32(send_buffer, rpm, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_pos(float pos) {
	if (motor_control_set_func) {
		motor_control_set_func(MOTOR_CONTROL_POS, pos);
		return;
	}
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_SET_POS;
	buffer_append_float32(send_buffer, pos, 1000000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_handbrake(float current) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_SET_HANDBRAKE;
	buffer_append_float32(send_buffer, current, 1e3, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_servo_pos(float pos) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_SET_SERVO_POS;
	buffer_append_float16(send_buffer, pos, 1000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_set_mcconf(const mc_configuration *mcconf) {
	int32_t ind = 0;
	send_buffer[ind++] = COMM_SET_MCCONF;

	send_buffer[ind++] = mcconf->pwm_mode;
	send_buffer[ind++] = mcconf->comm_mode;
	send_buffer[ind++] = mcconf->motor_type;
	send_buffer[ind++] = mcconf->sensor_mode;

	buffer_append_float32_auto(send_buffer, mcconf->l_current_max, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_current_min, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_in_current_max, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_in_current_min, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_abs_current_max, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_min_erpm, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_max_erpm, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_erpm_start, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_max_erpm_fbrake, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_max_erpm_fbrake_cc, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_min_vin, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_max_vin, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_battery_cut_start, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_battery_cut_end, &ind);
	send_buffer[ind++] = mcconf->l_slow_abs_current;
	buffer_append_float32_auto(send_buffer, mcconf->l_temp_fet_start, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_temp_fet_end, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_temp_motor_start, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_temp_motor_end, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_temp_accel_dec, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_min_duty, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_max_duty, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_watt_max, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->l_watt_min, &ind);

	buffer_append_float32_auto(send_buffer, mcconf->sl_min_erpm, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->sl_min_erpm_cycle_int_limit, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->sl_max_fullbreak_current_dir_change, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->sl_cycle_int_limit, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->sl_phase_advance_at_br, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->sl_cycle_int_rpm_br, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->sl_bemf_coupling_k, &ind);

	memcpy(send_buffer + ind, mcconf->hall_table, 8);
	ind += 8;
	buffer_append_float32_auto(send_buffer, mcconf->hall_sl_erpm, &ind);

	buffer_append_float32_auto(send_buffer, mcconf->foc_current_kp, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_current_ki, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_f_sw, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_dt_us, &ind);
	send_buffer[ind++] = mcconf->foc_encoder_inverted;
	buffer_append_float32_auto(send_buffer, mcconf->foc_encoder_offset, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_encoder_ratio, &ind);
	send_buffer[ind++] = mcconf->foc_sensor_mode;
	buffer_append_float32_auto(send_buffer, mcconf->foc_pll_kp, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_pll_ki, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_motor_l, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_motor_r, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_motor_flux_linkage, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_observer_gain, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_observer_gain_slow, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_duty_dowmramp_kp, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_duty_dowmramp_ki, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_openloop_rpm, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_sl_openloop_hyst, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_sl_openloop_time, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_sl_d_current_duty, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_sl_d_current_factor, &ind);
	memcpy(send_buffer + ind, mcconf->foc_hall_table, 8);
	ind += 8;
	buffer_append_float32_auto(send_buffer, mcconf->foc_sl_erpm, &ind);
	send_buffer[ind++] = mcconf->foc_sample_v0_v7;
	send_buffer[ind++] = mcconf->foc_sample_high_current;
	buffer_append_float32_auto(send_buffer, mcconf->foc_sat_comp, &ind);
	send_buffer[ind++] = mcconf->foc_temp_comp;
	buffer_append_float32_auto(send_buffer, mcconf->foc_temp_comp_base_temp, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->foc_current_filter_const, &ind);

	buffer_append_float32_auto(send_buffer, mcconf->s_pid_kp, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->s_pid_ki, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->s_pid_kd, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->s_pid_kd_filter, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->s_pid_min_erpm, &ind);
	send_buffer[ind++] = mcconf->s_pid_allow_braking;

	buffer_append_float32_auto(send_buffer, mcconf->p_pid_kp, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->p_pid_ki, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->p_pid_kd, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->p_pid_kd_filter, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->p_pid_ang_div, &ind);

	buffer_append_float32_auto(send_buffer, mcconf->cc_startup_boost_duty, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->cc_min_current, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->cc_gain, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->cc_ramp_step_max, &ind);

	buffer_append_int32(send_buffer, mcconf->m_fault_stop_time_ms, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->m_duty_ramp_step, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->m_current_backoff_gain, &ind);
	buffer_append_uint32(send_buffer, mcconf->m_encoder_counts, &ind);
	send_buffer[ind++] = mcconf->m_sensor_port_mode;
	send_buffer[ind++] = mcconf->m_invert_direction;
	send_buffer[ind++] = mcconf->m_drv8301_oc_mode;
	send_buffer[ind++] = mcconf->m_drv8301_oc_adj;
	buffer_append_float32_auto(send_buffer, mcconf->m_bldc_f_sw_min, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->m_bldc_f_sw_max, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->m_dc_f_sw, &ind);
	buffer_append_float32_auto(send_buffer, mcconf->m_ntc_motor_beta, &ind);
	send_buffer[ind++] = mcconf->m_out_aux_mode;

	send_packet_no_fwd(send_buffer, ind);
}

void bldc_interface_set_appconf(const app_configuration *appconf) {
	int32_t ind = 0;
	send_buffer[ind++] = COMM_SET_APPCONF;
	send_buffer[ind++] = appconf->controller_id;
	buffer_append_uint32(send_buffer, appconf->timeout_msec, &ind);
	buffer_append_float32_auto(send_buffer, appconf->timeout_brake_current, &ind);
	send_buffer[ind++] = appconf->send_can_status;
	buffer_append_uint16(send_buffer, appconf->send_can_status_rate_hz, &ind);
	send_buffer[ind++] = appconf->can_baud_rate;

	send_buffer[ind++] = appconf->app_to_use;

	send_buffer[ind++] = appconf->app_ppm_conf.ctrl_type;
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.pid_max_erpm, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.hyst, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.pulse_start, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.pulse_end, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.pulse_center, &ind);
	send_buffer[ind++] = appconf->app_ppm_conf.median_filter;
	send_buffer[ind++] = appconf->app_ppm_conf.safe_start;
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.throttle_exp, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.throttle_exp_brake, &ind);
	send_buffer[ind++] = appconf->app_ppm_conf.throttle_exp_mode;
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.ramp_time_pos, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.ramp_time_neg, &ind);
	send_buffer[ind++] = appconf->app_ppm_conf.multi_esc;
	send_buffer[ind++] = appconf->app_ppm_conf.tc;
	buffer_append_float32_auto(send_buffer, appconf->app_ppm_conf.tc_max_diff, &ind);

	send_buffer[ind++] = appconf->app_adc_conf.ctrl_type;
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.hyst, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.voltage_start, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.voltage_end, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.voltage_center, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.voltage2_start, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.voltage2_end, &ind);
	send_buffer[ind++] = appconf->app_adc_conf.use_filter;
	send_buffer[ind++] = appconf->app_adc_conf.safe_start;
	send_buffer[ind++] = appconf->app_adc_conf.cc_button_inverted;
	send_buffer[ind++] = appconf->app_adc_conf.rev_button_inverted;
	send_buffer[ind++] = appconf->app_adc_conf.voltage_inverted;
	send_buffer[ind++] = appconf->app_adc_conf.voltage2_inverted;
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.throttle_exp, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.throttle_exp_brake, &ind);
	send_buffer[ind++] = appconf->app_adc_conf.throttle_exp_mode;
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.ramp_time_pos, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.ramp_time_neg, &ind);
	send_buffer[ind++] = appconf->app_adc_conf.multi_esc;
	send_buffer[ind++] = appconf->app_adc_conf.tc;
	buffer_append_float32_auto(send_buffer, appconf->app_adc_conf.tc_max_diff, &ind);
	buffer_append_uint16(send_buffer, appconf->app_adc_conf.update_rate_hz, &ind);

	buffer_append_uint32(send_buffer, appconf->app_uart_baudrate, &ind);

	send_buffer[ind++] = appconf->app_chuk_conf.ctrl_type;
	buffer_append_float32_auto(send_buffer, appconf->app_chuk_conf.hyst, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_chuk_conf.ramp_time_pos, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_chuk_conf.ramp_time_neg, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_chuk_conf.stick_erpm_per_s_in_cc, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_chuk_conf.throttle_exp, &ind);
	buffer_append_float32_auto(send_buffer, appconf->app_chuk_conf.throttle_exp_brake, &ind);
	send_buffer[ind++] = appconf->app_chuk_conf.throttle_exp_mode;
	send_buffer[ind++] = appconf->app_chuk_conf.multi_esc;
	send_buffer[ind++] = appconf->app_chuk_conf.tc;
	buffer_append_float32_auto(send_buffer, appconf->app_chuk_conf.tc_max_diff, &ind);

	send_buffer[ind++] = appconf->app_nrf_conf.speed;
	send_buffer[ind++] = appconf->app_nrf_conf.power;
	send_buffer[ind++] = appconf->app_nrf_conf.crc_type;
	send_buffer[ind++] = appconf->app_nrf_conf.retry_delay;
	send_buffer[ind++] = appconf->app_nrf_conf.retries;
	send_buffer[ind++] = appconf->app_nrf_conf.channel;
	memcpy(send_buffer + ind, appconf->app_nrf_conf.address, 3);
	ind += 3;
	send_buffer[ind++] = appconf->app_nrf_conf.send_crc_ack;

	send_packet_no_fwd(send_buffer, ind);
}

// Getters
void bldc_interface_get_fw_version(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_FW_VERSION;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_get_values(void) {
	if (values_requested_func) {
		values_requested_func();
		return;
	}
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_GET_VALUES;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_get_mcconf(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_GET_MCCONF;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_get_appconf(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_GET_APPCONF;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_get_decoded_ppm(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_GET_DECODED_PPM;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_get_decoded_adc(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_GET_DECODED_ADC;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_get_decoded_chuk(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_GET_DECODED_CHUK;
	send_packet_no_fwd(send_buffer, send_index);
}

// Other functions
void bldc_interface_detect_motor_param(float current, float min_rpm, float low_duty) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_DETECT_MOTOR_PARAM;
	buffer_append_float32(send_buffer, current, 1000.0, &send_index);
	buffer_append_float32(send_buffer, min_rpm, 1000.0, &send_index);
	buffer_append_float32(send_buffer, low_duty, 1000.0, &send_index);
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_reboot(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_REBOOT;
	send_packet_no_fwd(send_buffer, send_index);
}

void bldc_interface_send_alive(void) {
	int32_t send_index = 0;
	send_buffer[send_index++] = COMM_ALIVE;
	send_packet_no_fwd(send_buffer, send_index);
}

void send_values_to_receiver(mc_values *values) {
	if (rx_value_func) {
		rx_value_func(values);
	}
}

// Helpers
const char* bldc_interface_fault_to_string(mc_fault_code fault) {
	switch (fault) {
	case FAULT_CODE_NONE: return "FAULT_CODE_NONE";
	case FAULT_CODE_OVER_VOLTAGE: return "FAULT_CODE_OVER_VOLTAGE";
	case FAULT_CODE_UNDER_VOLTAGE: return "FAULT_CODE_UNDER_VOLTAGE";
	case FAULT_CODE_DRV: return "FAULT_CODE_DRV";
	case FAULT_CODE_ABS_OVER_CURRENT: return "FAULT_CODE_ABS_OVER_CURRENT";
	case FAULT_CODE_OVER_TEMP_FET: return "FAULT_CODE_OVER_TEMP_FET";
	case FAULT_CODE_OVER_TEMP_MOTOR: return "FAULT_CODE_OVER_TEMP_MOTOR";
	default: return "Unknown fault";
	}
}

// Private functions
void send_packet_no_fwd(unsigned char *data, unsigned int len) {
	if (!forward_func) {
		bldc_interface_send_packet(data, len);
	}
}
