/*
	Copyright 2016-2017 Benjamin Vedder	benjamin@vedder.se

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

#ifndef BLDC_INTERFACE_H_
#define BLDC_INTERFACE_H_

#include "datatypes.h"

// interface functions
void bldc_interface_init(void(*func)(unsigned char *data, unsigned int len));
void bldc_interface_set_forward_func(void(*func)(unsigned char *data, unsigned int len));
void bldc_interface_send_packet(unsigned char *data, unsigned int len);
void bldc_interface_process_packet(unsigned char *data, unsigned int len);

// Function pointer setters
void bldc_interface_set_rx_value_func(void(*func)(mc_values *values));
void bldc_interface_set_rx_printf_func(void(*func)(char *str));
void bldc_interface_set_rx_fw_func(void(*func)(int major, int minor));
void bldc_interface_set_rx_rotor_pos_func(void(*func)(float pos));
void bldc_interface_set_rx_mcconf_func(void(*func)(mc_configuration *conf));
void bldc_interface_set_rx_appconf_func(void(*func)(app_configuration *conf));
void bldc_interface_set_rx_detect_func(void(*func)(float cycle_int_limit, float coupling_k,
		const signed char *hall_table, signed char hall_res));
void bldc_interface_set_rx_dec_ppm_func(void(*func)(float val, float ms));
void bldc_interface_set_rx_dec_adc_func(void(*func)(float val, float voltage));
void bldc_interface_set_rx_dec_chuk_func(void(*func)(float val));
void bldc_interface_set_rx_mcconf_received_func(void(*func)(void));
void bldc_interface_set_rx_appconf_received_func(void(*func)(void));

void bldc_interface_set_sim_control_function(void(*func)(motor_control_mode mode, float value));
void bldc_interface_set_sim_values_func(void(*func)(void));

// Setters
void bldc_interface_terminal_cmd(char* cmd);
void bldc_interface_set_duty_cycle(float dutyCycle);
void bldc_interface_set_current(float current);
void bldc_interface_set_current_brake(float current);
void bldc_interface_set_rpm(int rpm);
void bldc_interface_set_pos(float pos);
void bldc_interface_set_handbrake(float current);
void bldc_interface_set_servo_pos(float pos);
void bldc_interface_set_mcconf(const mc_configuration *mcconf);
void bldc_interface_set_appconf(const app_configuration *appconf);

// Getters
void bldc_interface_get_fw_version(void);
void bldc_interface_get_values(void);
void bldc_interface_get_mcconf(void);
void bldc_interface_get_appconf(void);
void bldc_interface_get_decoded_ppm(void);
void bldc_interface_get_decoded_adc(void);
void bldc_interface_get_decoded_chuk(void);

// Other functions
void bldc_interface_detect_motor_param(float current, float min_rpm, float low_duty);
void bldc_interface_reboot(void);
void bldc_interface_send_alive(void);
void send_values_to_receiver(mc_values *values);

// Helpers
const char* bldc_interface_fault_to_string(mc_fault_code fault);

#endif /* BLDC_INTERFACE_H_ */
