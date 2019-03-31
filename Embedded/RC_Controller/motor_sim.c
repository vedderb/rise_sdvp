/*
	Copyright 2018 Benjamin Vedder	benjamin@vedder.se

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

#include <math.h>

#include "motor_sim.h"
#include "ch.h"
#include "hal.h"
#include "bldc_interface.h"
#include "utils.h"

// Settings
#define MOTOR_NUM						2
#define SIMULATION_TIME_MS				10
#define MOTOR_KV						520.0
#define MOTOR_POLES						4.0
#define INPUT_VOLTAGE					39.0
#define ERPM_PER_SEC					25000.0
#define MAX_CURRENT						80.0
#define TIMEOUT							2.0

// Private types
typedef struct {
	mc_values values;
	motor_control_mode mode;
	float mode_value;
	float timeout;
	float tacho_f;
	float tacho_abs_f;
} motor_state;

// Private variables
static bool m_is_running;
static int m_motor_now;
static motor_state m_motors[MOTOR_NUM];

// Private functions
static void motor_control_set(motor_control_mode mode, float value);
static void motor_values_requested(void);

// Threads
static THD_WORKING_AREA(sim_thread_wa, 2048);
static THD_FUNCTION(sim_thread, arg);

void motor_sim_init(void) {
	m_is_running = false;
	m_motor_now = 0;
	memset(m_motors, 0, sizeof(m_motors));
	chThdCreateStatic(sim_thread_wa, sizeof(sim_thread_wa), NORMALPRIO, sim_thread, NULL);
}

void motor_sim_set_running(bool running) {
	m_is_running = running;

	if (m_is_running) {
		bldc_interface_set_sim_control_function(motor_control_set);
		bldc_interface_set_sim_values_func(motor_values_requested);
	} else  {
		bldc_interface_set_sim_control_function(0);
		bldc_interface_set_sim_values_func(0);
	}
}

void motor_sim_set_motor(int motor) {
	if (motor >= 0 && motor < MOTOR_NUM) {
		m_motor_now = motor;
	}
}

static THD_FUNCTION(sim_thread, arg) {
	(void)arg;

	chRegSetThreadName("MotorSim");

	systime_t iteration_timer = chVTGetSystemTime();

	for(;;) {
		float dt = (float)SIMULATION_TIME_MS / 1000.0;

		if (m_is_running) {
			for (int i = 0;i < MOTOR_NUM;i++) {
				const float rpm_max = INPUT_VOLTAGE * MOTOR_KV * (MOTOR_POLES / 2.0);

				switch (m_motors[i].mode) {
				case MOTOR_CONTROL_DUTY: {
					float rpm = m_motors[i].mode_value * rpm_max;
					utils_step_towards(&m_motors[i].values.rpm, rpm, ERPM_PER_SEC * dt);
				} break;

				case MOTOR_CONTROL_CURRENT: {
					utils_step_towards(&m_motors[i].values.rpm, SIGN(m_motors[i].mode_value) * rpm_max,
							ERPM_PER_SEC * dt * (fabsf(m_motors[i].mode_value) / MAX_CURRENT));
				} break;

				case MOTOR_CONTROL_CURRENT_BRAKE: {
					utils_step_towards(&m_motors[i].values.rpm, 0.0,
							ERPM_PER_SEC * dt * (fabsf(m_motors[i].mode_value) / MAX_CURRENT));
				} break;

				case MOTOR_CONTROL_RPM: {
					utils_step_towards(&m_motors[i].values.rpm, m_motors[i].mode_value, ERPM_PER_SEC * dt);
				} break;

				case MOTOR_CONTROL_POS: {
					// TODO
				} break;

				default:
					break;
				}

				// Friction
				if (m_motors[i].mode != MOTOR_CONTROL_RPM) {
					m_motors[i].values.rpm *= powf(0.9, dt);
					utils_step_towards(&m_motors[i].values.rpm, 0.0, ERPM_PER_SEC * dt * 0.02);
				}

				// Update values
				m_motors[i].tacho_f += m_motors[i].values.rpm / 60.0 * dt * 6.0;
				m_motors[i].tacho_abs_f += fabsf(m_motors[i].values.rpm) / 60.0 * dt * 6.0;
				m_motors[i].values.tachometer = m_motors[i].tacho_f;
				m_motors[i].values.tachometer_abs = m_motors[i].tacho_abs_f;
				m_motors[i].values.v_in = INPUT_VOLTAGE;
				m_motors[i].values.duty_now = m_motors[i].values.rpm / rpm_max;
				m_motors[i].values.temp_mos = 25.0;
				m_motors[i].values.temp_motor = 25.0;
				m_motors[i].values.current_motor = 0.0;
				m_motors[i].values.current_in = m_motors[i].values.duty_now * m_motors[i].values.current_motor;
				m_motors[i].values.id = 0.0;
				m_motors[i].values.iq = m_motors[i].values.current_motor;
				m_motors[i].values.amp_hours = 0.0;
				m_motors[i].values.amp_hours_charged = 0.0;
				m_motors[i].values.watt_hours = 0.0;
				m_motors[i].values.watt_hours_charged = 0.0;
				m_motors[i].values.fault_code = FAULT_CODE_NONE;

				// Timeout
				m_motors[i].timeout += dt;
				if (m_motors[i].timeout > TIMEOUT) {
					m_motors[i].mode = MOTOR_CONTROL_CURRENT_BRAKE;
					m_motors[i].mode_value = 10.0;
				}
			}
		}

		iteration_timer = chThdSleepUntilWindowed(iteration_timer,
				iteration_timer + MS2ST(SIMULATION_TIME_MS));
	}
}

static void motor_control_set(motor_control_mode mode, float value) {
	m_motors[m_motor_now].mode = mode;
	m_motors[m_motor_now].mode_value = value;
	m_motors[m_motor_now].timeout = 0.0;
}

static void motor_values_requested(void) {
	send_values_to_receiver(&(m_motors[m_motor_now].values));
}
