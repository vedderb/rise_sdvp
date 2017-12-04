
#include <math.h>

#include "motor_sim.h"
#include "ch.h"
#include "hal.h"
#include "bldc_interface.h"
#include "utils.h"

// Settings
#define SIMULATION_TIME_MS				10
#define MOTOR_KV						520.0
#define MOTOR_POLES						4.0
#define INPUT_VOLTAGE					39.0
#define ERPM_PER_SEC					25000.0
#define MAX_CURRENT						80.0
#define TIMEOUT							2.0

// Private variables
static bool m_is_running;
static mc_values m_values;
static motor_control_mode m_mode;
static float m_mode_value;
static float m_timeout;

// Private functions
static void motor_control_set(motor_control_mode mode, float value);
static void motor_values_requested(void);

// Threads
static THD_WORKING_AREA(sim_thread_wa, 2048);
static THD_FUNCTION(sim_thread, arg);

void motor_sim_init(void) {
	m_is_running = false;
	m_mode = MOTOR_CONTROL_DUTY;
	m_mode_value = 0.0;
	m_timeout = 0.0;
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

static THD_FUNCTION(sim_thread, arg) {
	(void)arg;

	chRegSetThreadName("MotorSim");

	systime_t iteration_timer = chVTGetSystemTime();

	for(;;) {
		float dt = (float)SIMULATION_TIME_MS / 1000.0;

		if (m_is_running) {
			const float rpm_max = INPUT_VOLTAGE * MOTOR_KV * (MOTOR_POLES / 2.0);

			switch (m_mode) {
			case MOTOR_CONTROL_DUTY: {
				float rpm = m_mode_value * rpm_max;
				utils_step_towards(&m_values.rpm, rpm, ERPM_PER_SEC * dt);
			} break;

			case MOTOR_CONTROL_CURRENT: {
				utils_step_towards(&m_values.rpm, SIGN(m_mode_value) * rpm_max,
						ERPM_PER_SEC * dt * (fabsf(m_mode_value) / MAX_CURRENT));
			} break;

			case MOTOR_CONTROL_CURRENT_BRAKE: {
				utils_step_towards(&m_values.rpm, 0.0,
						ERPM_PER_SEC * dt * (fabsf(m_mode_value) / MAX_CURRENT));
			} break;

			case MOTOR_CONTROL_RPM: {
				utils_step_towards(&m_values.rpm, m_mode_value, ERPM_PER_SEC * dt);
			} break;

			case MOTOR_CONTROL_POS: {
				// TODO
			} break;

			default:
				break;
			}

			// Friction
			m_values.rpm *= powf(0.9, dt);
			utils_step_towards(&m_values.rpm, 0.0, ERPM_PER_SEC * dt * 0.02);

			// Update values
			m_values.tachometer += m_values.rpm / 60.0 * dt * 6.0;
			m_values.tachometer_abs += fabsf(m_values.rpm) / 60.0 * dt * 6.0;
			m_values.v_in = INPUT_VOLTAGE;
			m_values.duty_now = m_values.rpm / rpm_max;
			m_values.temp_mos = 25.0;
			m_values.temp_motor = 25.0;
			m_values.current_motor = 0.0;
			m_values.current_in = m_values.duty_now * m_values.current_motor;
			m_values.id = 0.0;
			m_values.iq = m_values.current_motor;
			m_values.amp_hours = 0.0;
			m_values.amp_hours_charged = 0.0;
			m_values.watt_hours = 0.0;
			m_values.watt_hours_charged = 0.0;
			m_values.fault_code = FAULT_CODE_NONE;

			// Timeout
			m_timeout += dt;
			if (m_timeout > TIMEOUT) {
				m_mode = MOTOR_CONTROL_CURRENT_BRAKE;
				m_mode_value = 10.0;
			}
		}

		iteration_timer = chThdSleepUntilWindowed(iteration_timer,
				iteration_timer + MS2ST(SIMULATION_TIME_MS));
	}
}

static void motor_control_set(motor_control_mode mode, float value) {
	m_mode = mode;
	m_mode_value = value;
	m_timeout = 0.0;
}

static void motor_values_requested(void) {
	send_values_to_receiver(&m_values);
}
