
#include <math.h>

#include "motor_sim.h"
#include "ch.h"
#include "hal.h"
#include "bldc_interface.h"

// Settings
#define SIMULATION_TIME_MS				10

// Private variables
static bool m_is_running;

// Private functions
static void motor_control_set(motor_control_mode mode, float value);
static void motor_values_requested(void);

// Threads
static THD_WORKING_AREA(sim_thread_wa, 2048);
static THD_FUNCTION(sim_thread, arg);

void motor_sim_init(void) {
	m_is_running = false;
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
		if (m_is_running) {
			// TODO!
		}

		iteration_timer = chThdSleepUntilWindowed(iteration_timer,
				iteration_timer + MS2ST(SIMULATION_TIME_MS));
	}
}

static void motor_control_set(motor_control_mode mode, float value) {
	(void)mode;
	(void)value;
}

static void motor_values_requested(void) {

}
