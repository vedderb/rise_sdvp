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

#include "mr_control.h"
#include "pos.h"
#include "utils.h"
#include "actuator.h"

#include <math.h>

// Settings
#define INPUT_TIMEOUT_MS				1000
#define POWER_OVERRIDE_TIMEOUT_MS		500
#define AUTOPILOT_TIMEOUT_MS			1000
#define MIN_THROTTLE					0.1
#define THROTTLE_OVERRIDE_LIM			0.85

// Private types
typedef struct {
	float roll_goal;
	float pitch_goal;
	float yaw_goal;
	float roll_integrator;
	float pitch_integrator;
	float yaw_integrator;
	float last_roll_process;
	float last_roll_error;
	float last_pitch_process;
	float last_pitch_error;
	float last_yaw_process;
	float last_yaw_error;
} MR_CONTROL_STATE;

typedef struct {
	float roll;
	float pitch;
	float yaw;
	float throttle;
	systime_t last_update_time;
} MR_RC_STATE;

typedef struct {
	float throttle;
	float roll;
	float pitch;
	float yaw;
} MR_OUTPUT;

// Private variables
static MR_CONTROL_STATE m_ctrl;
static MR_RC_STATE m_rc;
static POS_STATE m_pos_last;
static MR_OUTPUT m_output;
static float m_power_override[4];
static float m_power_override_time;

// Private functions
static void update_rc_control(MR_CONTROL_STATE *ctrl, MR_RC_STATE *rc, POS_STATE *pos, float dt);

void mr_control_init(void) {
	memset(&m_rc, 0, sizeof(MR_RC_STATE));
	memset(&m_ctrl, 0, sizeof(MR_CONTROL_STATE));
	memset(&m_pos_last, 0, sizeof(POS_STATE));
	memset(&m_output, 0, sizeof(MR_OUTPUT));
	memset(m_power_override, 0, sizeof(m_power_override));
	m_power_override_time = 0.0;
}

systime_t mr_control_time_since_input_update(void) {
	return ST2MS(chVTTimeElapsedSinceX(m_rc.last_update_time));
}

void mr_control_set_input(float throttle, float roll, float pitch, float yaw) {
	m_rc.roll = roll;
	m_rc.pitch = pitch;
	m_rc.yaw = yaw;
	m_rc.throttle = throttle;
	m_rc.last_update_time = chVTGetSystemTime();
}

void mr_control_set_motor_override(int motor, float power) {
	if (motor >= 0 && motor < 4) {
		m_power_override[motor] = power;
		m_power_override_time = (float)POWER_OVERRIDE_TIMEOUT_MS / 1000.0;
	}
}

bool mr_control_is_throttle_over_tres(void) {
	return m_output.throttle > MIN_THROTTLE;
}

void mr_control_run_iteration(float dt) {
	bool lost_signal = true;

	pos_get_pos(&m_pos_last);

	if (mr_control_time_since_input_update() < INPUT_TIMEOUT_MS) {
		lost_signal = false;
	} else {
		memset(&m_rc, 0, sizeof(MR_RC_STATE));
	}

	if (m_power_override_time > 0.0) {
		m_power_override_time -= dt;

		for (int i = 0;i < 4;i++) {
			actuator_set_motor(i, m_power_override[i]);
		}

		if (m_power_override_time <= 0.0) {
			memset(m_power_override, 0, sizeof(m_power_override));
		}

		return;
	}

	memset(&m_output, 0, sizeof(MR_OUTPUT));

	if (!lost_signal) {
		if (m_rc.throttle < THROTTLE_OVERRIDE_LIM) {
			// Manual control
			update_rc_control(&m_ctrl, &m_rc, &m_pos_last, dt);
		} else {
			// TODO: autopilot
		}

		// Run attitude control
		float roll_error = utils_angle_difference(m_ctrl.roll_goal, m_pos_last.roll);
		float pitch_error = utils_angle_difference(m_ctrl.pitch_goal, m_pos_last.pitch);
		float yaw_error = utils_angle_difference(m_ctrl.yaw_goal, m_pos_last.yaw);

		roll_error /= 360.0;
		pitch_error /= 360.0;
		yaw_error /= 360.0;

		// Run integration
		m_ctrl.roll_integrator += roll_error * dt;
		m_ctrl.pitch_integrator += pitch_error * dt;
		m_ctrl.yaw_integrator += yaw_error * dt;
		// Prevent wind-up
		utils_truncate_number_abs(&m_ctrl.roll_integrator, 1.0 / main_config.mr.ctrl_gain_roll_i);
		utils_truncate_number_abs(&m_ctrl.pitch_integrator, 1.0 / main_config.mr.ctrl_gain_pitch_i);
		utils_truncate_number_abs(&m_ctrl.yaw_integrator, 1.0 / main_config.mr.ctrl_gain_yaw_i);

		float d_roll_sample_process = -utils_angle_difference(m_pos_last.roll, m_ctrl.last_roll_process) / dt;
		float d_roll_sample_error = (roll_error - m_ctrl.last_roll_error) / dt;
		float d_pitch_sample_process = -utils_angle_difference(m_pos_last.pitch, m_ctrl.last_pitch_process) / dt;
		float d_pitch_sample_error = (pitch_error - m_ctrl.last_pitch_error) / dt;
		float d_yaw_sample_process = -utils_angle_difference(m_pos_last.yaw, m_ctrl.last_yaw_process) / dt;
		float d_yaw_sample_error = (yaw_error - m_ctrl.last_yaw_error) / dt;

		m_ctrl.last_roll_process = m_pos_last.roll;
		m_ctrl.last_roll_error = roll_error;
		m_ctrl.last_pitch_process = m_pos_last.pitch;
		m_ctrl.last_pitch_error = pitch_error;
		m_ctrl.last_yaw_process = m_pos_last.yaw;
		m_ctrl.last_yaw_error = yaw_error;

		d_roll_sample_process /= 360.0;
		d_pitch_sample_process /= 360.0;
		d_yaw_sample_process /= 360.0;

		m_output.roll = roll_error * main_config.mr.ctrl_gain_roll_p +
				m_ctrl.roll_integrator * main_config.mr.ctrl_gain_roll_i +
				d_roll_sample_process * main_config.mr.ctrl_gain_roll_dp +
				d_roll_sample_error * main_config.mr.ctrl_gain_roll_de;
		m_output.pitch = pitch_error * main_config.mr.ctrl_gain_pitch_p +
				m_ctrl.pitch_integrator * main_config.mr.ctrl_gain_pitch_i +
				d_pitch_sample_process * main_config.mr.ctrl_gain_pitch_dp +
				d_pitch_sample_error * main_config.mr.ctrl_gain_pitch_de;
		m_output.yaw = yaw_error * main_config.mr.ctrl_gain_yaw_p +
				m_ctrl.yaw_integrator * main_config.mr.ctrl_gain_yaw_i +
				d_yaw_sample_process * main_config.mr.ctrl_gain_yaw_dp +
				d_yaw_sample_error * main_config.mr.ctrl_gain_yaw_de;

		utils_truncate_number_abs(&m_output.roll, 1.0);
		utils_truncate_number_abs(&m_output.pitch, 1.0);
		utils_truncate_number_abs(&m_output.yaw, 1.0);

		// Compensate throttle for roll and pitch
		const float tan_roll = tanf(m_pos_last.roll * M_PI / 180.0);
		const float tan_pitch = tanf(m_pos_last.pitch * M_PI / 180.0);
		const float tilt_comp_factor = sqrtf(tan_roll * tan_roll + tan_pitch * tan_pitch + 1);

		m_output.throttle *= tilt_comp_factor;
		utils_truncate_number(&m_output.throttle, 0.0, 1.0);
	}

	actuator_set_output(m_output.throttle, m_output.roll, m_output.pitch, m_output.yaw);
}

static void update_rc_control(MR_CONTROL_STATE *ctrl, MR_RC_STATE *rc, POS_STATE *pos, float dt) {
	if (fabsf(rc->yaw) < 0.05) {
		rc->yaw = 0.0;
	}

	float roll = ctrl->roll_goal + rc->roll * main_config.mr.js_gain_tilt * dt * 250.0;
	float pitch = ctrl->pitch_goal + rc->pitch * main_config.mr.js_gain_tilt * dt * 250.0;
	float yaw = ctrl->yaw_goal + rc->yaw * main_config.mr.js_gain_yaw * dt * 250.0;

	if (main_config.mr.js_mode_rate) {
		// Integrate towards the current angle slowly
		roll = utils_weight_angle(roll, pos->roll, 0.995);
		pitch = utils_weight_angle(pitch, pos->pitch, 0.995);
	} else {
		// A trick!
		roll = utils_weight_angle(roll, -pos->roll, 0.995);
		pitch = utils_weight_angle(pitch, -pos->pitch, 0.995);
	}

	utils_norm_angle(&roll);
	utils_norm_angle(&pitch);
	utils_norm_angle(&yaw);

	ctrl->roll_goal = roll;
	ctrl->pitch_goal = pitch;
	ctrl->yaw_goal = yaw;
}
