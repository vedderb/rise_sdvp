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

#include "actuator.h"
#include "ch.h"
#include "hal.h"
#include "pwm_esc.h"
#include "utils.h"
#include "conf_general.h"
#include "adconv.h"

void actuator_set_output(float throttle, float roll, float pitch, float yaw) {
	float motors[4];

	if (main_config.mr.motors_cw) {
		if (main_config.mr.motors_x) {
			motors[main_config.mr.motor_fl_f] = throttle + roll + pitch - yaw;
			motors[main_config.mr.motor_bl_l] = throttle + roll - pitch + yaw;
			motors[main_config.mr.motor_fr_r] = throttle - roll + pitch + yaw;
			motors[main_config.mr.motor_br_b] = throttle - roll - pitch - yaw;
		} else {
			motors[main_config.mr.motor_fl_f] = throttle + pitch - yaw;
			motors[main_config.mr.motor_bl_l] = throttle + roll + yaw;
			motors[main_config.mr.motor_fr_r] = throttle - roll + yaw;
			motors[main_config.mr.motor_br_b] = throttle - pitch - yaw;
		}
	} else {
		if (main_config.mr.motors_x) {
			motors[main_config.mr.motor_fl_f] = throttle + roll + pitch + yaw;
			motors[main_config.mr.motor_bl_l] = throttle + roll - pitch - yaw;
			motors[main_config.mr.motor_fr_r] = throttle - roll + pitch - yaw;
			motors[main_config.mr.motor_br_b] = throttle - roll - pitch + yaw;
		} else {
			motors[main_config.mr.motor_fl_f] = throttle + pitch + yaw;
			motors[main_config.mr.motor_bl_l] = throttle + roll - yaw;
			motors[main_config.mr.motor_fr_r] = throttle - roll - yaw;
			motors[main_config.mr.motor_br_b] = throttle - pitch + yaw;
		}
	}

	for (int i = 0;i < 4;i++) {
		// Charge-based throttle scaling
		const float v_in = adconv_get_vin();
		if (v_in > 6.0 && v_in < 20.0) {
			motors[i] *= (10.5 / v_in);
		}

		utils_truncate_number(&motors[i], 0.0, 1.0);
		pwm_esc_set(i, motors[i]);
	}
}

/**
 * Set single motor output.
 *
 * @param motor
 * Motor to set.
 * 0: x: Front Left  +: Front
 * 1: x: Back Left   +: Left
 * 2: x: Front Right +: Right
 * 3: x: Back Right  +: Back
 * -1: All motors
 *
 * @param throttle
 * The throttle value, range [0.0 1.0]
 */
void actuator_set_motor(int motor, float throttle) {
	utils_truncate_number(&throttle, 0.0, 1.0);

	switch (motor) {
	case 0: pwm_esc_set(main_config.mr.motor_fl_f, throttle); break;
	case 1: pwm_esc_set(main_config.mr.motor_bl_l, throttle); break;
	case 2: pwm_esc_set(main_config.mr.motor_fr_r, throttle); break;
	case 3: pwm_esc_set(main_config.mr.motor_br_b, throttle); break;
	case -1: pwm_esc_set_all(throttle); break;
	default: break;
	}
}
