/*
 * radar.h
 *
 *  Created on: 24 okt. 2016
 *      Author: benjamin
 */

#ifndef RADAR_H_
#define RADAR_H_

#include "conf_general.h"

#if RADAR_EN

// Functions
void radar_init(void);
void radar_setup_measurement(radar_settings_t *settings);
void radar_sample(void);
void radar_setup_measurement_default(void);
void radar_cmd(char *cmd);

#endif

#endif /* RADAR_H_ */
