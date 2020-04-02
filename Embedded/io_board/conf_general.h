/*
	Copyright 2020 Benjamin Vedder	benjamin@vedder.se

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

#ifndef CONF_GENERAL_H_
#define CONF_GENERAL_H_

#include <stdint.h>
#include <stdbool.h>
#include "ch.h"
#include "hal.h"
#include "datatypes.h"

// CAN Settings
#define BOARD_ID					0
#define BOARD_MASK					3
#define BOARD_ID_ALL				0x0F
#define CAN_TX_HZ					100

// ADC DIV
#define ADC_DIV_R1					10000.0
#define ADC_DIV_R2					1000.0

// LEDs
#define LINE_LED_RED				PAL_LINE(GPIOB, 12)
#define LINE_LED_GREEN				PAL_LINE(GPIOB, 13)

// Valves
#define LINE_VALVE_1				PAL_LINE(GPIOB, 5)
#define LINE_VALVE_2				PAL_LINE(GPIOB, 4)
#define LINE_VALVE_3				PAL_LINE(GPIOC, 12)
#define LINE_VALVE_4				PAL_LINE(GPIOC, 11)
#define LINE_VALVE_5				PAL_LINE(GPIOC, 10)
#define LINE_VALVE_6				PAL_LINE(GPIOA, 15)

// CAN
#define LINE_CAN_RX					PAL_LINE(GPIOB, 8)
#define LINE_CAN_TX					PAL_LINE(GPIOB, 9)
#define HW_CAN_DEV					CAND1
#define HW_CAN_AF					9

// Limit switch inputs
#define LINE_LIM_SW_1				PAL_LINE(GPIOC, 6)
#define LINE_LIM_SW_2				PAL_LINE(GPIOC, 7)
#define LINE_LIM_SW_3				PAL_LINE(GPIOC, 8)
#define LINE_LIM_SW_4				PAL_LINE(GPIOC, 9)

// AS5047
#define AS5047_MISO_GPIO			GPIOB
#define AS5047_MISO_PIN				1
#define AS5047_MOSI_GPIO			GPIOB
#define AS5047_MOSI_PIN				0
#define AS5047_SCK_GPIO				GPIOB
#define AS5047_SCK_PIN				2
#define AS5047_CS_GPIO				GPIOB
#define AS5047_CS_PIN				3

// Macros
#define LED_OFF(led)				palClearLine(led)
#define LED_ON(led)					palSetLine(led)
#define LED_TOGGLE(led)				palToggleLine(led)

#define VALVE_OFF(valve)			palClearLine(valve)
#define VALVE_ON(valve)				palSetLine(valve)

#define LIM_SW_READ(sw)				(!palReadLine(sw))

#endif /* CONF_GENERAL_H_ */
