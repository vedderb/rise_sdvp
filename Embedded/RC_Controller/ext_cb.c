/*
	Copyright 2013-2016 Benjamin Vedder	benjamin@vedder.se

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

#include "ext_cb.h"
#include "ch.h"
#include "hal.h"
#include "hal_rf.h"
#include "cc1120.h"
#include "pos.h"

static const EXTConfig extcfg = {
		{
				{EXT_CH_MODE_DISABLED, NULL}, // 0
				{EXT_CH_MODE_DISABLED, NULL}, // 1
				{EXT_CH_MODE_DISABLED, NULL}, // 2
				{EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOA, cc1120_ext_cb}, // 3
				{EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOD, pos_pps_cb}, // 4
				{EXT_CH_MODE_DISABLED, NULL}, // 5
				{EXT_CH_MODE_DISABLED, NULL}, // 6
				{EXT_CH_MODE_DISABLED, NULL}, // 7
				{EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOC, pos_pps_cb}, // 8
				{EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOD, halRfExtCb}, // 9
				{EXT_CH_MODE_DISABLED, NULL}, // 10
				{EXT_CH_MODE_DISABLED, NULL}, // 11
				{EXT_CH_MODE_DISABLED, NULL}, // 12
				{EXT_CH_MODE_DISABLED, NULL}, // 13
				{EXT_CH_MODE_DISABLED, NULL}, // 14
				{EXT_CH_MODE_DISABLED, NULL}, // 15
				{EXT_CH_MODE_DISABLED, NULL}, // 16
				{EXT_CH_MODE_DISABLED, NULL}, // 17
				{EXT_CH_MODE_DISABLED, NULL}, // 18
				{EXT_CH_MODE_DISABLED, NULL}, // 19
				{EXT_CH_MODE_DISABLED, NULL}, // 20
				{EXT_CH_MODE_DISABLED, NULL}, // 21
				{EXT_CH_MODE_DISABLED, NULL} // 22
		}
};

void ext_cb_init(void) {
	extStart(&EXTD1, &extcfg);
}
