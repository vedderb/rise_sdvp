/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#include "ch.h"
#include "hal.h"
#include "terminal.h"
#include "commands.h"
#include "utils.h"
#include "bldc_interface.h"
#include "radar.h"
#include "cc1120.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

void terminal_process_string(char *str) {
	enum { kMaxArgs = 64 };
	int argc = 0;
	char *argv[kMaxArgs];

	char *p2 = strtok(str, " ");
	while (p2 && argc < kMaxArgs) {
		argv[argc++] = p2;
		p2 = strtok(0, " ");
	}

	static char buffer[256];

	if (argc == 0) {
		commands_printf("No command received\n");
		return;
	}

	if (strcmp(argv[0], "ping") == 0) {
		commands_printf("pong\n");
	} else if (strcmp(argv[0], "mem") == 0) {
		size_t n, size;
		n = chHeapStatus(NULL, &size);
		commands_printf("core free memory : %u bytes", chCoreGetStatusX());
		commands_printf("heap fragments   : %u", n);
		commands_printf("heap free total  : %u bytes\n", size);
	} else if (strcmp(argv[0], "threads") == 0) {
		thread_t *tp;
		static const char *states[] = {CH_STATE_NAMES};
		commands_printf("    addr    stack prio refs     state           name time    ");
		commands_printf("-------------------------------------------------------------");
		tp = chRegFirstThread();
		do {
			commands_printf("%.8lx %.8lx %4lu %4lu %9s %14s %lu",
					(uint32_t)tp, (uint32_t)tp->p_ctx.r13,
					(uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
					states[tp->p_state], tp->p_name, (uint32_t)tp->p_time);
			tp = chRegNextThread(tp);
		} while (tp != NULL);
		commands_printf(" ");
	} else if (strcmp(argv[0], "vesc") == 0) {
		buffer[0] = '\0';
		int ind = 0;
		for (int i = 1;i < argc;i++) {
			sprintf(buffer + ind, " %s", argv[i]);
			ind += strlen(argv[i]) + 1;
		}
		bldc_interface_terminal_cmd(buffer);
	} else if (strcmp(argv[0], "cc1120_state") == 0) {
		commands_printf("%s\n", cc1120_state_name());
	}

#if RADAR_EN
	else if (strcmp(argv[0], "radar_sample") == 0) {
		commands_printf("Sampling radar...");
		radar_setup_measurement_default();
		radar_sample();
	} else if (strcmp(argv[0], "radar_cmd") == 0) {
		buffer[0] = '\0';
		int ind = 0;
		for (int i = 1;i < argc;i++) {
			if (i == 1) {
				sprintf(buffer + ind, "%s", argv[i]);
				ind += strlen(argv[i]);
			} else {
				sprintf(buffer + ind, " %s", argv[i]);
				ind += strlen(argv[i]) + 1;
			}
		}
		radar_cmd(buffer);
	}
#endif

	// The help command
	else if (strcmp(argv[0], "help") == 0) {
		commands_printf("Valid commands are:");
		commands_printf("help");
		commands_printf("  Show this help");

		commands_printf("ping");
		commands_printf("  Print pong here to see if the reply works");

		commands_printf("mem");
		commands_printf("  Show memory usage");

		commands_printf("threads");
		commands_printf("  List all threads");

		commands_printf("vesc");
		commands_printf("  Forward command to VESC");

		commands_printf("cc1120_state");
		commands_printf("  Print the state of the CC1120");

#if RADAR_EN
		commands_printf("radar_sample");
		commands_printf("  Start radar sampling");

		commands_printf("radar_cmd");
		commands_printf("  Forward command to radar");
#endif

		commands_printf("");
	} else {
		commands_printf("Invalid command: %s\n"
				"type help to list all available commands\n", argv[0]);
	}
}
