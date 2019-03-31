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

#include "terminal.h"
#include "commands.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

// Settings
#define PROBE_NUM			5
#define FAULTS_PER_PROBE	5
#define FAULT_ID_MAX_LEN	20

// Private types
typedef enum {
	FAULT_TYPE_NONE = 0,
	FAULT_TYPE_BITFLIP,
	FAULT_TYPE_OFFSET,
	FAULT_TYPE_AMPLIFICATION,
	FAULT_TYPE_SET_TO
} FAULT_TYPE_t;

typedef struct {
	FAULT_TYPE_t type;
	float param;
	bool active;
	int start_it;
	int duration;
} fault_info_t;

typedef struct {
	char id[FAULT_ID_MAX_LEN + 1];
	bool active;
	fault_info_t faults[FAULTS_PER_PROBE];
	int iteration;
} probe_t;

// Private variables
static bool m_active = false;
static probe_t m_probes[PROBE_NUM];

// Private functions
static void cmd_terminal_set_enabled(int argc, const char **argv);
static void cmd_terminal_add_fault(int argc, const char **argv);
static void cmd_terminal_clear_faults(int argc, const char **argv);
static void cmd_terminal_print_faults(int argc, const char **argv);
static void cmd_terminal_reset_cnt(int argc, const char **argv);

void fi_init(void) {
	m_active = false;
	memset(m_probes, 0, sizeof(m_probes));

	terminal_register_command_callback(
			"fi_set_enabled",
			"Enable or disable fault injection.\n"
			"  0 - Disabled\n"
			"  1 - Enabled",
			"[enabled]",
			cmd_terminal_set_enabled);

	terminal_register_command_callback(
			"fi_add_fault",
			"Add fault to probe",
			"[probe_id] [type] [param] [start_it] [duration]",
			cmd_terminal_add_fault);

	terminal_register_command_callback(
			"fi_clear_faults",
			"Clear all faults",
			0,
			cmd_terminal_clear_faults);

	terminal_register_command_callback(
			"fi_print_faults",
			"Print all active faults",
			0,
			cmd_terminal_print_faults);

	terminal_register_command_callback(
			"fi_reset_cnt",
			"Reset all probe iteration counters",
			0,
			cmd_terminal_reset_cnt);
}

bool fi_is_active(void) {
	return m_active;
}

void fi_inject_fault_float(const char* id, float *value) {
	if (!m_active) {
		return;
	}

	for (int i = 0;i < PROBE_NUM;i++) {
		probe_t *p = &m_probes[i];

		if (p->active && strcmp(p->id, id) == 0) {
			for (int j = 0;j < FAULTS_PER_PROBE;j++) {
				fault_info_t *f = &(p->faults[j]);

				if (f->active && p->iteration >= f->start_it &&
						(f->duration == 0 || f->duration > (p->iteration - f->start_it))) {
					switch (f->type) {
					case FAULT_TYPE_BITFLIP: {
						union {
							float f;
							uint32_t i;
						} conv;

						conv.f = *value;

						int i = (int)f->param;
						if (i < 0) {
							i = 0;
						} else if (i > 31) {
							i = 31;
						}
						conv.i ^= 1 << i;
						*value = conv.f;
					} break;

					case FAULT_TYPE_OFFSET:
						*value += f->param;
						break;

					case FAULT_TYPE_AMPLIFICATION:
						*value *= f->param;
						break;

					case FAULT_TYPE_SET_TO:
						*value = f->param;
						break;

					default:
						break;
					}
				}
			}

			p->iteration++;
		}
	}
}

void fi_inject_fault_int(const char* id, int *value) {
	if (!m_active) {
		return;
	}

	for (int i = 0;i < PROBE_NUM;i++) {
		probe_t *p = &m_probes[i];

		if (p->active && strcmp(p->id, id) == 0) {
			for (int j = 0;j < FAULTS_PER_PROBE;j++) {
				fault_info_t *f = &(p->faults[j]);

				if (f->active && p->iteration >= f->start_it &&
						(f->duration == 0 || f->duration > (p->iteration - f->start_it))) {
					switch (f->type) {
					case FAULT_TYPE_BITFLIP: {
						int i = (int)f->param;
						if (i < 0) {
							i = 0;
						} else if (i > 31) {
							i = 31;
						}
						*value ^= 1 << i;
					} break;

					case FAULT_TYPE_OFFSET:
						*value += f->param;
						break;

					case FAULT_TYPE_AMPLIFICATION:
						*value *= f->param;
						break;

					case FAULT_TYPE_SET_TO:
						*value = f->param;
						break;

					default:
						break;
					}
				}
			}

			p->iteration++;
		}
	}
}

static void cmd_terminal_set_enabled(int argc, const char **argv) {
	if (argc == 2) {
		if (strcmp(argv[1], "0") == 0) {
			m_active = 0;
			commands_printf("Fault Injection disabled\n");
		} else if (strcmp(argv[1], "1") == 0) {
			m_active = 1;
			commands_printf("Fault Injection enabled\n");
		} else {
			commands_printf("Invalid argument %s\n", argv[1]);
		}
	} else {
		commands_printf("Wrong number of arguments\n");
	}
}

static void cmd_terminal_add_fault(int argc, const char **argv) {
	// [probe_id] [type] [param] [start_it] [duration]

	if (argc == 6) {
		bool ok = true;
		const char *id = argv[1];
		const char *type_str = argv[2];
		FAULT_TYPE_t type = FAULT_TYPE_NONE;
		float param = 0.0;
		int start_it = 0;
		int duration = 0;

		if (strlen(id) > FAULT_ID_MAX_LEN) {
			commands_printf("Too long ID. Maximum length is %d\n", FAULT_ID_MAX_LEN);
			ok = false;
		}

		if (strcasecmp(type_str, "NONE") == 0) {
			type = FAULT_TYPE_NONE;
		} else if (strcmp(type_str, "BITFLIP") == 0) {
			type = FAULT_TYPE_BITFLIP;
		} else if (strcmp(type_str, "OFFSET") == 0) {
			type = FAULT_TYPE_OFFSET;
		} else if (strcmp(type_str, "AMPLIFICATION") == 0) {
			type = FAULT_TYPE_AMPLIFICATION;
		} else if (strcmp(type_str, "SET_TO") == 0) {
			type = FAULT_TYPE_SET_TO;
		} else {
			commands_printf("Invalid fault type %s\n", type_str);
			ok = false;
		}

		sscanf(argv[3], "%f", &param);
		sscanf(argv[4], "%d", &start_it);
		sscanf(argv[5], "%d", &duration);

		if (ok) {
			probe_t *p = 0;
			for (int i = 0;i < PROBE_NUM;i++) {
				if (strcmp(m_probes[i].id, id) == 0) {
					p = &m_probes[i];
					p->active = true;
					break;
				}
			}

			if (!p) {
				for (int i = 0;i < PROBE_NUM;i++) {
					if (m_probes[i].active == false) {
						p = &m_probes[i];
						strcpy(p->id, id);
						p->active = true;
						break;
					}
				}
			}

			if (p) {
				fault_info_t *f = 0;
				for (int i = 0;i < FAULTS_PER_PROBE;i++) {
					if (p->faults[i].active == false) {
						f = &(p->faults[i]);
						break;
					}
				}

				if (f) {
					f->active = true;
					f->type = type;
					f->param = param;
					f->start_it = start_it + p->iteration;
					f->duration = duration;
				} else {
					commands_printf("Too many faults on this probe, fault not added.\n");
					ok = false;
				}
			} else {
				commands_printf("Too many probes in use, fault not added.\n");
				ok = false;
			}
		}

		if (ok) {
			commands_printf("Fault %s with parameter %.2f successfully added to probe %s.\n",
					type_str, (double)param, id);
		}
	} else {
		commands_printf("Wrong number of arguments\n");
	}
}

static void cmd_terminal_clear_faults(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	for (int i = 0;i < PROBE_NUM;i++) {
		m_probes[i].active = false;
		m_probes[i].iteration = 0;
		for (int j = 0;j < FAULTS_PER_PROBE;j++) {
			m_probes[i].faults[j].active = false;
		}
	}

	commands_printf("Faults cleared\n");
}

static void cmd_terminal_print_faults(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	commands_printf("Currently active faults:");

	for (int i = 0;i < PROBE_NUM;i++) {
		for (int j = 0;j < FAULTS_PER_PROBE;j++) {
			probe_t *p = &m_probes[i];
			if (p->active) {
				fault_info_t *f = &(m_probes[i].faults[j]);

				if (f->active) {
					char *type_str = "Unknown";

					switch (f->type) {
						case FAULT_TYPE_NONE: type_str = "NONE"; break;
						case FAULT_TYPE_BITFLIP: type_str = "BITFLIP"; break;
						case FAULT_TYPE_OFFSET: type_str = "OFFSET"; break;
						case FAULT_TYPE_AMPLIFICATION: type_str = "AMPLIFICATION"; break;
						case FAULT_TYPE_SET_TO: type_str = "SET_TO"; break;
						default: break;
					}

					commands_printf(
							"\n"
							"Probe   : %s\n"
							"IT now  : %d\n"
							"Type    : %s\n"
							"Param   : %.2f\n"
							"Start   : %d\n"
							"Duration: %d",
							p->id,
							p->iteration,
							type_str,
							(double)f->param,
							f->start_it,
							f->duration);
				}
			}
		}
	}

	commands_printf(" ");
}

static void cmd_terminal_reset_cnt(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	for (int i = 0;i < PROBE_NUM;i++) {
		m_probes[i].iteration = 0;
	}

	commands_printf("Probe iteration counters reset");
}
