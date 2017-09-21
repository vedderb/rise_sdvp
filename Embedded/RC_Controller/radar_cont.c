
#include "radar_cont.h"
#include "ch.h"
#include "hal.h"
#include "utils.h"
#include "commands.h"
#include "comm_can.h"

#include <math.h>
#include <string.h>

// Defines
#define RADAR_CONF_RANGE_LENV				0,  1
#define RADAR_CONF_RANGE_LEN				15, 8
#define RADAR_CONF_OUTPUT_TYPEV				3,  1
#define RADAR_CONF_OUTPUT_TYPE				26, 2
#define RADAR_CONF_ENABLE_POWER_REDUCTIONV	2, 1
#define RADAR_CONF_ELEVATIONV				1, 1
#define RADAR_CONF_ENABLE_POWER_REDUCTION	24, 1
#define RADAR_CONF_ELEVATION				23, 8
#define RADAR_CONF_SENSOR_ID_V				4, 1
#define RADAR_CONF_SENSOR_ID				32, 3

// Private functions
static uint32_t get_uint32(const uint8_t *data, int start, int len);
static void set_uint32(uint8_t *data, int start, int len, uint32_t val);

void radar_cont_init(void) {
	uint8_t data[8];
	memset(data, 0, sizeof(data));

	// Settings
	set_uint32(data, RADAR_CONF_RANGE_LENV, 0);
	set_uint32(data, RADAR_CONF_RANGE_LEN, 200);
	set_uint32(data, RADAR_CONF_OUTPUT_TYPEV, 0);
	set_uint32(data, RADAR_CONF_OUTPUT_TYPE, 2);
	set_uint32(data, RADAR_CONF_ENABLE_POWER_REDUCTIONV, 0);
	set_uint32(data, RADAR_CONF_ELEVATIONV, 0);
	set_uint32(data, RADAR_CONF_ENABLE_POWER_REDUCTION, 0);
	set_uint32(data, RADAR_CONF_ELEVATION, 16 * 4);
	set_uint32(data, RADAR_CONF_SENSOR_ID_V, 1);
	set_uint32(data, RADAR_CONF_SENSOR_ID, 0);

	comm_can_transmit_sid(0x200, data, 8);

	(void)get_uint32;
}

void radar_cont_input(uint32_t id, uint8_t *data) {
	(void)data;

	switch (id) {
	case 0x60A: { // Object list
//		int num = get_uint32(data, 8, 8);
//		int meas = get_uint32(data, 16, 16);
//		commands_printf("OBJ: %d, MEAS: %d", num, meas);
	} break;

	case 0x600: { // Target status
//		int num_near = get_uint32(data, 8, 8);
//		int num_far = get_uint32(data, 16, 8);
//		commands_printf("Near: %d, Far: %d", num_near, num_far);
	} break;

	case 0x701: { // Target 1
//		int num = get_uint32(data, 8, 8);
//		float dist = (float)get_uint32(data, 16, 8) * 1e-1;
//		commands_printf("Num: %d, Dist: %f", num, (double)dist);
	} break;

	case 0x702: { // Target 2
//		int num = get_uint32(data, 8, 8);
//		float pdth0 = (float)get_uint32(data, 15, 7);
//		commands_printf("Num: %d, pdth0: %f", pdth0, (double)pdth0);
	} break;

	default:
		break;
	}
}

static uint32_t get_uint32(const uint8_t *data, int start, int len) {
	uint64_t all =
			((uint64_t) data[7]) << 56 |
			((uint64_t) data[6]) << 48 |
			((uint64_t) data[5]) << 40 |
			((uint64_t) data[4]) << 32 |
			((uint64_t) data[3]) << 24 |
			((uint64_t) data[2]) << 16 |
			((uint64_t) data[1]) << 8 |
			((uint64_t) data[0]);

	return (all >> (start - len + 1)) & (0xFFFFFFFF >> (32 - len));
}

static void set_uint32(uint8_t *data, int start, int len, uint32_t val) {
	uint64_t all =
			((uint64_t) data[7]) << 56 |
			((uint64_t) data[6]) << 48 |
			((uint64_t) data[5]) << 40 |
			((uint64_t) data[4]) << 32 |
			((uint64_t) data[3]) << 24 |
			((uint64_t) data[2]) << 16 |
			((uint64_t) data[1]) << 8 |
			((uint64_t) data[0]);

	all &= ~((0xFFFFFFFF >> (32 - len)) << (start - len + 1));
	all |= (uint64_t)val << (start - len + 1);

	data[7] = all >> 56;
	data[6] = all >> 48;
	data[5] = all >> 40;
	data[4] = all >> 32;
	data[3] = all >> 24;
	data[2] = all >> 16;
	data[1] = all >> 8;
	data[0] = all;
}
