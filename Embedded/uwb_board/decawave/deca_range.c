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

#include "deca_range.h"
#include "deca_regs.h"
#include "deca_range_tables.h"
#include <string.h>

#include "comm_usb.h"
#include "led.h"

// Make the ranging faster at the cost of some range
#define FAST_MODE						1

// Sizes
#define RX_BUF_LEN						1024
#define ALL_MSG_COMMON_LEN				7
#define ALL_MSG_OFFSET					3
#define ALL_MSG_HEADER_LEN				(ALL_MSG_OFFSET + ALL_MSG_COMMON_LEN)
#define MSG_DATA_MAX_PAYLOAD			110
#define MSG_CRC_LEN						2

// Indexes
#define ALL_MSG_DEST_IDX				0
#define ALL_MSG_SENDER_IDX				1
#define ALL_MSG_SEQUENCE_IDX			2
#define FINAL_MSG_POLL_TX_TS_IDX		10
#define FINAL_MSG_RESP_RX_TS_IDX		14
#define FINAL_MSG_FINAL_TX_TS_IDX		18
#define FINAL_MSG_TS_LEN				4

// Antenna delays
#define TX_ANT_DLY						16436
#define RX_ANT_DLY						16436
//#define TX_ANT_DLY						16410
//#define RX_ANT_DLY						16410

// UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
// 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu.
#define UUS_TO_DWT_TIME					65536

// Speed of light in air, in meters per second.
#define SPEED_OF_LIGHT					299702547.0

// Macro to check if message is of type
#define IS_MSG(buf, msg)				(memcmp(buf + ALL_MSG_OFFSET, msg + ALL_MSG_OFFSET, ALL_MSG_COMMON_LEN) == 0)

#if FAST_MODE

// Delay between frames, in UWB microseconds. See NOTE 4 below.
#define POLL_RX_TO_RESP_TX_DLY_UUS		500

// This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's
// delayed TX function.
#define RESP_RX_TO_FINAL_TX_DLY_UUS		500

static dwt_config_t config = {
		4,					// Channel number.
		DWT_PRF_64M,		// Pulse repetition frequency.
		DWT_PLEN_128,		// Preamble length. Used in TX only.
		DWT_PAC8,			// Preamble acquisition chunk size. Used in RX only.
		9,					// TX preamble code. Used in TX only.
		9,					// RX preamble code. Used in RX only.
		1,					// 0 to use standard SFD, 1 to use non-standard SFD.
		DWT_BR_6M8,			// Data rate.
		DWT_PHRMODE_STD,	// PHY header mode.
		(129 + 8 - 8)		// SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
};

#else

// Delay between frames, in UWB microseconds. See NOTE 4 below.
// This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's
// delayed TX function.
#define POLL_RX_TO_RESP_TX_DLY_UUS		2600

// This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's
// delayed TX function.
#define RESP_RX_TO_FINAL_TX_DLY_UUS		3100

static dwt_config_t config = {
		4,					// Channel number.
		DWT_PRF_64M,		// Pulse repetition frequency.
		DWT_PLEN_1024,		// Preamble length. Used in TX only.
		DWT_PAC32,			// Preamble acquisition chunk size. Used in RX only.
		9,					// TX preamble code. Used in TX only.
		9,					// RX preamble code. Used in RX only.
		1,					// 0 to use standard SFD, 1 to use non-standard SFD.
		DWT_BR_110K,		// Data rate.
		DWT_PHRMODE_STD,	// PHY header mode.
		(1025 + 64 - 32)	// SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
};

#endif

// Buffer to store received data
static uint8 rx_buffer[RX_BUF_LEN];

// Message buffers
static uint8 msg_poll[] = {0, 0, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 msg_resp[] = {0, 0, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 msg_final[] = {0, 0, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 msg_poll_rq[] = {0, 0, 0, 0xCA, 0xDE, 'P', 'O', 'L', 'L', 0x21, 0, 0};
static uint8 msg_data[] = {0, 0, 0, 0xCA, 0xDE, 'D', 'A', 'T', 'A', 0x25, 0, 0};

// Sequence number. Incremented for each packet
static uint8 frame_seq_nb = 0;

// Own address
static uint8_t m_address = 0;

// Mutex
static mutex_t m_lock;

// Range samples to tale
static int m_range_samples = 0;
static int m_range_sample_now = 0;
static float m_range_sample_sum = 0.0;

// Private functions
static void range_poll(uint8_t dest);
static void range_poll_request(uint8_t dest);
static uint64_t get_tx_timestamp_u64(void);
static uint64_t get_rx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
static void final_msg_set_ts(uint8 *ts_field, uint64_t ts);

// Callbacks
static void cb_tx_done(const dwt_cb_data_t *cb_data);
static void cb_rx_ok(const dwt_cb_data_t *cb_data);
static void cb_rx_to(const dwt_cb_data_t *cb_data);
static void cb_rx_err(const dwt_cb_data_t *cb_data);

// Function pointers
static void(*m_range_func)(float dist, uint8_t id) = 0;
static void(*m_data_func)(uint8_t sender, uint8_t *buffer, int len) = 0;

/**
 * Configure the DW as ranging node.
 *
 * @param addr
 * The address of this node.
 */
void deca_range_configure(uint8_t addr)  {
	m_address = addr;
	chMtxObjectInit(&m_lock);

	dwt_configure(&config);
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);

	dwt_setcallbacks(cb_tx_done, cb_rx_ok, cb_rx_to, cb_rx_err);
	dwt_setinterrupt(
			DWT_INT_TFRS |
			DWT_INT_RFCG |
			DWT_INT_RPHE |
			DWT_INT_RFCE |
			DWT_INT_RFSL |
			DWT_INT_RFTO |
			DWT_INT_RXOVRR |
			DWT_INT_RXPTO |
			DWT_INT_SFDT |
			DWT_INT_ARFE,
			1);

	dwt_setrxaftertxdelay(0);
	dwt_setrxtimeout(0);
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/**
 * Start ranging with a node. The node will get the result of the ranging.
 *
 * @param dest
 * The node to start ranging with.
 */
void deca_range_poll(uint8_t dest) {
	chMtxLock(&m_lock);
	range_poll(dest);
	chMtxUnlock(&m_lock);
}

/**
 * Send a range poll request. If the ranging succeeds the range_func
 * callback will be called with the result.
 *
 * @param dest
 * The node address.
 *
 * @param samples
 * The number of sample to take an average over.
 */
void deca_range_measure(uint8_t dest, int samples) {
	chMtxLock(&m_lock);

	m_range_samples = samples;
	m_range_sample_now = 0;
	m_range_sample_sum = 0.0;

	range_poll_request(dest);

	chMtxUnlock(&m_lock);
}

/**
 * Send data to a node.
 *
 * @param dest
 * The node address to send data to.
 *
 * @param buffer
 * Buffer with the data to send.
 *
 * @param len
 * The length of the data. Maximum length is 110 byte.
 *
 * @return
 * 0: ok
 * -1: Send error
 * -2: Too long payload
 */
int deca_range_send_data(uint8_t dest, uint8_t *buffer, int len) {
	int retval = 0;

	if (len < MSG_DATA_MAX_PAYLOAD) {
		chMtxLock(&m_lock);

		dwt_forcetrxoff();

		// Write frame data to DW1000 and prepare transmission. See NOTE 8 below.
		msg_data[ALL_MSG_DEST_IDX] = dest;
		msg_data[ALL_MSG_SENDER_IDX] = m_address;
		msg_data[ALL_MSG_SEQUENCE_IDX] = frame_seq_nb;
		dwt_writetxdata(sizeof(msg_data), msg_data, 0);
		dwt_writetxdata(len + MSG_CRC_LEN, buffer, ALL_MSG_HEADER_LEN);
		dwt_writetxfctrl(sizeof(msg_data) + len, 0, 0);

		// Start transmission, indicating that a response is expected so that reception is enabled automatically
		// after the frame is sent and the delay set by dwt_setrxaftertxdelay() has elapsed.
		int ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

		if (ret == DWT_SUCCESS) {
			frame_seq_nb++;
		} else {
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
			retval = -1;
		}

		chMtxUnlock(&m_lock);
	} else {
		retval = -2;
	}

	return retval;
}

/**
 * Set the address of this node.
 *
 * @param adr
 * The address to set.
 */
void deca_range_set_address(uint8_t addr) {
	m_address = addr;
}

/**
 * Read the address of this node.
 *
 * @return
 * The address of this node.
 */
uint8_t deca_range_get_address(void) {
	return m_address;
}

/**
 * Set the function to be called when ranging is done.
 *
 * @param func
 * A pointer to the ranging function.
 */
void deca_range_set_range_func(void(*func)(float dist, uint8_t id)) {
	m_range_func = func;
}

/**
 * Set the function to be called when data arrives.
 *
 * @param func
 * A pointer to the function that is called when the data arrives.
 */
void deca_range_set_data_func(void(*func)(uint8_t sender, uint8_t *buffer, int len)) {
	m_data_func = func;
}

// Private functions

static void range_poll(uint8_t dest) {
	dwt_forcetrxoff();

	// Write frame data to DW1000 and prepare transmission. See NOTE 8 below.
	msg_poll[ALL_MSG_DEST_IDX] = dest;
	msg_poll[ALL_MSG_SENDER_IDX] = m_address;
	msg_poll[ALL_MSG_SEQUENCE_IDX] = frame_seq_nb;
	dwt_writetxdata(sizeof(msg_poll), msg_poll, 0); // Zero offset in TX buffer.
	dwt_writetxfctrl(sizeof(msg_poll), 0, 1); // Zero offset in TX buffer, ranging.

	// Start transmission, indicating that a response is expected so that reception is enabled automatically
	// after the frame is sent and the delay set by dwt_setrxaftertxdelay() has elapsed.
	int ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

	if (ret == DWT_SUCCESS) {
		frame_seq_nb++;
	} else {
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
	}
}

static void range_poll_request(uint8_t dest) {
	dwt_forcetrxoff();

	// Write frame data to DW1000 and prepare transmission. See NOTE 8 below.
	msg_poll_rq[ALL_MSG_DEST_IDX] = dest;
	msg_poll_rq[ALL_MSG_SENDER_IDX] = m_address;
	msg_poll_rq[ALL_MSG_SEQUENCE_IDX] = frame_seq_nb;
	dwt_writetxdata(sizeof(msg_poll_rq), msg_poll_rq, 0);
	dwt_writetxfctrl(sizeof(msg_poll_rq), 0, 0); // Zero offset in TX buffer, no ranging.

	// Start transmission, indicating that a response is expected so that reception is enabled automatically
	// after the frame is sent and the delay set by dwt_setrxaftertxdelay() has elapsed.
	int ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

	if (ret == DWT_SUCCESS) {
		frame_seq_nb++;
	} else {
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
	}
}

static uint64_t get_tx_timestamp_u64(void) {
	uint8 ts_tab[5];
	uint64_t ts = 0;

	dwt_readtxtimestamp(ts_tab);

	for (int i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}

	return ts;
}

static uint64_t get_rx_timestamp_u64(void) {
	uint8_t ts_tab[5];
	uint64_t ts = 0;

	dwt_readrxtimestamp(ts_tab);
	for (int i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}

	return ts;
}

static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts) {
	*ts = 0;
	for (int i = 0; i < FINAL_MSG_TS_LEN; i++) {
		*ts += ts_field[i] << (i * 8);
	}
}

static void final_msg_set_ts(uint8 *ts_field, uint64_t ts) {
	for (int i = 0; i < FINAL_MSG_TS_LEN; i++) {
		ts_field[i] = (uint8) ts;
		ts >>= 8;
	}
}

static void cb_tx_done(const dwt_cb_data_t *cb_data) {
	(void)cb_data;

//	comm_usb_printf("tx done\r\n");
//	led_toggle(LED_GREEN);
}

static void cb_rx_ok(const dwt_cb_data_t *cb_data) {
	chMtxLock(&m_lock);

	int len = cb_data->datalength;

	// A frame has been received, read it into the local buffer.
	dwt_readrxdata(rx_buffer, len, 0);

	static uint64_t poll_rx_ts;

	uint8_t dest = rx_buffer[ALL_MSG_DEST_IDX];
	uint8_t sender = rx_buffer[ALL_MSG_SENDER_IDX];
	uint8_t seq = rx_buffer[ALL_MSG_SEQUENCE_IDX];

	// Don't use sequence number for now
	(void)seq;

	if (dest != m_address) {
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		chMtxUnlock(&m_lock);
		return;
	}

	if (IS_MSG(rx_buffer, msg_poll)) {
		uint32 resp_tx_time;
		int ret;

		// Retrieve poll reception timestamp.
		poll_rx_ts = get_rx_timestamp_u64();

		// Set send time for response. See NOTE 9 below.
		resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
		dwt_setdelayedtrxtime(resp_tx_time);

		// Write and send the response message. See NOTE 10
		msg_resp[ALL_MSG_DEST_IDX] = sender;
		msg_resp[ALL_MSG_SENDER_IDX] = m_address;
		msg_resp[ALL_MSG_SEQUENCE_IDX] = frame_seq_nb;
		dwt_writetxdata(sizeof(msg_resp), msg_resp, 0); // Zero offset in TX buffer.
		dwt_writetxfctrl(sizeof(msg_resp), 0, 1); // Zero offset in TX buffer, ranging.
		ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

		if (ret == DWT_ERROR) {
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
		} else {
			frame_seq_nb++;
		}

		chMtxUnlock(&m_lock);
	} else if (IS_MSG(rx_buffer, msg_final)) {
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		chMtxUnlock(&m_lock);

		uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
		uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
		double Ra, Rb, Da, Db;
		int64_t tof_dtu;

		// Retrieve response transmission and final reception timestamps.
		uint64_t resp_tx_ts = get_tx_timestamp_u64();
		uint64_t final_rx_ts = get_rx_timestamp_u64();

		// Get timestamps embedded in the final message.
		final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
		final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
		final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

		// Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12
		poll_rx_ts_32 = (uint32)poll_rx_ts;
		resp_tx_ts_32 = (uint32)resp_tx_ts;
		final_rx_ts_32 = (uint32)final_rx_ts;
		Ra = (double)(resp_rx_ts - poll_tx_ts);
		Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
		Da = (double)(final_tx_ts - resp_rx_ts);
		Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
		tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

		float tof = tof_dtu * DWT_TIME_UNITS;
		float distance = tof * SPEED_OF_LIGHT;
		distance -= dwt_getrangebias(config.chan, distance, config.prf);

		m_range_sample_sum += distance;
		m_range_sample_now++;

		if (m_range_sample_now >= m_range_samples) {
			float dist_avg = m_range_sample_sum / (float)m_range_sample_now;
			m_range_sample_sum = 0.0;
			m_range_sample_now = 0;
			m_range_samples = 0;

			if (m_range_func) {
				m_range_func(dist_avg, sender);
			}
		} else {
			range_poll_request(sender);
		}
	} else if (IS_MSG(rx_buffer, msg_resp)) {
		uint32 final_tx_time;
		int ret;

		// Retrieve poll transmission and response reception timestamp.
		uint64_t poll_tx_ts = get_tx_timestamp_u64();
		uint64_t resp_rx_ts = get_rx_timestamp_u64();

		// Compute final message transmission time. See NOTE 10 below.
		final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
		dwt_setdelayedtrxtime(final_tx_time);

		// Final TX timestamp is the transmission time we programmed plus the TX antenna delay.
		uint64_t final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

		// Write all timestamps in the final message. See NOTE 11 below.
		final_msg_set_ts(&msg_final[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
		final_msg_set_ts(&msg_final[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
		final_msg_set_ts(&msg_final[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

		// Write and send final message. See NOTE 8 below.
		msg_final[ALL_MSG_DEST_IDX] = sender;
		msg_final[ALL_MSG_SENDER_IDX] = m_address;
		msg_final[ALL_MSG_SEQUENCE_IDX] = frame_seq_nb;
		dwt_writetxdata(sizeof(msg_final), msg_final, 0); // Zero offset in TX buffer.
		dwt_writetxfctrl(sizeof(msg_final), 0, 1); // Zero offset in TX buffer, ranging.
		ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

		if (ret == DWT_SUCCESS) {
			frame_seq_nb++;
		} else {
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}

		chMtxUnlock(&m_lock);
	} else if (IS_MSG(rx_buffer, msg_poll_rq)) {
		// Poll request.
		range_poll(sender);
		chMtxUnlock(&m_lock);
		led_toggle(LED_GREEN);
	} else if (IS_MSG(rx_buffer, msg_data)) {
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		chMtxUnlock(&m_lock);

		// Data message
		if (m_data_func) {
			m_data_func(sender, rx_buffer + ALL_MSG_HEADER_LEN, len - ALL_MSG_HEADER_LEN - MSG_CRC_LEN);
		}
	} else {
		// Unknown message
		led_toggle(LED_RED);
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		chMtxUnlock(&m_lock);
	}
}

static void cb_rx_to(const dwt_cb_data_t *cb_data) {
	(void)cb_data;

//	comm_usb_printf("rx to\r\n");
//	led_toggle(LED_RED);

	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void cb_rx_err(const dwt_cb_data_t *cb_data) {
	(void)cb_data;

//	comm_usb_printf("rx err\r\n");
//	led_toggle(LED_RED);

	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}
