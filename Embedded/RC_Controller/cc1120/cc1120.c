/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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

#include "cc1120.h"
#include "commands.h"
#include "utils.h"

// Settings
#define CC1120_SPI								SPID1
#define CC1120_PORT_CS							GPIOA
#define CC1120_PIN_CS							4
#define CC1120_PORT_SCK							GPIOA
#define CC1120_PIN_SCK							5
#define CC1120_PORT_MISO						GPIOA
#define CC1120_PIN_MISO							6
#define CC1120_PORT_MOSI						GPIOA
#define CC1120_PIN_MOSI							7
#define CC1120_PORT_RESET						GPIOA
#define CC1120_PIN_RESET						0
#define CC1120_PORT_GPIO0						GPIOA
#define CC1120_PIN_GPIO0						3
#define CC1120_PORT_GPIO2						GPIOA
#define CC1120_PIN_GPIO2						2
#define CC1120_PORT_GPIO3						GPIOA
#define CC1120_PIN_GPIO3						1

#define CC1120_MAX_PAYLOAD						1024

// Macros
#define CC1120_READ_BIT							0x80
#define CC1120_WRITE_BIT						0x00
#define CC1120_BURST_BIT						0x40
#define CC1120_IS_EXTENDED(x)					(x & 0x2F00)
#define CC1120_STATUS_STATE_MASK				0x70
#define CC1120_STATUS_STATE_TXFIFO_UNDERFLOW	0x70

/*
 * // Peripherial Clock 42MHz SPI2 SPI3
 * // Peripherial Clock 84MHz SPI1                                SPI1        SPI2/3
 * #define SPI_BaudRatePrescaler_2         ((uint16_t)0x0000) //  42 MHz      21 MHZ
 * #define SPI_BaudRatePrescaler_4         ((uint16_t)0x0008) //  21 MHz      10.5 MHz
 * #define SPI_BaudRatePrescaler_8         ((uint16_t)0x0010) //  10.5 MHz    5.25 MHz
 * #define SPI_BaudRatePrescaler_16        ((uint16_t)0x0018) //  5.25 MHz    2.626 MHz
 * #define SPI_BaudRatePrescaler_32        ((uint16_t)0x0020) //  2.626 MHz   1.3125 MHz
 * #define SPI_BaudRatePrescaler_64        ((uint16_t)0x0028) //  1.3125 MHz  656.25 KHz
 * #define SPI_BaudRatePrescaler_128       ((uint16_t)0x0030) //  656.25 KHz  328.125 KHz
 * #define SPI_BaudRatePrescaler_256       ((uint16_t)0x0038) //  328.125 KHz 164.06 KHz
 */
static const SPIConfig spicfg = {
		NULL,
		CC1120_PORT_CS,
		CC1120_PIN_CS,
		SPI_CR1_BR_0 | SPI_CR1_BR_1 // 5.25 MHz
};

// Threads
static THD_WORKING_AREA(isr_thread_wa, 2048);
static THD_FUNCTION(isr_thread, arg);
static thread_t *isr_tp;

// Private functions
static int interrupt(void);
static void spi_enable(void);
static void spi_disable(void);
static uint8_t spi_exchange(uint8_t x);

// Private variables (infinite length test)
static uint8_t rx_buffer[1100];
static int rx_pos = 0;
#define RX_LEN		50

void cc1120_init(void) {
	palSetPadMode(CC1120_PORT_CS, CC1120_PIN_CS, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(CC1120_PORT_SCK, CC1120_PIN_SCK, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(CC1120_PORT_MISO, CC1120_PIN_MISO, PAL_MODE_ALTERNATE(5));
	palSetPadMode(CC1120_PORT_MOSI, CC1120_PIN_MOSI, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(CC1120_PORT_RESET, CC1120_PIN_RESET, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(CC1120_PORT_GPIO0, CC1120_PIN_GPIO0, PAL_MODE_INPUT);
	palSetPadMode(CC1120_PORT_GPIO2, CC1120_PIN_GPIO2, PAL_MODE_INPUT);
	palSetPadMode(CC1120_PORT_GPIO3, CC1120_PIN_GPIO3, PAL_MODE_INPUT);

	palSetPad(CC1120_PORT_CS, CC1120_PIN_CS);

	palClearPad(CC1120_PORT_RESET, CC1120_PIN_RESET);
	chThdSleepMilliseconds(100);
	palSetPad(CC1120_PORT_RESET, CC1120_PIN_RESET);
	chThdSleepMilliseconds(100);

	spiStart(&CC1120_SPI, &spicfg);

	// Address Config = No address check
	// Bit Rate = 9.6
	// Carrier Frequency = 452.000000
	// Deviation = 2.395630
	// Device Address = 0
	// Manchester Enable = false
	// Modulation Format = 2-GFSK
	// PA Ramping = true
	// Packet Bit Length = 0
	// Packet Length = 3
	// Packet Length Mode = Fixed
	// Performance Mode = High Performance
	// RX Filter BW = 50.000000
	// Symbol rate = 9.6
	// TX Power = 15
	// Whitening = false

	//
	// From TI SmartRF studio 7
	//
	cc1120_single_write(CC1120_SYNC_CFG1, 0x08);      // Sync Word Detection Configuration Reg. 1
	cc1120_single_write(CC1120_DEVIATION_M, 0x3A);    // Frequency Deviation Configuration
	cc1120_single_write(CC1120_MODCFG_DEV_E, 0x0A);   // Modulation Format and Frequency Deviation Configur..
	cc1120_single_write(CC1120_IQIC, 0x44);           // Digital Image Channel Compensation Configuration
	cc1120_single_write(CC1120_CHAN_BW, 0x04);        // Channel Filter Configuration
	cc1120_single_write(CC1120_SYMBOL_RATE2, 0x73);   // Symbol Rate Configuration Exponent and Mantissa [1..
	cc1120_single_write(CC1120_FS_CFG, 0x04);         // Frequency Synthesizer Configuration
	cc1120_single_write(CC1120_PA_CFG0, 0x7D);        // Power Amplifier Configuration Reg. 0
	cc1120_single_write(CC1120_FREQ2, 0x71);          // Frequency Configuration [23:16]
	cc1120_single_write(CC1120_FS_DIG1, 0x00);        // Frequency Synthesizer Digital Reg. 1
	cc1120_single_write(CC1120_FS_DIG0, 0x5F);        // Frequency Synthesizer Digital Reg. 0
	cc1120_single_write(CC1120_FS_CAL1, 0x40);        // Frequency Synthesizer Calibration Reg. 1
	cc1120_single_write(CC1120_FS_CAL0, 0x0E);        // Frequency Synthesizer Calibration Reg. 0
	cc1120_single_write(CC1120_FS_DIVTWO, 0x03);      // Frequency Synthesizer Divide by 2
	cc1120_single_write(CC1120_FS_DSM0, 0x33);        // FS Digital Synthesizer Module Configuration Reg. 0
	cc1120_single_write(CC1120_FS_DVC0, 0x17);        // Frequency Synthesizer Divider Chain Configuration ..
	cc1120_single_write(CC1120_FS_PFD, 0x50);         // Frequency Synthesizer Phase Frequency Detector Con..
	cc1120_single_write(CC1120_FS_PRE, 0x6E);         // Frequency Synthesizer Prescaler Configuration
	cc1120_single_write(CC1120_FS_REG_DIV_CML, 0x14); // Frequency Synthesizer Divider Regulator Configurat..
	cc1120_single_write(CC1120_FS_SPARE, 0xAC);       // Frequency Synthesizer Spare
	cc1120_single_write(CC1120_FS_VCO0, 0xB4);        // FS Voltage Controlled Oscillator Configuration Reg..
	cc1120_single_write(CC1120_XOSC5, 0x0E);          // Crystal Oscillator Configuration Reg. 5
	cc1120_single_write(CC1120_XOSC1, 0x03);          // Crystal Oscillator Configuration Reg. 1

	// IO Configuration
	cc1120_single_write(CC1120_IOCFG3, IOCFG_GPIO_CFG_CS);
	cc1120_single_write(CC1120_IOCFG2, IOCFG_GPIO_CFG_HIGHZ);
	cc1120_single_write(CC1120_IOCFG1, IOCFG_GPIO_CFG_HIGHZ); // This pin is shared with SPI
//	cc1120_single_write(CC1120_IOCFG0, IOCFG_GPIO_CFG_PKT_SYNC_RXTX | IOCFG_GPIO_CFG_INVERT);
	cc1120_single_write(CC1120_IOCFG0, IOCFG_GPIO_CFG_RXFIFO_THR_PKT);

	// Packet configuration: Infinite length, use CRC1, no address, 32 sync bits (use default bits),
	// 4 preamble bytes, preamble byte 0xAA, no data whitening, interrupt on 2 bytes in FIFO
	cc1120_single_write(CC1120_PKT_CFG0, PKT_CFG0_LENGTH_CONFIG_INFINITE);
	cc1120_single_write(CC1120_PKT_CFG1, PKT_CFG1_APPEND_STATUS | PKT_CFG1_ADDR_CHECK_OFF | PKT_CFG1_CRC_ON_1);
	cc1120_single_write(CC1120_FIFO_CFG, 2);
	cc1120_single_write(CC1120_PREAMBLE_CFG1, PREAMBLE_CFG1_NUM_4 | PREAMBLE_CFG1_WORD_AA);
	cc1120_single_write(CC1120_SYNC_CFG0, SYNC_CFG0_32_BITS | SYNC_CFG0_NUM_ERROR_ENABLED);
	cc1120_single_write(CC1120_SYNC3, 0x93);
	cc1120_single_write(CC1120_SYNC2, 0x0B);
	cc1120_single_write(CC1120_SYNC1, 0x51);
	cc1120_single_write(CC1120_SYNC0, 0xDE);

	// Other settings
	cc1120_single_write(CC1120_RFEND_CFG0, RFEND_CFG0_TXOFF_MODE_RETURN_TO_RX); // Return to RX after TX
	cc1120_single_write(CC1120_RFEND_CFG1, 0x0F | RFEND_CFG1_RXOFF_MODE_RETURN_TO_RX); // Stay in RX after RX
	cc1120_single_write(CC1120_AGC_CS_THR, -10); // Carrier Sense Threshold (lower = more sensitive)
	cc1120_single_write(CC1120_PA_CFG2, 0x3F); // Full output power

	// Infinite length rx test
//	cc1120_single_write(CC1120_IOCFG0, IOCFG_GPIO_CFG_RXFIFO_THR_PKT); // FIFO threshold interrupt
//	cc1120_single_write(CC1120_PKT_CFG0, PKT_CFG0_LENGTH_CONFIG_INFINITE);
//	cc1120_single_write(CC1120_SYNC_CFG0, SYNC_CFG0_16_BITS | SYNC_CFG0_NUM_ERROR_DISABLED);
//	cc1120_single_write(CC1120_SYNC1, 0xAA);
//	cc1120_single_write(CC1120_SYNC0, 0xAA);
//	cc1120_single_write(CC1120_SYNC1, 20);
//	cc1120_single_write(CC1120_SYNC0, 12);
//	cc1120_single_write(CC1120_SYNC1, 0xD3);
//	cc1120_single_write(CC1120_SYNC0, 0x00);
//	cc1120_single_write(CC1120_FIFO_CFG, 10); // Interrupt when there are 10 bytes in fifo

	chThdCreateStatic(isr_thread_wa, sizeof(isr_thread_wa), NORMALPRIO + 2, isr_thread, NULL);
	extChannelEnable(&EXTD1, 3);

	// Manual calibration. Only needed for PARTVERSION 0x21
//	cc1120_off();
//	cc1120_strobe(CC1120_SIDLE);
//	cc1120_calibrate_manual();

	cc1120_on();
}

uint8_t cc1120_state(void) {
	return cc1120_single_read(CC1120_MARCSTATE) & 0x1f;
}

char *cc1120_state_name(void) {
	uint8_t state = cc1120_state();

	switch (state) {
		case 0: return "SLEEP"; break;
		case 1: return "IDLE"; break;
		case 2: return "XOFF"; break;
		case 3: return "BIAS_SETTLE_MC"; break;
		case 4: return "REG_SETTLE_MC"; break;
		case 5: return "MANCAL"; break;
		case 6: return "BIAS_SETTLE"; break;
		case 7: return "REG_SETTLE"; break;
		case 8: return "STARTCAL"; break;
		case 9: return "BWBOOST"; break;
		case 10: return "FS_LOCK"; break;
		case 11: return "IFADCON"; break;
		case 12: return "ENDCAL"; break;
		case 13: return "RX"; break;
		case 14: return "RX_END"; break;
		case 15: return "Reserved"; break;
		case 16: return "TXRX_SWITCH"; break;
		case 17: return "RX_FIFO_ERR"; break;
		case 18: return "FSTXON"; break;
		case 19: return "TX"; break;
		case 20: return "TX_END"; break;
		case 21: return "RXTX_SWITCH"; break;
		case 22: return "TX_FIFO_ERR"; break;
		case 23: return "IFADCON_TXRX"; break;

		default:
			return "UNKNOWN";
			break;
	}
}

uint8_t cc1120_strobe(uint8_t strobe) {
	uint8_t ret;

	spiAcquireBus(&CC1120_SPI);
	spi_enable();
	ret = spi_exchange(strobe);
	spi_disable();
	spiReleaseBus(&CC1120_SPI);

	return ret;
}

uint8_t cc1120_single_read(uint16_t addr) {
	uint8_t val;

	spiAcquireBus(&CC1120_SPI);
	spi_enable();

	if(CC1120_IS_EXTENDED(addr)) {
		addr &= ~0x2F00;
		spi_exchange(CC1120_EXTENDED_MEMORY_ACCESS | CC1120_READ_BIT);
		spi_exchange(addr);
	} else {
		spi_exchange(addr | CC1120_READ_BIT);
	}

	val = spi_exchange(0xFF);

	spi_disable();
	spiReleaseBus(&CC1120_SPI);

	return val;
}

uint8_t cc1120_single_write(uint16_t addr, uint8_t val) {
	uint8_t ret;

	spiAcquireBus(&CC1120_SPI);
	spi_enable();

	if(CC1120_IS_EXTENDED(addr)) {
		addr &= ~0x2F00;
		spi_exchange(CC1120_EXTENDED_MEMORY_ACCESS | CC1120_WRITE_BIT);
		spi_exchange(addr);
	} else {
		spi_exchange(addr | CC1120_WRITE_BIT);
	}

	ret = spi_exchange(val);

	spi_disable();
	spiReleaseBus(&CC1120_SPI);

	return ret;
}

void cc1120_burst_read(uint16_t addr, uint8_t *buffer, uint8_t count) {
	spiAcquireBus(&CC1120_SPI);
	spi_enable();

	if(CC1120_IS_EXTENDED(addr)) {
		addr &= ~0x2F00;
		spi_exchange(CC1120_EXTENDED_MEMORY_ACCESS | CC1120_READ_BIT | CC1120_BURST_BIT);
		spi_exchange(addr);
	} else {
		spi_exchange(addr | CC1120_READ_BIT | CC1120_BURST_BIT);
	}

	spiReceive(&CC1120_SPI, count, buffer);

	spi_disable();
	spiReleaseBus(&CC1120_SPI);
}

void cc1120_burst_write(uint16_t addr, uint8_t *buffer, uint8_t count) {
	spiAcquireBus(&CC1120_SPI);
	spi_enable();

	if(CC1120_IS_EXTENDED(addr)) {
		addr &= ~0x2F00;
		spi_exchange(CC1120_EXTENDED_MEMORY_ACCESS | CC1120_WRITE_BIT | CC1120_BURST_BIT);
		spi_exchange(addr);
	} else {
		spi_exchange(addr | CC1120_WRITE_BIT | CC1120_BURST_BIT);
	}

	spiSend(&CC1120_SPI, count, buffer);

	spi_disable();
	spiReleaseBus(&CC1120_SPI);
}

void cc1120_write_txfifo(uint8_t *data, int len) {
	uint8_t status;
	int i;

	// The length is the first two bytes
	uint8_t buf_len[2];
	buf_len[0] = (len >> 8) & 0xFF;
	buf_len[1] = len & 0xFF;
	cc1120_burst_write(CC1120_TXFIFO, buf_len, 2);

	i = MIN(len, 8);
	cc1120_burst_write(CC1120_TXFIFO, data, i);
	cc1120_strobe(CC1120_STX);

	if(len > i) {
		spiAcquireBus(&CC1120_SPI);
		spi_enable();
		spi_exchange(CC1120_TXFIFO | 0x40);

		for(;i < len;i++) {
			status = spi_exchange(data[i]);

			if((status & CC1120_STATUS_STATE_MASK) ==
					CC1120_STATUS_STATE_TXFIFO_UNDERFLOW) {

				spi_disable();
				spiReleaseBus(&CC1120_SPI);

				// TX FIFO underflow, acknowledge it with an SFTX (otherwise the
				// radio becomes completely unresponsive) followed by an SRX,
				// and break the transmission.

				cc1120_strobe(CC1120_SFTX);
				cc1120_strobe(CC1120_SRX);
				spiAcquireBus(&CC1120_SPI);
				break;
			} else if((status & 0x0f) < 2) {
				// see: https://e2e.ti.com/support/wireless_connectivity/proprietary_sub_1_ghz_simpliciti/f/156/t/330634
				// These bits should tell how much space there is left in the FIFO, but there is an issue
				// so they are reserved in the datasheet.

				spi_disable();
				spiReleaseBus(&CC1120_SPI);

				int to = 100;
				while (!(cc1120_single_read(CC1120_NUM_TXBYTES) < 60 ||
						(cc1120_single_read(CC1120_NUM_TXBYTES) & 0x80) != 0) &&
						to > 0) {
					chThdSleepMilliseconds(1);
					to--;
				}

				if(cc1120_single_read(CC1120_NUM_TXBYTES) & 0x80) {
					// TX FIFO underflow.
					cc1120_strobe(CC1120_SFTX);
					cc1120_strobe(CC1120_SRX);
					commands_printf("TX FIFO underflow");
					break;
				}

				spiAcquireBus(&CC1120_SPI);
				spi_enable();
				spi_exchange(CC1120_TXFIFO | 0x40);
			}
		}

		spi_disable();
		spiReleaseBus(&CC1120_SPI);
	}
}

void cc1120_check_txfifo(void) {
	if(cc1120_state() == CC1120_STATUS_STATE_TXFIFO_UNDERFLOW) {
		// Acknowledge TX FIFO underflow.
		cc1120_strobe(CC1120_SFTX);
		cc1120_strobe(CC1120_SRX);
		commands_printf("TX FIFO underflow");
	}
}

void cc1120_flushrx(void) {
	commands_printf("Flush RX");

	if(cc1120_state() == CC1120_STATE_RXFIFO_OVERFLOW) {
		cc1120_strobe(CC1120_SFRX);
	}

	cc1120_strobe(CC1120_SIDLE);

	int to = 100;
	while (!(cc1120_state() == CC1120_STATE_IDLE) && to > 0) {
		to--;
	}

	cc1120_strobe(CC1120_SFRX);
	cc1120_strobe(CC1120_SRX);
}

int cc1120_transmit(uint8_t *data, int len) {
	if(cc1120_state() == CC1120_STATE_RXFIFO_OVERFLOW) {
		cc1120_flushrx();
	}

	if(len > CC1120_MAX_PAYLOAD) {
		commands_printf("CC1120: too many tx bytes %d", len);
		return -1;
	}

	cc1120_strobe(CC1120_SIDLE);
	cc1120_write_txfifo(data, len);

	int to = 1000;
	while (cc1120_state() != CC1120_STATE_TX && to > 0) {
		chThdSleepMilliseconds(1);
		to--;
	}

	if(cc1120_state() != CC1120_STATE_TX) {
		commands_printf("Didn't start tx (state %s)", cc1120_state_name());
		cc1120_check_txfifo();
		cc1120_flushrx();
		return -2;
	}

	to = 1000;
	while (cc1120_state() == CC1120_STATE_TX && to > 0) {
		chThdSleepMilliseconds(1);
		to--;
	}

	if(cc1120_state() == CC1120_STATE_TX) {
		commands_printf("Didn't end tx (state %s, txbytes %d, to %d)",
				cc1120_state_name(), cc1120_single_read(CC1120_NUM_TXBYTES), to);
		cc1120_check_txfifo();
		cc1120_flushrx();
		return -3;
	}

	cc1120_check_txfifo();

	return 1;
}

int cc1120_on(void) {
	cc1120_flushrx();
	cc1120_strobe(CC1120_SRX);
	return 1;
}

int cc1120_off(void) {
	cc1120_strobe(CC1120_SIDLE);

	int to = 100;
	while (!(cc1120_state() == CC1120_STATE_IDLE) && to > 0) {
		chThdSleepMilliseconds(1);
		to--;
	}

	cc1120_strobe(CC1120_SPWD);

	return 1;
}

bool cc1120_carrier_sense(void) {
//	uint8_t reg = cc1120_single_read(CC1120_RSSI0);
//	return ((reg & 0b010) && (reg & 0b100)) ? 1 : 0;
	return palReadPad(CC1120_PORT_GPIO3, CC1120_PIN_GPIO3);
}

void cc1120_ext_cb(EXTDriver *extp, expchannel_t channel) {
	(void)extp;
	(void)channel;

	chSysLockFromISR();
	chEvtSignalI(isr_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

static THD_FUNCTION(isr_thread, arg) {
	(void) arg;
	chRegSetThreadName("CC1120 EXTI");

	isr_tp = chThdGetSelfX();

	for (;;) {
		chEvtWaitAny((eventmask_t) 1);
		commands_printf("CC1120_IRQ");
		interrupt();
	}
}

// Private functions

static int interrupt(void) {
	uint8_t rxbytes, s;

	s = cc1120_state();
	if(s == CC1120_STATE_RXFIFO_OVERFLOW) {
		cc1120_burst_read(CC1120_NUM_RXBYTES, &rxbytes, 1);
		commands_printf("CC1120 ISR: RX overflow, flush");
		commands_printf("CC1120 ISR: rxbytes 0x%02x", rxbytes);
		cc1120_flushrx();
		return 1;
	}

	if(s == CC1120_STATE_TXFIFO_UNDERFLOW) {
		commands_printf("CC1120 ISR: TX underflow, flush");
		cc1120_strobe(CC1120_SFTX);
		cc1120_strobe(CC1120_SRX);
		return 1;
	}

	do {
		rxbytes = cc1120_single_read(CC1120_NUM_RXBYTES);
		commands_printf("CC1120 ISR: rxbytes: %d", rxbytes);

		if(rxbytes == 0) {
			commands_printf("CC1120 ISR: FIFO empty");
			return 1;
		}

		cc1120_burst_read(CC1120_RXFIFO, rx_buffer + rx_pos, rxbytes);
		rx_pos += rxbytes;

		int rx_len = 0;
		if (rx_pos >= 2) {
			rx_len = (uint16_t)rx_buffer[0] << 8 | (uint16_t)rx_buffer[1];
		}

		if (rx_len > 0) {
			if(rx_len > CC1120_MAX_PAYLOAD) {
				commands_printf("CC1120 ISR: rx_len too large %d", rx_len);
				cc1120_flushrx();
				cc1120_single_write(CC1120_PKT_CFG0, PKT_CFG0_LENGTH_CONFIG_INFINITE);
				cc1120_single_write(CC1120_FIFO_CFG, 2);
				return -1;
			} else {
				cc1120_single_write(CC1120_FIFO_CFG, 50);
			}

			// Switch to fixed length mode and set the remaining length if less than 255 bytes are left
			if ((RX_LEN - rx_pos) < 255) {
				cc1120_single_write(CC1120_PKT_LEN, RX_LEN % 256);
				cc1120_single_write(CC1120_PKT_CFG0, PKT_CFG0_LENGTH_CONFIG_FIXED);
			}

			// The whole packet is received
			if (rx_pos >= rx_len) {
				cc1120_flushrx();
				cc1120_single_write(CC1120_PKT_CFG0, PKT_CFG0_LENGTH_CONFIG_INFINITE);
				cc1120_single_write(CC1120_FIFO_CFG, 2);
				rx_pos = 0;

				// Print the bytes for now
				for (int i = 0;i < rx_len;i++) {
					commands_printf("CC1120 RX Byte: %d", rx_buffer[i]);
				}
			}
		}

		if (rxbytes) {
			commands_printf(" ");
		}

		rxbytes = cc1120_single_read(CC1120_NUM_RXBYTES);
	} while(rxbytes > 1);

	return 1;
}

static void spi_enable(void) {
	palClearPad(CC1120_PORT_CS, CC1120_PIN_CS);

	int timeout = 100;
	while (palReadPad(CC1120_PORT_MISO, CC1120_PIN_MISO)) {
		chThdSleepMilliseconds(1);
		timeout--;
		if (!timeout) {
			commands_printf("CC1120 SPI Enable Timeout");
			break;
		}
	}
}

static void spi_disable(void) {
	palSetPad(CC1120_PORT_CS, CC1120_PIN_CS);
}

static uint8_t spi_exchange(uint8_t x) {
	uint8_t rx;
	spiExchange(&CC1120_SPI, 1, &x, &rx);
	return rx;
}

// See http://www.ti.com/lit/er/swrz039d/swrz039d.pdf
#define VCDAC_START_OFFSET  2
#define FS_VCO2_INDEX       0
#define FS_VCO4_INDEX       1
#define FS_CHP_INDEX        2

void cc1120_calibrate_manual(void) {
	uint8_t original_fs_cal2;
	uint8_t calResults_for_vcdac_start_high[3];
	uint8_t calResults_for_vcdac_start_mid[3];
	uint8_t marcstate;
	uint8_t writeByte;

	// 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
	writeByte = 0x00;
	cc1120_burst_write(CC1120_FS_VCO2, &writeByte, 1);

	// 2) Start with high VCDAC (original VCDAC_START + 2):
	cc1120_burst_read(CC1120_FS_CAL2, &original_fs_cal2, 1);
	writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
	cc1120_burst_write(CC1120_FS_CAL2, &writeByte, 1);

	// 3) Calibrate and wait for calibration to be done (radio back in IDLE state)
	cc1120_strobe(CC1120_SCAL);
	do {
		cc1120_burst_read(CC1120_MARCSTATE, &marcstate, 1);
	} while(marcstate != 0x41);

	// 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with high VCDAC_START value
	cc1120_burst_read(CC1120_FS_VCO2,
			&calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
	cc1120_burst_read(CC1120_FS_VCO4,
			&calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
	cc1120_burst_read(CC1120_FS_CHP, &calResults_for_vcdac_start_high[FS_CHP_INDEX],
			1);

	// 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
	writeByte = 0x00;
	cc1120_burst_write(CC1120_FS_VCO2, &writeByte, 1);

	// 6) Continue with mid VCDAC (original VCDAC_START):
	writeByte = original_fs_cal2;
	cc1120_burst_write(CC1120_FS_CAL2, &writeByte, 1);

	// 7) Calibrate and wait for calibration to be done (radio back in IDLE state)
	cc1120_strobe(CC1120_SCAL);
	do {
		cc1120_burst_read(CC1120_MARCSTATE, &marcstate, 1);
	} while(marcstate != 0x41);

	// 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with mid VCDAC_START value
	cc1120_burst_read(CC1120_FS_VCO2, &calResults_for_vcdac_start_mid[FS_VCO2_INDEX],
			1);
	cc1120_burst_read(CC1120_FS_VCO4, &calResults_for_vcdac_start_mid[FS_VCO4_INDEX],
			1);
	cc1120_burst_read(CC1120_FS_CHP, &calResults_for_vcdac_start_mid[FS_CHP_INDEX],
			1);

	// 9) Write back highest FS_VCO2 and corresponding FS_VCO and FS_CHP result
	if(calResults_for_vcdac_start_high[FS_VCO2_INDEX]
									   > calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
		writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
		cc1120_burst_write(CC1120_FS_VCO2, &writeByte, 1);
		writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
		cc1120_burst_write(CC1120_FS_VCO4, &writeByte, 1);
		writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
		cc1120_burst_write(CC1120_FS_CHP, &writeByte, 1);
	} else {
		writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
		cc1120_burst_write(CC1120_FS_VCO2, &writeByte, 1);
		writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
		cc1120_burst_write(CC1120_FS_VCO4, &writeByte, 1);
		writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
		cc1120_burst_write(CC1120_FS_CHP, &writeByte, 1);
	}
}
