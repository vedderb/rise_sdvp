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

#define CC1120_MAX_PAYLOAD						1100

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
static THD_WORKING_AREA(check_thread_wa, 256);
static THD_FUNCTION(check_thread, arg);

// Private functions
static int interrupt(void);
static void spi_enable(void);
static void spi_disable(void);
static uint8_t spi_exchange(uint8_t x);
static void calibrate_manual(void);

// Private variables
static uint8_t m_rx_buffer[CC1120_MAX_PAYLOAD + 10];
static int m_rx_pos = 0;
static bool m_init_done = false;
static float m_last_freqoff_est;
static mutex_t m_radio_mutex;

// Function pointers
static void(*rx_callback)(uint8_t *data, int len, int rssi, int lqi, bool crc_ok) = 0;

bool cc1120_init(void) {
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

	chMtxObjectInit(&m_radio_mutex);

	// Check the partnumber to confirm that the chip communicates.
	if (cc1120_single_read(CC1120_PARTNUMBER) != 0x48) {
		return false;
	}

	// IO Configuration
	cc1120_single_write(CC1120_IOCFG3, IOCFG_GPIO_CFG_CS);
	cc1120_single_write(CC1120_IOCFG2, IOCFG_GPIO_CFG_HIGHZ);
	cc1120_single_write(CC1120_IOCFG1, IOCFG_GPIO_CFG_HIGHZ); // This pin is shared with SPI
	cc1120_single_write(CC1120_IOCFG0, IOCFG_GPIO_CFG_RXFIFO_THR_PKT);

	// Packet configuration: Infinite length, use CRC1, no address, 32 sync bits (use default bits),
	// 4 preamble bytes, preamble byte 0xAA, data whitening, interrupt on 2 bytes in FIFO,
	// append status to RX fifo
	cc1120_single_write(CC1120_PKT_CFG0, PKT_CFG0_LENGTH_CONFIG_INFINITE);
	cc1120_single_write(CC1120_PKT_CFG1, PKT_CFG1_APPEND_STATUS | PKT_CFG1_ADDR_CHECK_OFF | PKT_CFG1_CRC_ON_1 | PKT_CFG1_WHITE_DATA);
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
	cc1120_single_write(CC1120_FREQOFF_CFG, FREQOFF_CFG_FOC_EN | FREQOFF_CFG_FOC_CFG_10);

	// Start threads
	chThdCreateStatic(isr_thread_wa, sizeof(isr_thread_wa), NORMALPRIO + 2, isr_thread, NULL);
	chThdCreateStatic(check_thread_wa, sizeof(check_thread_wa), NORMALPRIO, check_thread, NULL);

	// Enable interrupt
	extChannelEnable(&EXTD1, 3);

	cc1120_update_rf(CC1120_SET_434_0M_9_6K_2FSK_BW50K_12K);

	// Manual calibration. Only needed for PARTVERSION 0x21
//	cc1120_set_idle();
//	calibrate_manual();
	(void)calibrate_manual;

	cc1120_on();

	m_init_done = true;

	return true;
}

bool cc1120_init_done(void) {
	return m_init_done;
}

void cc1120_update_rf(CC1120_SETTINGS set) {
	cc1120_set_idle();

	// Note: These values were generated with TI smartrf studio

	// Set default register values
	cc1120_single_write(CC1120_SYNC_CFG1, 0x0A);      // Sync Word Detection Configuration Reg. 1
	cc1120_single_write(CC1120_DEVIATION_M, 0x06);    // Frequency Deviation Configuration
	cc1120_single_write(CC1120_MODCFG_DEV_E, 0x03);   // Modulation Format and Frequency Deviation Configur..
	cc1120_single_write(CC1120_IQIC, 0xC4);           // Digital Image Channel Compensation Configuration
	cc1120_single_write(CC1120_CHAN_BW, 0x14);        // Channel Filter Configuration
	cc1120_single_write(CC1120_SYMBOL_RATE2, 0x43);   // Symbol Rate Configuration Exponent and Mantissa [1..
	cc1120_single_write(CC1120_SYMBOL_RATE1, 0xA9);   // Symbol Rate Configuration Mantissa [15:8]
	cc1120_single_write(CC1120_SYMBOL_RATE0, 0x2A);   // Symbol Rate Configuration Mantissa [7:0]
	cc1120_single_write(CC1120_FS_CFG, 0x02);         // Frequency Synthesizer Configuration
	cc1120_single_write(CC1120_PA_CFG0, 0x7C);        // Power Amplifier Configuration Reg. 0
	cc1120_single_write(CC1120_FREQ2, 0x00);          // Frequency Configuration [23:16]
	cc1120_single_write(CC1120_FREQ1, 0x00);          // Frequency Configuration [15:8]
	cc1120_single_write(CC1120_FREQ0, 0x00);          // Frequency Configuration [7:0]
	cc1120_single_write(CC1120_FS_DIG1, 0x08);        // Frequency Synthesizer Digital Reg. 1
	cc1120_single_write(CC1120_FS_DIG0, 0x5A);        // Frequency Synthesizer Digital Reg. 0
	cc1120_single_write(CC1120_FS_CAL3, 0x00);        // Frequency Synthesizer Calibration Reg. 3
	cc1120_single_write(CC1120_FS_CAL2, 0x20);        // Frequency Synthesizer Calibration Reg. 2
	cc1120_single_write(CC1120_FS_CAL1, 0x00);        // Frequency Synthesizer Calibration Reg. 1
	cc1120_single_write(CC1120_FS_CAL0, 0x00);        // Frequency Synthesizer Calibration Reg. 0
	cc1120_single_write(CC1120_FS_CHP, 0x28);         // Frequency Synthesizer Charge Pump Configuration
	cc1120_single_write(CC1120_FS_DIVTWO, 0x01);      // Frequency Synthesizer Divide by 2
	cc1120_single_write(CC1120_FS_DSM1, 0x00);        // FS Digital Synthesizer Module Configuration Reg. 1
	cc1120_single_write(CC1120_FS_DSM0, 0x03);        // FS Digital Synthesizer Module Configuration Reg. 0
	cc1120_single_write(CC1120_FS_DVC1, 0xFF);        // Frequency Synthesizer Divider Chain Configuration ..
	cc1120_single_write(CC1120_FS_DVC0, 0x1F);        // Frequency Synthesizer Divider Chain Configuration ..
	cc1120_single_write(CC1120_FS_LBI, 0x00);         // Frequency Synthesizer Local Bias Configuration
	cc1120_single_write(CC1120_FS_PFD, 0x51);         // Frequency Synthesizer Phase Frequency Detector Con..
	cc1120_single_write(CC1120_FS_PRE, 0x2C);         // Frequency Synthesizer Prescaler Configuration
	cc1120_single_write(CC1120_FS_REG_DIV_CML, 0x11); // Frequency Synthesizer Divider Regulator Configurat..
	cc1120_single_write(CC1120_FS_SPARE, 0x00);       // Frequency Synthesizer Spare
	cc1120_single_write(CC1120_FS_VCO4, 0x14);        // FS Voltage Controlled Oscillator Configuration Reg..
	cc1120_single_write(CC1120_FS_VCO3, 0x00);        // FS Voltage Controlled Oscillator Configuration Reg..
	cc1120_single_write(CC1120_FS_VCO2, 0x00);        // FS Voltage Controlled Oscillator Configuration Reg..
	cc1120_single_write(CC1120_FS_VCO1, 0x00);        // FS Voltage Controlled Oscillator Configuration Reg..
	cc1120_single_write(CC1120_FS_VCO0, 0x81);        // FS Voltage Controlled Oscillator Configuration Reg..
	cc1120_single_write(CC1120_XOSC5, 0x0C);          // Crystal Oscillator Configuration Reg. 5
	cc1120_single_write(CC1120_XOSC4, 0xA0);          // Crystal Oscillator Configuration Reg. 4
	cc1120_single_write(CC1120_XOSC3, 0x03);          // Crystal Oscillator Configuration Reg. 3
	cc1120_single_write(CC1120_XOSC2, 0x04);          // Crystal Oscillator Configuration Reg. 2
	cc1120_single_write(CC1120_XOSC1, 0x01);          // Crystal Oscillator Configuration Reg. 1
	cc1120_single_write(CC1120_XOSC0, 0x00);          // Crystal Oscillator Configuration Reg. 0

	switch (set) {
	case CC1120_SET_434_0M_1_2K_2FSK_BW25K_4K:
		cc1120_single_write(CC1120_CHAN_BW, 0x08);        // Channel Filter Configuration
		cc1120_single_write(CC1120_FS_CFG, 0x04);         // Frequency Synthesizer Configuration
		cc1120_single_write(CC1120_FREQ2, 0x6C);          // Frequency Configuration [23:16]
		cc1120_single_write(CC1120_FREQ1, 0x80);          // Frequency Configuration [15:8]
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
		break;

	case CC1120_SET_434_0M_1_2K_2FSK_BW50K_20K:
		cc1120_single_write(CC1120_DEVIATION_M, 0x48);    // Frequency Deviation Configuration
		cc1120_single_write(CC1120_MODCFG_DEV_E, 0x05);   // Modulation Format and Frequency Deviation Configur..
		cc1120_single_write(CC1120_IQIC, 0x44);           // Digital Image Channel Compensation Configuration
		cc1120_single_write(CC1120_CHAN_BW, 0x04);        // Channel Filter Configuration
		cc1120_single_write(CC1120_FS_CFG, 0x04);         // Frequency Synthesizer Configuration
		cc1120_single_write(CC1120_FREQ2, 0x6C);          // Frequency Configuration [23:16]
		cc1120_single_write(CC1120_FREQ1, 0x80);          // Frequency Configuration [15:8]
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
		break;

	case CC1120_SET_434_0M_1_2K_2FSK_BW10K_4K:
		cc1120_single_write(CC1120_FS_CFG, 0x04);         // Frequency Synthesizer Configuration
		cc1120_single_write(CC1120_FREQ2, 0x6C);          // Frequency Configuration [23:16]
		cc1120_single_write(CC1120_FREQ1, 0x80);          // Frequency Configuration [15:8]
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
		break;

	case CC1120_SET_434_0M_50K_2GFSK_BW100K_25K:
		cc1120_single_write(CC1120_SYNC_CFG1, 0x08);      // Sync Word Detection Configuration Reg. 1
		cc1120_single_write(CC1120_DEVIATION_M, 0x9A);    // Frequency Deviation Configuration
		cc1120_single_write(CC1120_MODCFG_DEV_E, 0x0D);   // Modulation Format and Frequency Deviation Configur..
		cc1120_single_write(CC1120_IQIC, 0x44);           // Digital Image Channel Compensation Configuration
		cc1120_single_write(CC1120_CHAN_BW, 0x02);        // Channel Filter Configuration
		cc1120_single_write(CC1120_SYMBOL_RATE2, 0x99);   // Symbol Rate Configuration Exponent and Mantissa [1..
		cc1120_single_write(CC1120_SYMBOL_RATE1, 0x99);   // Symbol Rate Configuration Mantissa [15:8]
		cc1120_single_write(CC1120_SYMBOL_RATE0, 0x9A);   // Symbol Rate Configuration Mantissa [7:0]
		cc1120_single_write(CC1120_FS_CFG, 0x04);         // Frequency Synthesizer Configuration
		cc1120_single_write(CC1120_PA_CFG0, 0x7B);        // Power Amplifier Configuration Reg. 0
		cc1120_single_write(CC1120_FREQ2, 0x6C);          // Frequency Configuration [23:16]
		cc1120_single_write(CC1120_FREQ1, 0x80);          // Frequency Configuration [15:8]
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
		break;

	case CC1120_SET_434_0M_100K_4FSK_BW100K_25K:
		cc1120_single_write(CC1120_SYNC_CFG1, 0x0B);      // Sync Word Detection Configuration Reg. 1
		cc1120_single_write(CC1120_DEVIATION_M, 0x9A);    // Frequency Deviation Configuration
		cc1120_single_write(CC1120_MODCFG_DEV_E, 0x25);   // Modulation Format and Frequency Deviation Configur..
		cc1120_single_write(CC1120_IQIC, 0x44);           // Digital Image Channel Compensation Configuration
		cc1120_single_write(CC1120_CHAN_BW, 0x02);        // Channel Filter Configuration
		cc1120_single_write(CC1120_SYMBOL_RATE2, 0x99);   // Symbol Rate Configuration Exponent and Mantissa [1..
		cc1120_single_write(CC1120_SYMBOL_RATE1, 0x99);   // Symbol Rate Configuration Mantissa [15:8]
		cc1120_single_write(CC1120_SYMBOL_RATE0, 0x9A);   // Symbol Rate Configuration Mantissa [7:0]
		cc1120_single_write(CC1120_FS_CFG, 0x04);         // Frequency Synthesizer Configuration
		cc1120_single_write(CC1120_PA_CFG0, 0x7B);        // Power Amplifier Configuration Reg. 0
		cc1120_single_write(CC1120_FREQ2, 0x6C);          // Frequency Configuration [23:16]
		cc1120_single_write(CC1120_FREQ1, 0x80);          // Frequency Configuration [15:8]
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
		break;

	case CC1120_SET_434_0M_4_8K_2FSK_BW40K_9K:
		cc1120_single_write(CC1120_SYNC_CFG1, 0x08);      // Sync Word Detection Configuration Reg. 1
		cc1120_single_write(CC1120_DEVIATION_M, 0x27);    // Frequency Deviation Configuration
		cc1120_single_write(CC1120_MODCFG_DEV_E, 0x0C);   // Modulation Format and Frequency Deviation Configur..
		cc1120_single_write(CC1120_IQIC, 0x44);           // Digital Image Channel Compensation Configuration
		cc1120_single_write(CC1120_CHAN_BW, 0x05);        // Channel Filter Configuration
		cc1120_single_write(CC1120_SYMBOL_RATE2, 0x63);   // Symbol Rate Configuration Exponent and Mantissa [1..
		cc1120_single_write(CC1120_FS_CFG, 0x04);         // Frequency Synthesizer Configuration
		cc1120_single_write(CC1120_PA_CFG0, 0x7E);        // Power Amplifier Configuration Reg. 0
		cc1120_single_write(CC1120_FREQ2, 0x6C);          // Frequency Configuration [23:16]
		cc1120_single_write(CC1120_FREQ1, 0x80);          // Frequency Configuration [15:8]
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
		break;

	case CC1120_SET_434_0M_4_8K_2FSK_BW50K_14K:
		cc1120_single_write(CC1120_SYNC_CFG1, 0x08);      // Sync Word Detection Configuration Reg. 1
		cc1120_single_write(CC1120_DEVIATION_M, 0xCB);    // Frequency Deviation Configuration
		cc1120_single_write(CC1120_MODCFG_DEV_E, 0x0C);   // Modulation Format and Frequency Deviation Configur..
		cc1120_single_write(CC1120_IQIC, 0x44);           // Digital Image Channel Compensation Configuration
		cc1120_single_write(CC1120_CHAN_BW, 0x04);        // Channel Filter Configuration
		cc1120_single_write(CC1120_SYMBOL_RATE2, 0x63);   // Symbol Rate Configuration Exponent and Mantissa [1..
		cc1120_single_write(CC1120_FS_CFG, 0x04);         // Frequency Synthesizer Configuration
		cc1120_single_write(CC1120_PA_CFG0, 0x7E);        // Power Amplifier Configuration Reg. 0
		cc1120_single_write(CC1120_FREQ2, 0x6C);          // Frequency Configuration [23:16]
		cc1120_single_write(CC1120_FREQ1, 0x80);          // Frequency Configuration [15:8]
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
		break;

	case CC1120_SET_434_0M_4_8K_2FSK_BW100K_39K:
		cc1120_single_write(CC1120_SYNC_CFG1, 0x08);      // Sync Word Detection Configuration Reg. 1
		cc1120_single_write(CC1120_DEVIATION_M, 0x3F);    // Frequency Deviation Configuration
		cc1120_single_write(CC1120_MODCFG_DEV_E, 0x0E);   // Modulation Format and Frequency Deviation Configur..
		cc1120_single_write(CC1120_IQIC, 0x44);           // Digital Image Channel Compensation Configuration
		cc1120_single_write(CC1120_CHAN_BW, 0x02);        // Channel Filter Configuration
		cc1120_single_write(CC1120_SYMBOL_RATE2, 0x63);   // Symbol Rate Configuration Exponent and Mantissa [1..
		cc1120_single_write(CC1120_FS_CFG, 0x04);         // Frequency Synthesizer Configuration
		cc1120_single_write(CC1120_PA_CFG0, 0x7E);        // Power Amplifier Configuration Reg. 0
		cc1120_single_write(CC1120_FREQ2, 0x6C);          // Frequency Configuration [23:16]
		cc1120_single_write(CC1120_FREQ1, 0x80);          // Frequency Configuration [15:8]
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
		break;

	case CC1120_SET_434_0M_9_6K_2FSK_BW50K_12K:
		cc1120_single_write(CC1120_DEVIATION_M, 0x89);    // Frequency Deviation Configuration
		cc1120_single_write(CC1120_MODCFG_DEV_E, 0x04);   // Modulation Format and Frequency Deviation Configur..
		cc1120_single_write(CC1120_IQIC, 0x44);           // Digital Image Channel Compensation Configuration
		cc1120_single_write(CC1120_CHAN_BW, 0x04);        // Channel Filter Configuration
		cc1120_single_write(CC1120_SYMBOL_RATE2, 0x73);   // Symbol Rate Configuration Exponent and Mantissa [1..
		cc1120_single_write(CC1120_FS_CFG, 0x04);         // Frequency Synthesizer Configuration
		cc1120_single_write(CC1120_PA_CFG0, 0x7D);        // Power Amplifier Configuration Reg. 0
		cc1120_single_write(CC1120_FREQ2, 0x6C);          // Frequency Configuration [23:16]
		cc1120_single_write(CC1120_FREQ1, 0x80);          // Frequency Configuration [15:8]
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
		break;

	case CC1120_SET_452_0M_9_6K_2GFSK_BW33K_2_4K:
		cc1120_single_write(CC1120_SYNC_CFG1, 0x08);      // Sync Word Detection Configuration Reg. 1
		cc1120_single_write(CC1120_DEVIATION_M, 0x3A);    // Frequency Deviation Configuration
		cc1120_single_write(CC1120_MODCFG_DEV_E, 0x0A);   // Modulation Format and Frequency Deviation Configur..
		cc1120_single_write(CC1120_IQIC, 0x44);           // Digital Image Channel Compensation Configuration
		cc1120_single_write(CC1120_CHAN_BW, 0x06);        // Channel Filter Configuration
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
		break;

	case CC1120_SET_452_0M_9_6K_2GFSK_BW50K_2_4K:
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
		break;

	default:
		// CC1120_SET_434_0M_4_8K_2GFSK_BW40K_9K
		cc1120_single_write(CC1120_SYNC_CFG1, 0x08);      // Sync Word Detection Configuration Reg. 1
		cc1120_single_write(CC1120_DEVIATION_M, 0x27);    // Frequency Deviation Configuration
		cc1120_single_write(CC1120_MODCFG_DEV_E, 0x0C);   // Modulation Format and Frequency Deviation Configur..
		cc1120_single_write(CC1120_IQIC, 0x44);           // Digital Image Channel Compensation Configuration
		cc1120_single_write(CC1120_CHAN_BW, 0x05);        // Channel Filter Configuration
		cc1120_single_write(CC1120_SYMBOL_RATE2, 0x63);   // Symbol Rate Configuration Exponent and Mantissa [1..
		cc1120_single_write(CC1120_FS_CFG, 0x04);         // Frequency Synthesizer Configuration
		cc1120_single_write(CC1120_PA_CFG0, 0x7E);        // Power Amplifier Configuration Reg. 0
		cc1120_single_write(CC1120_FREQ2, 0x6C);          // Frequency Configuration [23:16]
		cc1120_single_write(CC1120_FREQ1, 0x80);          // Frequency Configuration [15:8]
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
		break;
	}

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

	cc1120_single_write(CC1120_PKT_LEN, (len + 2) % 256);

	// The length is the first two bytes
	uint8_t buf_len[2];
	buf_len[0] = (len >> 8) & 0xFF;
	buf_len[1] = len & 0xFF;
	cc1120_burst_write(CC1120_TXFIFO, buf_len, 2);

	// Write a few bytes and start transmitting before writing the rest to minimize delay.
	i = MIN(len, 8);
	cc1120_burst_write(CC1120_TXFIFO, data, i);
	cc1120_strobe(CC1120_STX);

	bool end_written = false;

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
				// These bits should tell how much space there is left in the FIFO, but there is a silicon issue that
				// can corrupt them so they are reserved in the datasheet. We are using them anyway.

				spi_disable();
				spiReleaseBus(&CC1120_SPI);

				int to = 1000;
				while ((cc1120_single_read(CC1120_NUM_TXBYTES) > 60) && to > 0) {
					chThdSleepMilliseconds(1);
					to--;
				}

				if(cc1120_state() == CC1120_STATUS_STATE_TXFIFO_UNDERFLOW) {
					// TX FIFO underflow.
					cc1120_strobe(CC1120_SFTX);
					cc1120_strobe(CC1120_SRX);
					break;
				}

				spiAcquireBus(&CC1120_SPI);
				spi_enable();
				spi_exchange(CC1120_TXFIFO | 0x40);
			}

			if (((len + 2) - i) < 100 && !end_written) {
				spi_disable();
				spiReleaseBus(&CC1120_SPI);

				cc1120_single_write(CC1120_PKT_CFG0, PKT_CFG0_LENGTH_CONFIG_FIXED);
				end_written = true;

				spiAcquireBus(&CC1120_SPI);
				spi_enable();
				spi_exchange(CC1120_TXFIFO | 0x40);
			}
		}

		spi_disable();
		spiReleaseBus(&CC1120_SPI);
	} else {
		cc1120_single_write(CC1120_PKT_CFG0, PKT_CFG0_LENGTH_CONFIG_FIXED);
	}
}

void cc1120_check_txfifo(void) {
	if(cc1120_state() == CC1120_STATUS_STATE_TXFIFO_UNDERFLOW) {
		// Acknowledge TX FIFO underflow.
		cc1120_strobe(CC1120_SFTX);
		cc1120_strobe(CC1120_SRX);
	}
}

void cc1120_flushrx(void) {
	if(cc1120_state() == CC1120_STATE_RXFIFO_OVERFLOW) {
		cc1120_strobe(CC1120_SFRX);
	}

	cc1120_strobe(CC1120_SIDLE);

	int to = 100;
	while (!(cc1120_state() == CC1120_STATE_IDLE) && to > 0) {
		chThdSleepMilliseconds(1);
		to--;
	}

	cc1120_strobe(CC1120_SFRX);
	cc1120_strobe(CC1120_SRX);
}

int cc1120_transmit(uint8_t *data, int len) {
	if (!m_init_done) {
		return -4;
	}

	// Wait for ongoing reception
	int to = 2000;
	while (m_rx_pos && to > 0) {
		chThdSleepMilliseconds(1);
		to--;
	}

	chMtxLock(&m_radio_mutex);

	if(cc1120_state() == CC1120_STATE_RXFIFO_OVERFLOW) {
		cc1120_flushrx();
	}

	if(len > CC1120_MAX_PAYLOAD) {
		chMtxUnlock(&m_radio_mutex);
		return -1;
	}

	cc1120_strobe(CC1120_SIDLE);
	m_rx_pos = 0;
	cc1120_write_txfifo(data, len);

	to = 1000;
	while (cc1120_state() != CC1120_STATE_TX && to > 0) {
		chThdSleepMilliseconds(1);
		to--;
	}

	if(cc1120_state() != CC1120_STATE_TX) {
		cc1120_check_txfifo();
		cc1120_flushrx();
		cc1120_single_write(CC1120_PKT_CFG0, PKT_CFG0_LENGTH_CONFIG_INFINITE);
		cc1120_single_write(CC1120_FIFO_CFG, 2);
		chMtxUnlock(&m_radio_mutex);
		return -2;
	}

	to = 1000;
	while (cc1120_state() == CC1120_STATE_TX && to > 0) {
		chThdSleepMilliseconds(1);
		to--;
	}

	if(cc1120_state() == CC1120_STATE_TX) {
		cc1120_check_txfifo();
		cc1120_flushrx();
		cc1120_single_write(CC1120_PKT_CFG0, PKT_CFG0_LENGTH_CONFIG_INFINITE);
		cc1120_single_write(CC1120_FIFO_CFG, 2);
		chMtxUnlock(&m_radio_mutex);
		return -3;
	}

	cc1120_check_txfifo();
	cc1120_single_write(CC1120_PKT_CFG0, PKT_CFG0_LENGTH_CONFIG_INFINITE);
	cc1120_single_write(CC1120_FIFO_CFG, 2);

	chMtxUnlock(&m_radio_mutex);
	return 1;
}

int cc1120_on(void) {
	cc1120_flushrx();
	cc1120_strobe(CC1120_SRX);
	cc1120_single_write(CC1120_PKT_CFG0, PKT_CFG0_LENGTH_CONFIG_INFINITE);
	cc1120_single_write(CC1120_FIFO_CFG, 2);
	m_rx_pos = 0;
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

int cc1120_set_idle(void) {
	cc1120_strobe(CC1120_SIDLE);

	int to = 100;
	while (!(cc1120_state() == CC1120_STATE_IDLE) && to > 0) {
		chThdSleepMilliseconds(1);
		to--;
	}

	return to > 0 ? 1 : 0;
}

bool cc1120_carrier_sense(void) {
//	uint8_t reg = cc1120_single_read(CC1120_RSSI0);
//	return ((reg & 0b010) && (reg & 0b100)) ? 1 : 0;
	return palReadPad(CC1120_PORT_GPIO3, CC1120_PIN_GPIO3);
}

float cc1120_read_freqoff_est(void) {
	uint8_t fs_cfg = cc1120_single_read(CC1120_FS_CFG);
	uint8_t fo_est_hi = cc1120_single_read(CC1120_FREQOFF_EST1);
	uint8_t fo_est_lo = cc1120_single_read(CC1120_FREQOFF_EST0);

	float fo_est = (float)((int16_t)((uint16_t)fo_est_hi << 8 | (uint16_t)fo_est_lo));
	float lo_div = (float)((fs_cfg & 0x0F) * 2);

	return fo_est * 32e6 / lo_div / 262144.0;
}

float cc1120_read_freqoff(void) {
	uint8_t fs_cfg = cc1120_single_read(CC1120_FS_CFG);
	uint8_t fo_est_hi = cc1120_single_read(CC1120_FREQOFF1);
	uint8_t fo_est_lo = cc1120_single_read(CC1120_FREQOFF0);

	float fo_est = (float)((int16_t)((uint16_t)fo_est_hi << 8 | (uint16_t)fo_est_lo));
	float lo_div = (float)((fs_cfg & 0x0F) * 2);

	return fo_est * 32e6 / lo_div / 262144.0;
}

float cc1120_get_last_freqoff_est(void) {
	return m_last_freqoff_est;
}

void cc1120_ext_cb(EXTDriver *extp, expchannel_t channel) {
	(void)extp;
	(void)channel;

	chSysLockFromISR();
	chEvtSignalI(isr_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

void cc1120_set_rx_callback(void(*cb)(uint8_t *data, int len, int rssi, int lqi, bool crc_ok)) {
	rx_callback = cb;
}

static THD_FUNCTION(isr_thread, arg) {
	(void) arg;
	chRegSetThreadName("CC1120 EXTI");

	isr_tp = chThdGetSelfX();

	for (;;) {
		chEvtWaitAnyTimeout((eventmask_t) 1, MS2ST(100));

		while (palReadPad(CC1120_PORT_GPIO0, CC1120_PIN_GPIO0)) {
			interrupt();
		}
	}
}

static THD_FUNCTION(check_thread, arg) {
	(void) arg;
	chRegSetThreadName("CC1120 Check");

	// Note: this thread has low stack size, so prints cannot be done here
	// without increasing the stack.

	for (;;) {
		chMtxLock(&m_radio_mutex);

		uint8_t s = cc1120_state();

		if(s == CC1120_STATE_RXFIFO_OVERFLOW) {
			cc1120_flushrx();
			cc1120_single_write(CC1120_PKT_CFG0, PKT_CFG0_LENGTH_CONFIG_INFINITE);
			cc1120_single_write(CC1120_FIFO_CFG, 2);
			m_rx_pos = 0;
		}

		chMtxUnlock(&m_radio_mutex);

		chThdSleepMilliseconds(1000);
	}
}

// Private functions

static int interrupt(void) {
	uint8_t rxbytes;

	chMtxLock(&m_radio_mutex);

	do {
		rxbytes = cc1120_single_read(CC1120_NUM_RXBYTES);

		if(rxbytes == 0) {
			chMtxUnlock(&m_radio_mutex);
			return 1;
		}

		// Read frequency offset estimation
		if (m_rx_pos == 0) {
			m_last_freqoff_est = cc1120_read_freqoff_est();
//			cc1120_strobe(CC1120_SAFC); // Apply frequency offset estimation
			// TODO: find a good place to apply FOC. Doing it here seems to
			// detune the filter so much on certain packets that reception stops working.
		}

		cc1120_burst_read(CC1120_RXFIFO, m_rx_buffer + m_rx_pos, rxbytes);
		m_rx_pos += rxbytes;

		int rx_len = 0;
		if (m_rx_pos >= 2) {
			rx_len = (uint16_t)m_rx_buffer[0] << 8 | (uint16_t)m_rx_buffer[1];
			rx_len += 2;
		}

		if (rx_len > 0) {
			if((rx_len - 2) > CC1120_MAX_PAYLOAD) {
				cc1120_flushrx();
				cc1120_single_write(CC1120_PKT_CFG0, PKT_CFG0_LENGTH_CONFIG_INFINITE);
				cc1120_single_write(CC1120_FIFO_CFG, 2);
				m_rx_pos = 0;
				chMtxUnlock(&m_radio_mutex);
				return -1;
			} else {
				cc1120_single_write(CC1120_FIFO_CFG, 25);
			}

			// Switch to fixed length mode and set the remaining length if less than 200 bytes are left
			if ((rx_len - m_rx_pos) < 200) {
				cc1120_single_write(CC1120_PKT_LEN, rx_len % 256);
				cc1120_single_write(CC1120_PKT_CFG0, PKT_CFG0_LENGTH_CONFIG_FIXED);
			}

			// The whole packet is received
			if (m_rx_pos >= (rx_len + 2)) { // +2 is for the status bytes
				cc1120_flushrx();
				cc1120_single_write(CC1120_PKT_CFG0, PKT_CFG0_LENGTH_CONFIG_INFINITE);
				cc1120_single_write(CC1120_FIFO_CFG, 2);
				m_rx_pos = 0;

				chMtxUnlock(&m_radio_mutex);

				int rssi = (int8_t)m_rx_buffer[rx_len];
				int lqi = m_rx_buffer[rx_len + 1] & 0x7F;
				bool crc_ok = m_rx_buffer[rx_len + 1] & 0x80;
				int len = rx_len - 2;

				if (rx_callback) {
					rx_callback(m_rx_buffer + 2, len, rssi, lqi, crc_ok);
				}

				return 1;
			}
		}

		rxbytes = cc1120_single_read(CC1120_NUM_RXBYTES);
	} while(rxbytes);

	chMtxUnlock(&m_radio_mutex);
	return 1;
}

static void spi_enable(void) {
	palClearPad(CC1120_PORT_CS, CC1120_PIN_CS);

	int timeout = 100;
	while (palReadPad(CC1120_PORT_MISO, CC1120_PIN_MISO)) {
		chThdSleepMilliseconds(1);
		timeout--;
		if (!timeout) {
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
#define VCDAC_START_OFFSET	2
#define FS_VCO2_INDEX		0
#define FS_VCO4_INDEX		1
#define FS_CHP_INDEX		2

static void calibrate_manual(void) {
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
	cc1120_burst_read(CC1120_FS_VCO2, &calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
	cc1120_burst_read(CC1120_FS_VCO4, &calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
	cc1120_burst_read(CC1120_FS_CHP, &calResults_for_vcdac_start_high[FS_CHP_INDEX], 1);

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
	cc1120_burst_read(CC1120_FS_VCO2, &calResults_for_vcdac_start_mid[FS_VCO2_INDEX], 1);
	cc1120_burst_read(CC1120_FS_VCO4, &calResults_for_vcdac_start_mid[FS_VCO4_INDEX], 1);
	cc1120_burst_read(CC1120_FS_CHP, &calResults_for_vcdac_start_mid[FS_CHP_INDEX], 1);

	// 9) Write back highest FS_VCO2 and corresponding FS_VCO and FS_CHP result
	if(calResults_for_vcdac_start_high[FS_VCO2_INDEX] > calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
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
