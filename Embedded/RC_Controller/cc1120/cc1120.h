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

#ifndef CC1120_CC1120_H_
#define CC1120_CC1120_H_

#include "conf_general.h"
#include "ch.h"
#include "hal.h"

// Functions
void cc1120_init(void);
uint8_t cc1120_state(void);
char *cc1120_state_name(void);
unsigned char cc1120_strobe(uint8_t strobe);
unsigned char cc1120_single_read(uint16_t addr);
uint8_t cc1120_single_write(uint16_t addr, uint8_t value);
void cc1120_burst_read(uint16_t addr, uint8_t *buffer, uint8_t count);
void cc1120_burst_write(uint16_t addr, uint8_t *buffer, uint8_t count);
void cc1120_write_txfifo(uint8_t *data, int len);
void cc1120_check_txfifo(void);
void cc1120_flushrx(void);
int cc1120_transmit(uint8_t *data, int len);
int cc1120_on(void);
int cc1120_off(void);
bool cc1120_carrier_sense(void);
void cc1120_ext_cb(EXTDriver *extp, expchannel_t channel);
void cc1120_calibrate_manual(void);

#ifndef BV
#define BV(n)      (1 << (n))
#endif

// Note: Many of the definitions below were taken from the contiki cc1120 driver.

// ================ Configuration registers ================ //

#define CC1120_IOCFG3						0x00
#define CC1120_IOCFG2						0x01
#define CC1120_IOCFG1						0x02
#define CC1120_IOCFG0						0x03
#define CC1120_SYNC3						0x04
#define CC1120_SYNC2						0x05
#define CC1120_SYNC1						0x06
#define CC1120_SYNC0						0x07
#define CC1120_SYNC_CFG1					0x08
#define CC1120_SYNC_CFG0					0x09
#define CC1120_DEVIATION_M					0x0A
#define CC1120_MODCFG_DEV_E					0x0B
#define CC1120_DCFILT_CFG					0x0C
#define CC1120_PREAMBLE_CFG1				0x0D
#define CC1120_PREAMBLE_CFG0				0x0E
#define CC1120_FREQ_IF_CFG					0x0F
#define CC1120_IQIC							0x10
#define CC1120_CHAN_BW						0x11
#define CC1120_MDMCFG1						0x12
#define CC1120_MDMCFG0						0x13
#define CC1120_SYMBOL_RATE2					0x14
#define CC1120_SYMBOL_RATE1					0x15
#define CC1120_SYMBOL_RATE0					0x16
#define CC1120_AGC_REF						0x17
#define CC1120_AGC_CS_THR					0x18
#define CC1120_AGC_GAIN_ADJUST				0x19
#define CC1120_AGC_CFG3						0x1A
#define CC1120_AGC_CFG2						0x1B
#define CC1120_AGC_CFG1						0x1C
#define CC1120_AGC_CFG0						0x1D
#define CC1120_FIFO_CFG						0x1E
#define CC1120_DEV_ADDR						0x1F
#define CC1120_SETTLING_CFG					0x20
#define CC1120_FS_CFG						0x21
#define CC1120_WOR_CFG1						0x22
#define CC1120_WOR_CFG0						0x23
#define CC1120_WOR_EVENT0_MSB				0x24
#define CC1120_WOR_EVENT0_LSB				0x25
#define CC1120_PKT_CFG2						0x26
#define CC1120_PKT_CFG1						0x27
#define CC1120_PKT_CFG0						0x28
#define CC1120_RFEND_CFG1					0x29
#define CC1120_RFEND_CFG0					0x2A
#define CC1120_PA_CFG2						0x2B
#define CC1120_PA_CFG1						0x2C
#define CC1120_PA_CFG0						0x2D
#define CC1120_PKT_LEN						0x2E
#define CC1120_EXTENDED_MEMORY_ACCESS		0x2F

// ================ Status registers ================ //

// Extended register space, accessed via CC1120_EXTENDED_MEMORY_ACCESS
// NOTE: Below addresses have been increased by 0x2F00, to allow detection of extended address space registers.

#define CC1120_IF_MIX_CFG					0x2F00
#define CC1120_FREQOFF_CFG					0x2F01
#define CC1120_TOC_CFG						0x2F02
#define CC1120_MARC_SPARE					0x2F03
#define CC1120_ECG_CFG						0x2F04
#define CC1120_SOFT_TX_DATA_CFG				0x2F05
#define CC1120_EXT_CTRL						0x2F06
#define CC1120_RCCAL_FINE					0x2F07
#define CC1120_RCCAL_COARSE					0x2F08
#define CC1120_RCCAL_OFFSET					0x2F09
#define CC1120_FREQOFF1						0x2F0A
#define CC1120_FREQOFF0						0x2F0B
#define CC1120_FREQ2						0x2F0C
#define CC1120_FREQ1						0x2F0D
#define CC1120_FREQ0						0x2F0E
#define CC1120_IF_ADC2						0x2F0F
#define CC1120_IF_ADC1						0x2F10
#define CC1120_IF_ADC0						0x2F11
#define CC1120_FS_DIG1						0x2F12
#define CC1120_FS_DIG0						0x2F13
#define CC1120_FS_CAL3						0x2F14
#define CC1120_FS_CAL2						0x2F15
#define CC1120_FS_CAL1						0x2F16
#define CC1120_FS_CAL0						0x2F17
#define CC1120_FS_CHP						0x2F18
#define CC1120_FS_DIVTWO					0x2F19
#define CC1120_FS_DSM1						0x2F1A
#define CC1120_FS_DSM0						0x2F1B
#define CC1120_FS_DVC1						0x2F1C
#define CC1120_FS_DVC0						0x2F1D
#define CC1120_FS_LBI						0x2F1E
#define CC1120_FS_PFD						0x2F1F
#define CC1120_FS_PRE						0x2F20
#define CC1120_FS_REG_DIV_CML				0x2F21
#define CC1120_FS_SPARE						0x2F22
#define CC1120_FS_VCO4						0x2F23

#define CC1120_FS_VCO3						0x2F24
#define CC1120_FS_VCO2						0x2F25
#define CC1120_FS_VCO1						0x2F26
#define CC1120_FS_VCO0						0x2F27
#define CC1120_GBIAS6						0x2F28
#define CC1120_GBIAS5						0x2F29
#define CC1120_GBIAS4						0x2F2A
#define CC1120_GBIAS3						0x2F2B
#define CC1120_GBIAS2						0x2F2C
#define CC1120_GBIAS1						0x2F2D
#define CC1120_GBIAS0						0x2F2E
#define CC1120_IFAMP						0x2F2F
#define CC1120_LNA							0x2F30
#define CC1120_RXMIX						0x2F31
#define CC1120_XOSC5						0x2F32
#define CC1120_XOSC4						0x2F33
#define CC1120_XOSC3						0x2F34
#define CC1120_XOSC2						0x2F35
#define CC1120_XOSC1						0x2F36
#define CC1120_XOSC0						0x2F37
#define CC1120_ANALOG_SPARE					0x2F38
#define CC1120_PA_CFG3						0x2F39
#define CC1120_WOR_TIME1					0x2F64
#define CC1120_WOR_TIME0					0x2F65
#define CC1120_WOR_CAPTURE1					0x2F66
#define CC1120_WOR_CAPTURE0					0x2F67
#define CC1120_BIST							0x2F68
#define CC1120_DCFILTOFFSET_I1				0x2F69
#define CC1120_DCFILTOFFSET_I0				0x2F6A
#define CC1120_DCFILTOFFSET_Q1				0x2F6B
#define CC1120_DCFILTOFFSET_Q0				0x2F6C
#define CC1120_IQIE_I1						0x2F6D
#define CC1120_IQIE_I0						0x2F6E

#define CC1120_IQIE_Q1						0x2F6F
#define CC1120_IQIE_Q0						0x2F70
#define CC1120_RSSI1						0x2F71
#define CC1120_RSSI0						0x2F72
#define CC1120_MARCSTATE					0x2F73
#define CC1120_LQI_VAL						0x2F74
#define CC1120_PQT_SYNC_ERR					0x2F75
#define CC1120_DEM_STATUS					0x2F76
#define CC1120_FREQOFF_EST1					0x2F77
#define CC1120_FREQOFF_EST0					0x2F78
#define CC1120_AGC_GAIN3					0x2F79
#define CC1120_AGC_GAIN2					0x2F7A
#define CC1120_AGC_GAIN1					0x2F7B
#define CC1120_AGC_GAIN0					0x2F7C
#define CC1120_SOFT_RX_DATA_OUT				0x2F7D
#define CC1120_SOFT_TX_DATA_IN				0x2F7E
#define CC1120_ASK_SOFT_RX_DATA				0x2F7F
#define CC1120_RNDGEN						0x2F80
#define CC1120_MAGN2						0x2F81
#define CC1120_MAGN1						0x2F82
#define CC1120_MAGN0						0x2F83
#define CC1120_ANG1							0x2F84
#define CC1120_ANG0							0x2F85
#define CC1120_CHFILT_I2					0x2F86
#define CC1120_CHFILT_I1					0x2F87
#define CC1120_CHFILT_I0					0x2F88
#define CC1120_CHFILT_Q2					0x2F89
#define CC1120_CHFILT_Q1					0x2F8A
#define CC1120_CHFILT_Q0					0x2F8B
#define CC1120_GPIO_STATUS					0x2F8C

#define CC1120_FSCAL_CTRL					0x2F8D
#define CC1120_PHASE_ADJUST					0x2F8E
#define CC1120_PARTNUMBER					0x2F8F
#define CC1120_PARTVERSION					0x2F90
#define CC1120_SERIAL_STATUS				0x2F91
#define CC1120_RX_STATUS					0x2F92
#define CC1120_TX_STATUS					0x2F93
#define CC1120_MARC_STATUS1					0x2F94
#define CC1120_MARC_STATUS0					0x2F95
#define CC1120_PA_IFAMP_TEST				0x2F96
#define CC1120_FSRF_TEST					0x2F97
#define CC1120_PRE_TEST						0x2F98
#define CC1120_PRE_OVR						0x2F99
#define CC1120_ADC_TEST						0x2F9A
#define CC1120_DVC_TEST						0x2F9B
#define CC1120_ATEST						0x2F9C
#define CC1120_ATEST_LVDS					0x2F9D
#define CC1120_ATEST_MODE					0x2F9E
#define CC1120_XOSC_TEST1					0x2F9F
#define CC1120_XOSC_TEST0					0x2FA0
#define CC1120_RXFIRST						0x2FD2
#define CC1120_TXFIRST						0x2FD3
#define CC1120_RXLAST						0x2FD4
#define CC1120_TXLAST						0x2FD5
#define CC1120_NUM_TXBYTES					0x2FD6
#define CC1120_NUM_RXBYTES					0x2FD7
#define CC1120_FIFO_NUM_TXBYTES				0x2FD8
#define CC1120_FIFO_NUM_RXBYTES				0x2FD9

#define CC1120_TXFIFO						0x3F
#define CC1120_RXFIFO						0x3F

#define CC1120_STATE_SLEEP					0
#define CC1120_STATE_IDLE					1
#define CC1120_STATE_XOFF					2
#define CC1120_STATE_VCOON_MC				3
#define CC1120_STATE_REGON_MC				4
#define CC1120_STATE_MANCAL					5
#define CC1120_STATE_VCOON					6
#define CC1120_STATE_REGON					7
#define CC1120_STATE_STARTCAL				8
#define CC1120_STATE_BWBOOST				9
#define CC1120_STATE_FS_LOCK				10
#define CC1120_STATE_IFADCON				11
#define CC1120_STATE_ENDCAL					12
#define CC1120_STATE_RX						13
#define CC1120_STATE_RX_END					14
#define CC1120_STATE_RX_RST					15
#define CC1120_STATE_TXRX_SWITCH			16
#define CC1120_STATE_RXFIFO_OVERFLOW		17
#define CC1120_STATE_FSTXON					18
#define CC1120_STATE_TX						19
#define CC1120_STATE_TX_END					20
#define CC1120_STATE_RXTX_SWITCH			21
#define CC1120_STATE_TXFIFO_UNDERFLOW		22

// ================ Strobe commands ================ //

// Reset chip.
#define CC1120_SRES							0x30

// Enable and calibrate frequency synthesizer (if SETTLING_CFG.FS_AUTOCAL=1).
// If in RX/TX: Go to a wait state where only the synthesizer is
// running (for quick RX / TX turnaround).
#define CC1120_SFSTXON						0x31

// Turn off crystal oscillator.
#define CC1120_SXOFF						0x32

// Calibrate frequency synthesizer and turn it off (enables quick start).
#define CC1120_SCAL							0x33

// Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.
#define CC1120_SRX							0x34

// In IDLE state: Enable TX. Perform calibration first if
// MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
// Only go to TX if channel is clear.
#define CC1120_STX							0x35

// Exit RX / TX, turn off frequency synthesizer and exit
// Wake-On-Radio mode if applicable.
#define CC1120_SIDLE						0x36

// Perform AFC adjustment of the frequency synthesizer
#define CC1120_SAFC							0x37

// Start automatic RX polling sequence (Wake-on-Radio)
#define CC1120_SWOR							0x38

// Enter power down mode when CSn goes high.
#define CC1120_SPWD							0x39

// Flush the RX FIFO buffer.
#define CC1120_SFRX							0x3A

// Flush the TX FIFO buffer.
#define CC1120_SFTX							0x3B

// Reset real time clock.
#define CC1120_SWORRST						0x3C

// No operation. May be used to pad strobe commands to two
// bytes for simpler software.
#define CC1120_SNOP							0x3D

// ================ Settings ================ //

// IOCFG
#define IOCFG_GPIO_CFG_INVERT				BV(6) // invert assert/de-assert
#define IOCFG_GPIO_CFG_ATRAN				BV(7) // set pin as "Analog transfer" (==pin not used as GPIO)
#define IOCFG_GPIO_CFG_RXFIFO_THR			0 // Asserted as long as length > THR
#define IOCFG_GPIO_CFG_RXFIFO_THR_PKT		1 // Assert when THR is reached or EOP; de-assert on empty RxFIFO
#define IOCFG_GPIO_CFG_PKT_SYNC_RXTX		6 // assert on SYNC recv/sent, de-assert on EOP
#define IOCFG_GPIO_CFG_CS_VALID				16 // CS valid ? assert : de-assert
#define IOCFG_GPIO_CFG_CS					17 // Carrier Sense ? assert : de-assert
#define IOCFG_GPIO_CFG_RXIDLE_OR_TX			26 // assert when in TX, de-assert in Rx/IDLE/settling
#define IOCFG_GPIO_CFG_RXTX_OR_IDLE			38 // assert if in rx or tx, de-assert if idle/settling (MARC_2PIN_STATUS[0])
#define IOCFG_GPIO_CFG_HIGHZ				48 // high impedance

// PKT_CFG_0
#define PKT_CFG0_LENGTH_CONFIG_FIXED		(0 << 5) // Fixed length packets
#define PKT_CFG0_LENGTH_CONFIG_VARIABLE		(1 << 5) // Variable length packets
#define PKT_CFG0_LENGTH_CONFIG_INFINITE		(2 << 5) // Infinite length packets

// PKT_CFG_1
#define PKT_CFG1_WHITE_DATA					(1 << 7) // Data whitening enabled
#define PKT_CFG1_BYTE_SWAP					(1 << 1) // Swap all bytes
#define PKT_CFG1_APPEND_STATUS				(1 << 0) // Append status bytes to end of RX FIFO with RSSI and CRC check info
#define PKT_CFG1_ADDR_CHECK_OFF				(0 << 4) // No address check
#define PKT_CFG1_ADDR_CHECK_ON				(1 << 4) // Address check on, no broadcast
#define PKT_CFG1_ADDR_CHECK_ON_BR00			(2 << 4) // Address check on, 0x00 = broadcast
#define PKT_CFG1_ADDR_CHECK_ON_BR00_BRFF	(3 << 4) // Address check on, 0x00 and 0xFF = broadcast
#define PKT_CFG1_CRC_OFF					(0 << 2) // No CRC
#define PKT_CFG1_CRC_ON_1					(1 << 2) // RX and TX CRC with CRC16(X16+X15+X2+1), Initialized to 0xFFFF
#define PKT_CFG1_CRC_ON_2					(2 << 2) // RX and TX CRC with CRC16(X16+X12+X5+1), Initialized to 0x0000

// RFEND_CFG0
#define RFEND_CFG0_TXOFF_MODE_RETURN_TO_RX	(BV(4) | BV(5)) // Return to RX after TX

// RFEND_CFG1
#define RFEND_CFG1_RXOFF_MODE_RETURN_TO_RX	(BV(4) | BV(5)) // Return to RX after RX

// SYNC_CFG0
#define SYNC_CFG0_NOSYNC					0
#define SYNC_CFG0_11_BITS					(1 << 2)
#define SYNC_CFG0_16_BITS					(2 << 2)
#define SYNC_CFG0_18_BITS					(3 << 2)
#define SYNC_CFG0_24_BITS					(4 << 2)
#define SYNC_CFG0_32_BITS					(5 << 2)
#define SYNC_CFG0_NUM_ERROR_ENABLED			0
#define SYNC_CFG0_NUM_ERROR_DISABLED		3

// PREAMBLE_CFG0
#define PREAMBLE_CFG0_PQT_EN				(1 << 5) // Enable preamble detection
#define PREAMBLE_CFG0_TIMEOUT_16			(0 << 4) // 16 symbols required to consider preamble valid
#define PREAMBLE_CFG0_TIMEOUT_43			(1 << 4) // 43 symbols required to consider preamble valid

// PREAMBLE_CFG1
#define PREAMBLE_CFG1_WORD_AA				(0 << 0) // Use 0xAA (10101010) as preamble
#define PREAMBLE_CFG1_WORD_55				(1 << 0) // Use 0x55 (01010101) as preamble
#define PREAMBLE_CFG1_WORD_33				(2 << 0) // Use 0x33 (00110011) as preamble
#define PREAMBLE_CFG1_WORD_CC				(3 << 0) // Use 0xCC (11001100) as preamble
#define PREAMBLE_CFG1_NUM_0					(0 << 2) // No preamble transmitted
#define PREAMBLE_CFG1_NUM_0_5				(1 << 2) // 0.5 preamble bytes transmitted
#define PREAMBLE_CFG1_NUM_1					(2 << 2) // 1 preamble bytes transmitted
#define PREAMBLE_CFG1_NUM_1_5				(3 << 2) // 1.5 preamble bytes transmitted
#define PREAMBLE_CFG1_NUM_2					(4 << 2) // 2 preamble bytes transmitted
#define PREAMBLE_CFG1_NUM_3					(5 << 2) // 3 preamble bytes transmitted
#define PREAMBLE_CFG1_NUM_4					(6 << 2) // 4 preamble bytes transmitted
#define PREAMBLE_CFG1_NUM_5					(7 << 2) // 5 preamble bytes transmitted
#define PREAMBLE_CFG1_NUM_6					(8 << 2) // 6 preamble bytes transmitted
#define PREAMBLE_CFG1_NUM_7					(9 << 2) // 7 preamble bytes transmitted
#define PREAMBLE_CFG1_NUM_8					(10 << 2) // 8 preamble bytes transmitted
#define PREAMBLE_CFG1_NUM_12				(11 << 2) // 12 preamble bytes transmitted
#define PREAMBLE_CFG1_NUM_24				(12 << 2) // 24 preamble bytes transmitted
#define PREAMBLE_CFG1_NUM_30				(13 << 2) // 30 preamble bytes transmitted

// FIFO_CFG
#define FIFO_CFG_CRC_AUTOFLUSH_ENABLED		(1 << 7)
#define FIFO_CFG_CRC_AUTOFLUSH_DISABLED		0

#endif /* CC1120_CC1120_H_ */
