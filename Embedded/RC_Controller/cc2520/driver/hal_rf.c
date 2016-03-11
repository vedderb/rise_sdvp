/***********************************************************************************

 Filename:       hal_rf.c

 Description:    CC2520 radio interface.

 ***********************************************************************************/

//#pragma GCC optimize ("O0")

/***********************************************************************************
 * INCLUDES
 */
#include "hal_rf.h"
#include "hal_cc2520.h"
#include "util.h"
#include "stm32_hw.h"

#include <stdint.h>

/***********************************************************************************
 * CONSTANTS AND DEFINES
 */

#define RSSI_OFFSET               76
#define RSSI_OFFSET_LNA_HIGHGAIN  ( RSSI_OFFSET + 3 )
#define RSSI_OFFSET_LNA_LOWGAIN   ( RSSI_OFFSET + 14 )

// CC2590-CC2591 support
//
#ifdef INCLUDE_PA

#ifdef PA_CTRL_FROM_MCU
#define PA_RX()		PA_LNA_EN(1)
#define PA_TX()		PA_LNA_EN(0);PA_EN(1)
#define PA_OFF()	PA_LNA_EN(0);PA_EN(0)
#endif

/* Select CC2591 RX low gain mode */
#define HAL_PA_LNA_RX_LGM() PA_HGM(0)

/* Select CC2591 RX high gain mode */
#define HAL_PA_LNA_RX_HGM() PA_HGM(1)

/* TX power lookup index */
#define HAL_RF_TXPOWER_MIN1_DBM   1
#define HAL_RF_TXPOWER_11_DBM     2
#define HAL_RF_TXPOWER_14_DBM     3
#define HAL_RF_TXPOWER_16_DBM     4
#define HAL_RF_TXPOWER_17_DBM     5

/* TX power values */
// See AN065
#define CC2520_TXPOWER_MIN1_DBM   0x03
#define CC2520_TXPOWER_11_DBM     0x2C
#define CC2520_TXPOWER_14_DBM     0xA0
#define CC2520_TXPOWER_16_DBM     0xF0
#define CC2520_TXPOWER_17_DBM     0xF9

#else

/* dummy macros when not using CC2591 */
#define HAL_PA_LNA_RX_LGM()
#define HAL_PA_LNA_RX_HGM()

/* TX power constants */
#define HAL_RF_TXPOWER_MIN_4_DBM   0
#define HAL_RF_TXPOWER_0_DBM       1
#define HAL_RF_TXPOWER_4_DBM       2

#endif

// These macros do nothing by default
#ifndef PA_RX
#define PA_RX()
#endif
#ifndef PA_TX
#define PA_TX()
#endif
#ifndef PA_OFF
#define PA_OFF()
#endif

/***********************************************************************************
 * LOCAL DATA
 */
#ifdef INCLUDE_PA
static const menuItem_t pPowerSettings[] =
{
	{"-1dBm", HAL_RF_TXPOWER_MIN1_DBM},
	{"11dBm", HAL_RF_TXPOWER_11_DBM},
	{"14dBm", HAL_RF_TXPOWER_14_DBM},
	{"16dBm", HAL_RF_TXPOWER_16_DBM},
	{"17dBm", HAL_RF_TXPOWER_17_DBM},
};
#else
static const menuItem_t pPowerSettings[] = {
	{"-4 dBm", HAL_RF_TXPOWER_MIN_4_DBM },
	{ "0 dBm", HAL_RF_TXPOWER_0_DBM },
	{"4 dBm", HAL_RF_TXPOWER_4_DBM } };
#endif

static THD_WORKING_AREA(waIsrTx, 1024);
static thread_t *isr_tp;

// Function pointer to interrupt handler
static void (*gpio0_func)(void) = NULL;

/***********************************************************************************
 * GLOBAL DATA
 */
const menu_t powerMenu = { pPowerSettings,
N_ITEMS(pPowerSettings) };

/***********************************************************************************
 * LOCAL DATA TYPES
 */
typedef struct {
	uint8 reg;
	uint8 val;
} regVal_t;

/***********************************************************************************
 * LOCAL DATA
 */

// Recommended register settings which differ from the data sheet
static regVal_t regval[] = {
		// Tuning settings
#ifdef INCLUDE_PA
		{CC2520_TXPOWER, 0xF9}, // Max TX output power
		{CC2520_TXCTRL, 0xC1}, // See AN065
#else
		{CC2520_TXPOWER, 0xF7}, // Max TX output power
#endif
		{CC2520_CCACTRL0, 0xF8}, // CCA threshold -80dBm
		{CC2520_CCACTRL1, 0b00011010},

		// Recommended RX settings
		{CC2520_MDMCTRL0, 0x85},
		{CC2520_MDMCTRL1, 0x14},
		{CC2520_RXCTRL, 0x3F},
		{CC2520_FSCTRL, 0x5A},
		{CC2520_FSCAL1, 0x03},
#ifdef INCLUDE_PA
		{CC2520_AGCCTRL1, 0x16},
#else
		{CC2520_AGCCTRL1, 0x11},
#endif
		{CC2520_ADCTEST0, 0x10},
		{CC2520_ADCTEST1, 0x0E},
		{CC2520_ADCTEST2, 0x03},

		// Configuration for applications using halRfInit()
		{CC2520_FRMCTRL0, 0x60}, // Auto-ack
		{CC2520_EXTCLOCK, 0x00},
		{CC2520_GPIOCTRL0, 1 + CC2520_EXC_RX_FRM_DONE},
		{CC2520_GPIOCTRL1, CC2520_GPIO_SAMPLED_CCA},
		{CC2520_GPIOCTRL2, CC2520_GPIO_RSSI_VALID},
#if defined(INCLUDE_PA) && !defined(PA_CTRL_FROM_MCU)
		{CC2520_GPIOCTRL3, 1 + CC2520_EXC_RX_OVERFLOW},
		{CC2520_GPIOCTRL4, 0x46}, // EN set to lna_pd[1] inverted
		{CC2520_GPIOCTRL5, 0x47}, // PAEN set to pa_pd inverted
		{CC2520_GPIOPOLARITY, 0b00001111}, // Invert GPIO4 and GPIO5
#else
		{CC2520_GPIOCTRL3, CC2520_GPIO_SFD},
		{CC2520_GPIOCTRL4, CC2520_GPIO_SNIFFER_DATA},
//		{CC2520_GPIOCTRL5, CC2520_GPIO_SNIFFER_CLK},
		{CC2520_GPIOCTRL5, 1 + CC2520_EXC_RX_OVERFLOW},
#endif
		// Terminate array
		{0, 0x00} };

#ifdef INCLUDE_PA
static uint8 rssiOffset = RSSI_OFFSET_LNA_HIGHGAIN;
#else
static uint8 rssiOffset = RSSI_OFFSET;
#endif

/***********************************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 halRfWaitRadioReady(void);
static THD_FUNCTION(isrThread, arg);

/***********************************************************************************
 * GLOBAL FUNCTIONS
 */

/***********************************************************************************
 * @fn      halRfInit
 *
 * @brief   Power up, sets default tuning settings, enables autoack and configures
 *          chip IO
 *
 * @param   none
 *
 * @return  SUCCESS if the radio has started, FAILURE otherwise
 */
uint8 halRfInit(void) {
	regVal_t* p;
	uint8 val;

#ifdef INCLUDE_PA
	PA_HGM_OUT();
#ifdef PA_CTRL_FROM_MCU
	PA_EN_OUT();
	PA_LNA_EN_OUT();
	PA_OFF();
#endif
#endif

	// High gain mode by default
	HAL_PA_LNA_RX_HGM();

	// Avoid GPIO0 interrupts during reset
	halRfDisableRxInterrupt();

	// Make sure to pull the CC2520 RESETn pin low
	CC2520_RESET_OPIN(0);
	CC2520_SPI_END();
	chThdSleepMilliseconds(2);

	// Make sure MISO is configured as output.
	CC2520_MISO_DIR_OUT();

	// Release reset
	CC2520_RESET_OPIN(1);

	// Wait for XOSC stable to be announced on the MISO pin
	if (halRfWaitRadioReady() == FAILED) {
		return FAILED;
	}

	// Write non-default register values
	p = regval;
	while (p->reg != 0) {
		CC2520_MEMWR8(p->reg, p->val);
		p++;
	}

	// ISR Thread
	chThdCreateStatic(waIsrTx, sizeof(waIsrTx), NORMALPRIO + 2, isrThread, NULL);

	// Verify a register
	val = CC2520_MEMRD8(CC2520_MDMCTRL0);
	return val == 0x85 ? SUCCESS : FAILED;
}

/***********************************************************************************
 * @fn      halRfGetChipId
 *
 * @brief   Get chip id
 *
 * @param   none
 *
 * @return  uint8 - result
 */
uint8 halRfGetChipId(void) {
	return (CC2520_MEMRD8(CC2520_CHIPID));
}

/***********************************************************************************
 * @fn      halRfGetChipVer
 *
 * @brief   Get chip version
 *
 * @param   none
 *
 * @return  uint8 - result
 */
uint8 halRfGetChipVer(void) {
	return (CC2520_MEMRD8(CC2520_VERSION));
}

/***********************************************************************************
 * @fn      halRfGetRandomByte
 *
 * @brief   Return random byte
 *
 * @param   none
 *
 * @return  uint8 - random byte
 */

// Comment: has the random generator been initialized
uint8 halRfGetRandomByte(void) {
	return CC2520_RANDOM8();
}

/***********************************************************************************
 * @fn      halRfGetRssiOffset
 *
 * @brief   Return RSSI Offset
 *
 * @param   none
 *
 * @return  uint8 - RSSI offset
 */
uint8 halRfGetRssiOffset(void) {
	return rssiOffset;
}

/***********************************************************************************
 * @fn      halRfSetChannel
 *
 * @brief   Set RF channel in the 2.4GHz band. The Channel must be in the range 11-26,
 *          11= 2005 MHz, channel spacing 5 MHz.
 *
 * @param   channel - logical channel number
 *
 * @return  none
 */
void halRfSetChannel(uint8 channel) {
	CC2520_REGWR8(CC2520_FREQCTRL,
			MIN_CHANNEL + ((channel - MIN_CHANNEL) * CHANNEL_SPACING));
}

/***********************************************************************************
 * @fn      halRfSetShortAddr
 *
 * @brief   Write short address to chip
 *
 * @param   none
 *
 * @return  none
 */
void halRfSetShortAddr(uint16 shortAddr) {
	CC2520_MEMWR16(CC2520_RAM_SHORTADDR, shortAddr);
}

/***********************************************************************************
 * @fn      halRfSetPanId
 *
 * @brief   Write PAN Id to chip
 *
 * @param   none
 *
 * @return  none
 */
void halRfSetPanId(uint16 panId) {
	CC2520_MEMWR16(CC2520_RAM_PANID, panId);
}

/***********************************************************************************
 * @fn      halRfSetTxPower
 *
 * @brief   Set TX output power
 *
 * @param   uint8 power - power level
 *
 * @return  SUCCESS or FAILED
 */
uint8 halRfSetTxPower(uint8 power) {
	uint8 n;

	switch (power) {
#ifdef INCLUDE_PA
	case HAL_RF_TXPOWER_MIN1_DBM: n = CC2520_TXPOWER_MIN1_DBM; break;
	case HAL_RF_TXPOWER_11_DBM: n = CC2520_TXPOWER_11_DBM; break;
	case HAL_RF_TXPOWER_14_DBM: n = CC2520_TXPOWER_14_DBM; break;
	case HAL_RF_TXPOWER_16_DBM: n = CC2520_TXPOWER_16_DBM; break;
	case HAL_RF_TXPOWER_17_DBM: n = CC2520_TXPOWER_17_DBM; break;
#else
	case HAL_RF_TXPOWER_MIN_4_DBM:
		n = CC2520_TXPOWER_MIN_4_DBM;
		break;
	case HAL_RF_TXPOWER_0_DBM:
		n = CC2520_TXPOWER_0_DBM;
		break;
	case HAL_RF_TXPOWER_4_DBM:
		n = CC2520_TXPOWER_4_DBM;
		break;
#endif
	default:
		return FAILED;
	}
	CC2520_REGWR8(CC2520_TXPOWER, n);
	return SUCCESS;
}

/***********************************************************************************
 * @fn      halRfSetGain
 *
 * @brief   Set gain mode - only applicable for units with CC2590/91.
 *
 * @param   uint8 - gain mode
 *
 * @return  none
 */
void halRfSetGain(uint8 gainMode) {
	if (gainMode == HAL_RF_GAIN_LOW) {
		HAL_PA_LNA_RX_LGM();
		rssiOffset = RSSI_OFFSET_LNA_LOWGAIN;
	} else {
		HAL_PA_LNA_RX_HGM();
		rssiOffset = RSSI_OFFSET_LNA_HIGHGAIN;
	}
}

/***********************************************************************************
 * @fn      halRfWriteTxBuf
 *
 * @brief   Write to TX buffer
 *
 * @param   uint8* data - buffer to write
 *          uint8 length - number of bytes
 *
 * @return  none
 */
void halRfWriteTxBuf(uint8* data, uint8 length) {
	// Copy packet to TX FIFO
	CC2520_TXBUF(length, data);
}

/***********************************************************************************
 * @fn      halRfReadRxBuf
 *
 * @brief   Read RX buffer
 *
 * @param   uint8* pData - data buffer. This must be allocated by caller.
 *          uint8 length - number of bytes
 *
 * @return  none
 */
void halRfReadRxBuf(uint8* pData, uint8 length) {
	CC2520_RXBUF(length, pData);
}

/***********************************************************************************
 * @fn      halRfReadMemory
 *
 * @brief   Read RF device memory
 *
 * @param   uint16 addr - memory address
 *          uint8* pData - data buffer. This must be allocated by caller.
 *          uint8 length - number of bytes
 *
 * @return  Number of bytes read
 */
uint8 halRfReadMemory(uint16 addr, uint8* pData, uint8 length) {
	if (addr >= CC2520_MEMORY_SIZE) {
		length = 0;
	} else if (addr + length > CC2520_MEMORY_SIZE) {
		length = CC2520_MEMORY_SIZE - addr;
	}

	CC2520_MEMRD(addr, length, pData);
	return length;
}

/***********************************************************************************
 * @fn      halRfWriteMemory
 *
 * @brief   Write RF device memory
 *
 * @param   uint16 addr - memory address
 *          uint8* pData - data buffer. This must be allocated by caller.
 *          uint8 length - number of bytes
 *
 * @return  Number of bytes written
 */
uint8 halRfWriteMemory(uint16 addr, uint8* pData, uint8 length) {
	(void)addr;
	(void)pData;
	(void)length;
	return 0;
}

/***********************************************************************************
 * @fn      halRfTransmit
 *
 * @brief   Transmit frame with Clear Channel Assessment.
 *
 * @param   none
 *
 * @return  uint8 - SUCCESS or FAILED
 */
uint8 halRfTransmit(void) {
	uint16 timeout = 500; // 500 x 100us = 50ms
	uint8 status = 0;

	PA_RX();

	// Wait for RSSI to become valid
	while(!CC2520_RSSI_VALID_PIN){}

	// Reuse GPIO2 for TX_FRM_DONE exception
	CC2520_CFG_GPIO_OUT(2, 1 + CC2520_EXC_TX_FRM_DONE);

	// Wait for the transmission to begin before exiting (makes sure that this function cannot be called
	// a second time, and thereby canceling the first transmission.
	while (--timeout > 0) {
		CC2520_INS_STROBE(CC2520_INS_STXONCCA);
		if (CC2520_SAMPLED_CCA_PIN) {
			PA_TX();
			break;
		}

		chThdSleepMicroseconds(100);
	}

	if (timeout == 0) {
		status = FAILED;
		CC2520_INS_STROBE(CC2520_INS_SFLUSHTX);
	} else {
		status = SUCCESS;
		// Wait for TX_FRM_DONE exception
		while(!CC2520_TX_FRM_DONE_PIN){};
	}

	PA_RX();

	// Reconfigure GPIO2
	CC2520_CFG_GPIO_OUT(2, CC2520_GPIO_RSSI_VALID);

	return status;
}

/***********************************************************************************
 * @fn      halRfReceiveOn
 *
 * @brief   Turn receiver on
 *
 * @param   none
 *
 * @return  none
 */
void halRfReceiveOn(void) {
	PA_RX();

	/* Flush RX FIFO if needed.
	 * See bug #1 in http://www.ti.com/lit/er/swrz024/swrz024.pdf */
	halRfDisableRxInterrupt();
	CC2520_INS_STROBE(CC2520_INS_SFLUSHRX);
	uint8_t rx_fifo_cnt = CC2520_REGRD8(CC2520_RXFIFOCNT);

	CC2520_INS_STROBE(CC2520_INS_SRXON);

	if (rx_fifo_cnt != CC2520_REGRD8(CC2520_RXFIFOCNT)) {
		CC2520_INS_STROBE(CC2520_INS_SFLUSHRX);
	}
	halRfEnableRxInterrupt();
}

/***********************************************************************************
 * @fn      halRfReceiveOff
 *
 * @brief   Turn receiver off
 *
 * @param   none
 *
 * @return  none
 */
void halRfReceiveOff(void) {
	PA_OFF();

	/* Flush RX FIFO if needed.
	 * See bug #1 in http://www.ti.com/lit/er/swrz024/swrz024.pdf */
	halRfDisableRxInterrupt();
	CC2520_INS_STROBE(CC2520_INS_SFLUSHRX);
	uint8_t rx_fifo_cnt = CC2520_REGRD8(CC2520_RXFIFOCNT);

	CC2520_INS_STROBE(CC2520_INS_SRFOFF);

	if (rx_fifo_cnt != CC2520_REGRD8(CC2520_RXFIFOCNT)) {
		CC2520_INS_STROBE(CC2520_INS_SFLUSHRX);
		// Software clean-up
		// Reset software for frame reception...
	}
	halRfEnableRxInterrupt();
}

void halRfFlushRx(void) {
	/* Flush RX FIFO twice instead of once. See bug #1 in
	 * http://www.ti.com/lit/er/swrz024/swrz024.pdf */
	halRfDisableRxInterrupt();
	CC2520_RXBUF8();
	CC2520_INS_STROBE(CC2520_INS_SFLUSHRX);
	CC2520_INS_STROBE(CC2520_INS_SFLUSHRX);
	halRfEnableRxInterrupt();
}

/***********************************************************************************
 * @fn      halIntOn
 *
 * @brief   Enable global interrupts.
 *
 * @param   none
 *
 * @return  none
 */
void halIntOn(void) {
	HAL_INT_ON();
}

/***********************************************************************************
 * @fn      halIntOff
 *
 * @brief   Turns global interrupts off.
 *
 * @param   none
 *
 * @return  none
 */
void halIntOff(void) {
	HAL_INT_OFF();
}

void halRfRxInterruptConfig(void (*func)(void)) {
	// Set function pointer
	gpio0_func = func;

	CLEAR_EXC_RX_FRM_DONE();
}

/***********************************************************************************
 * @fn      halRfDisableRxInterrupt
 *
 * @brief   Clear and disable RX interrupt.
 *
 * @param   none
 *
 * @return  none
 */
void halRfDisableRxInterrupt(void) {
	CLEAR_EXC_RX_FRM_DONE();
	extChannelDisable(&EXTD1, 9);
}

/***********************************************************************************
 * @fn      halRfEnableRxInterrupt
 *
 * @brief   Enable RX interrupt.
 *
 * @param   none
 *
 * @return  none
 */
void halRfEnableRxInterrupt(void) {
	extChannelEnable(&EXTD1, 9);
}

/**
 * External interrupt handler for lines 9 to 5
 */

void halRfExtCb(EXTDriver *extp, expchannel_t channel) {
	(void)extp;
	(void)channel;

	chSysLockFromISR();
	chEvtSignalI(isr_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

static THD_FUNCTION(isrThread, arg) {
	(void) arg;
	chRegSetThreadName("CC2520 EXTI");

	isr_tp = chThdGetSelfX();

	for (;;) {
		chEvtWaitAny((eventmask_t) 1);

		if (gpio0_func) {
			gpio0_func();
		}
	}
}

/***********************************************************************************
 * @fn      halRfWaitTransceiverReady
 *
 * @brief   Wait until the transceiver is ready (SFD low).
 *
 * @param   none
 *
 * @return  none
 */
void halRfWaitTransceiverReady(void) {
#if defined(INCLUDE_PA) && !defined(PA_CTRL_FROM_MCU)
	// GPIO3 is not conncted to combo board; use SFD at GPIO2 instead
	HAL_INT_OFF();
	// GPIO2 = SFD
	CC2520_CFG_GPIO_OUT(2,CC2520_GPIO_SFD);
	while (CC2520_GPIO2_IPIN) {
		chThdSleep(1);
	};
	// GPIO2 = default (RSSI_VALID)
	CC2520_CFG_GPIO_OUT(2,CC2520_GPIO_RSSI_VALID);
	HAL_INT_ON();
#else
	while (CC2520_SFD_PIN){
		chThdSleep(1);
	};
#endif
}

/***********************************************************************************
 * LOCAL FUNCTIONS
 */
/***********************************************************************************
 * @fn      halRfWaitRadioReady
 *
 * @brief   Wait for the crystal oscillator to stabilise.
 *
 * @param   none
 *
 * @return  SUCCESS if oscillator starts, FAILED otherwise
 */
static uint8 halRfWaitRadioReady(void) {
	uint8 i;

	// Wait for XOSC stable to be announced on the MISO pin
	i= 50;
	CC2520_CSN_OPIN(0);
	while (i > 0 && !CC2520_MISO_IPIN) {
		chThdSleepMicroseconds(100);
		--i;
	}
	CC2520_CSN_OPIN(1);

	return i > 0 ? SUCCESS : FAILED;
}

/***********************************************************************************
 Copyright 2007 Texas Instruments Incorporated. All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License").  You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product.  Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.
 ***********************************************************************************/
