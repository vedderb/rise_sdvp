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

#include "deca_port.h"
#include "stm32f4xx_conf.h"
#include "comm_usb.h"

/*
 * Connections
 *
 * SPI1
 *
 * PA1: WAKEUP
 * PA4: CS
 * PA5: SCK
 * PA6: MISO
 * PA7: MOSI
 *
 * PB5: IRQ (GPIO8)
 *
 * PC1: GPIO7
 * PC2: GPIO6 (SPI_PHA)
 * PC3: GPIO5 (SPI_POL)
 * PC4: GPIO4
 * PC5: GPIO3 (TXLED)
 * PC6: GPIO2 (RXLED)
 * PC7: GPIO1 (SFD_LED)
 * PC8: GPIO0 (RXOK_LED)
 * PC9: RSTn
 *
 */

#define DW_SPI				SPID1

#define DW_PORT_CS			GPIOA
#define DW_PIN_CS			4
#define DW_PORT_SCK			GPIOA
#define DW_PIN_SCK			5
#define DW_PORT_MISO		GPIOA
#define DW_PIN_MISO			6
#define DW_PORT_MOSI		GPIOA
#define DW_PIN_MOSI			7

#define DW_PORT_GPIO5		GPIOC
#define DW_PIN_GPIO5		3
#define DW_PORT_GPIO6		GPIOC
#define DW_PIN_GPIO6		2

#define DW_PORT_RST			GPIOC
#define DW_PIN_RST			9

#define DW_PORT_IRQ			GPIOB
#define DW_PIN_IRQ			5

// Threads
static THD_WORKING_AREA(isr_thread_wa, 2048);
static THD_FUNCTION(isr_thread, arg);
static thread_t *isr_tp;

// Private variables
bool isr_enabled;

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
static const SPIConfig spicfg_slow = {
		NULL,
		DW_PORT_CS,
		DW_PIN_CS,
		SPI_CR1_BR_2 // 2.626 MHz
};

static const SPIConfig spicfg_fast = {
		NULL,
		DW_PORT_CS,
		DW_PIN_CS,
		SPI_CR1_BR_1 // 10.5 MHz
};

void deca_port_init(void) {
	palSetPadMode(DW_PORT_GPIO5, DW_PIN_GPIO5, PAL_MODE_INPUT_PULLDOWN);
	palSetPadMode(DW_PORT_GPIO6, DW_PIN_GPIO6, PAL_MODE_INPUT_PULLDOWN);
	palSetPadMode(DW_PORT_IRQ, DW_PIN_IRQ, PAL_MODE_INPUT_PULLDOWN);

	palSetPadMode(DW_PORT_CS, DW_PIN_CS, PAL_STM32_MODE_OUTPUT | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(DW_PORT_SCK, DW_PIN_SCK, PAL_MODE_ALTERNATE(GPIO_AF_SPI1));
	palSetPadMode(DW_PORT_MISO, DW_PIN_MISO, PAL_MODE_ALTERNATE(GPIO_AF_SPI1));
	palSetPadMode(DW_PORT_MOSI, DW_PIN_MOSI, PAL_MODE_ALTERNATE(GPIO_AF_SPI1));

	spiStart(&DW_SPI, &spicfg_slow);

	chThdCreateStatic(isr_thread_wa, sizeof(isr_thread_wa), NORMALPRIO + 2, isr_thread, NULL);
	extChannelEnable(&EXTD1, 5);
	isr_enabled = true;
}

void deca_port_reset(void) {
	palSetPadMode(DW_PORT_RST, DW_PIN_RST, PAL_STM32_MODE_OUTPUT | PAL_STM32_OSPEED_MID1);
	palClearPad(DW_PORT_RST, DW_PIN_RST);
	chThdSleepMilliseconds(1);
	palSetPadMode(DW_PORT_RST, DW_PIN_RST, PAL_MODE_INPUT);
	palSetPad(DW_PORT_RST, DW_PIN_RST);
	chThdSleepMilliseconds(2);
}

void deca_port_set_spi_slow(void) {
	spiStart(&DW_SPI, &spicfg_slow);
}

void deca_port_set_spi_fast(void) {
	spiStart(&DW_SPI, &spicfg_fast);
}

int deca_port_writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer) {
	spiAcquireBus(&DW_SPI);
	spiSelect(&DW_SPI);
	spiSend(&DW_SPI, headerLength, headerBuffer);
	spiSend(&DW_SPI, bodylength, bodyBuffer);
	spiUnselect(&DW_SPI);
	spiReleaseBus(&DW_SPI);

	return 0;
}

int deca_port_readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer) {
	spiAcquireBus(&DW_SPI);
	spiSelect(&DW_SPI);
	spiSend(&DW_SPI, headerLength, headerBuffer);
	spiReceive(&DW_SPI, readlength, readBuffer);
	spiUnselect(&DW_SPI);
	spiReleaseBus(&DW_SPI);

	return 0;
}

void deca_port_sleep(unsigned int time_ms) {
	chThdSleepMilliseconds(time_ms);
}

decaIrqStatus_t decamutexon(void) {
	decaIrqStatus_t ret = isr_enabled;
	extChannelDisable(&EXTD1, 5);
	isr_enabled = false;
	return ret;
}

void decamutexoff(decaIrqStatus_t s) {
	if (s) {
		extChannelEnable(&EXTD1, 5);
		isr_enabled = true;
	}
}

void deca_port_ext_cb(EXTDriver *extp, expchannel_t channel) {
	(void)extp;
	(void)channel;

	chSysLockFromISR();
	chEvtSignalI(isr_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

static THD_FUNCTION(isr_thread, arg) {
	(void) arg;
	chRegSetThreadName("DECA ISR");

	isr_tp = chThdGetSelfX();

	for (;;) {
		chEvtWaitAnyTimeout((eventmask_t)1, 100);

		while (palReadPad(DW_PORT_IRQ, DW_PIN_IRQ)) {
			dwt_isr();
		}
	}
}
