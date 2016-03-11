#ifndef STM32_HW_H
#define STM32_HW_H

/*************************************************7**********************************
 * INCLUDES
 */
#include "ch.h"
#include "hal.h"
#include "hal_defs.h"

/***********************************************************************************
 * MACROS
 */

#define SPI_USE_SW							1

#if SPI_USE_SW
#include "spi_sw.h"
#endif

#define INCLUDE_PA
//#define PA_CTRL_FROM_MCU

// SPI HW
#if SPI_USE_SW
#define CC2520_SPI_CSN_PORT					GPIOB
#define CC2520_SPI_CSN_PIN					12
#define CC2520_SPI_SCK_PORT					GPIOB
#define CC2520_SPI_SCK_PIN					13
#define CC2520_SPI_MISO_PORT				GPIOB
#define CC2520_SPI_MISO_PIN					15
#define CC2520_SPI_MOSI_PORT				GPIOB
#define CC2520_SPI_MOSI_PIN					14
#else
#define CC2520_SPI							SPID2
#define GPIO_AF_SPI							5
#define CC2520_SPI_CSN_PORT					GPIOB
#define CC2520_SPI_CSN_PIN					12
#define CC2520_SPI_SCK_PORT					GPIOB
#define CC2520_SPI_SCK_PIN					13
#define CC2520_SPI_MISO_PORT				GPIOB
#define CC2520_SPI_MISO_PIN					14
#define CC2520_SPI_MOSI_PORT				GPIOB
#define CC2520_SPI_MOSI_PIN					15
#endif

// CC2520 I/O Definitions
#define MCU_SET_PIN_VAL(port, pin, value) 	palWritePad(port, pin, value)

// Basic I/O pin setup (Set reset pin to output)
#define CC2520_BASIC_IO_DIR_INIT()	palSetPadMode(GPIOB, 11, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST)

// MCU port control for SPI interface
#define CC2520_DISABLE_SPI_FUNC() \
		palSetPadMode(CC2520_SPI_SCK_PORT, CC2520_SPI_SCK_PIN, PAL_MODE_INPUT); \
		palSetPadMode(CC2520_SPI_MISO_PORT, CC2520_SPI_MISO_PIN, PAL_MODE_INPUT); \
		palSetPadMode(CC2520_SPI_MOSI_PORT, CC2520_SPI_MOSI_PIN, PAL_MODE_INPUT);

#if SPI_USE_SW
#define CC2520_ENABLE_SPI_FUNC() \
		palSetPadMode(CC2520_SPI_MISO_PORT, CC2520_SPI_MISO_PIN, PAL_MODE_INPUT); \
		palSetPadMode(CC2520_SPI_SCK_PORT, CC2520_SPI_SCK_PIN, PAL_MODE_OUTPUT_PUSHPULL); \
		palSetPadMode(CC2520_SPI_MOSI_PORT, CC2520_SPI_MOSI_PIN, PAL_MODE_OUTPUT_PUSHPULL); \
		palClearPad(CC2520_SPI_SCK_PORT, CC2520_SPI_SCK_PIN);
#else
#define CC2520_ENABLE_SPI_FUNC() \
		palSetPadMode(CC2520_SPI_SCK_PORT, CC2520_SPI_SCK_PIN, PAL_MODE_ALTERNATE(GPIO_AF_SPI) | PAL_STM32_OSPEED_HIGHEST); \
		palSetPadMode(CC2520_SPI_MISO_PORT, CC2520_SPI_MISO_PIN, PAL_MODE_ALTERNATE(GPIO_AF_SPI)); \
		palSetPadMode(CC2520_SPI_MOSI_PORT, CC2520_SPI_MOSI_PIN, PAL_MODE_ALTERNATE(GPIO_AF_SPI) | PAL_STM32_OSPEED_HIGHEST);

#endif

// GPIO pin direction control
#define CC2520_GPIO_DIR_OUT(pin) \
		st( \
				if (pin == 0) CC2520_GPIO0_DIR_OUT(); \
				if (pin == 1) CC2520_GPIO1_DIR_OUT(); \
				if (pin == 2) CC2520_GPIO2_DIR_OUT(); \
				if (pin == 3) CC2520_GPIO3_DIR_OUT(); \
				if (pin == 4) CC2520_GPIO4_DIR_OUT(); \
				if (pin == 5) CC2520_GPIO5_DIR_OUT(); \
		)
#define CC2520_GPIO0_DIR_OUT()          palSetPadMode(GPIOA, 7, PAL_MODE_INPUT)
#define CC2520_GPIO1_DIR_OUT()          palSetPadMode(GPIOC, 4, PAL_MODE_INPUT)
#define CC2520_GPIO2_DIR_OUT()          palSetPadMode(GPIOC, 5, PAL_MODE_INPUT)
#define CC2520_GPIO3_DIR_OUT()          palSetPadMode(GPIOB, 0, PAL_MODE_INPUT)
#define CC2520_GPIO4_DIR_OUT()          palSetPadMode(GPIOB, 1, PAL_MODE_INPUT)
#define CC2520_GPIO5_DIR_OUT()          palSetPadMode(GPIOB, 10, PAL_MODE_INPUT)
#define CC2520_GPIO_DIR_IN(pin) \
		st( \
				if (pin == 0) CC2520_GPIO0_DIR_IN(); \
				if (pin == 1) CC2520_GPIO1_DIR_IN(); \
				if (pin == 2) CC2520_GPIO2_DIR_IN(); \
				if (pin == 3) CC2520_GPIO3_DIR_IN(); \
				if (pin == 4) CC2520_GPIO4_DIR_IN(); \
				if (pin == 5) CC2520_GPIO5_DIR_IN(); \
		)
#define CC2520_GPIO0_DIR_IN()           palSetPadMode(GPIOA, 7, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST)
#define CC2520_GPIO1_DIR_IN()           palSetPadMode(GPIOC, 4, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST)
#define CC2520_GPIO2_DIR_IN()           palSetPadMode(GPIOC, 5, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST)
#define CC2520_GPIO3_DIR_IN()           palSetPadMode(GPIOB, 0, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST)
#define CC2520_GPIO4_DIR_IN()           palSetPadMode(GPIOB, 1, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST)
#define CC2520_GPIO5_DIR_IN()           palSetPadMode(GPIOB, 10, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST)

// Outputs: Power and reset control
#define CC2520_RESET_OPIN(v)			MCU_SET_PIN_VAL(GPIOB, 11, v)

// Outputs: GPIO
#define CC2520_GPIO0_OPIN(v)			MCU_SET_PIN_VAL(GPIOA, 7, v)
#define CC2520_GPIO1_OPIN(v)			MCU_SET_PIN_VAL(GPIOC, 4, v)
#define CC2520_GPIO2_OPIN(v)			MCU_SET_PIN_VAL(GPIOC, 5, v)
#define CC2520_GPIO3_OPIN(v)			MCU_SET_PIN_VAL(GPIOB, 0, v)
#define CC2520_GPIO4_OPIN(v)			MCU_SET_PIN_VAL(GPIOB, 1, v)
#define CC2520_GPIO5_OPIN(v)			MCU_SET_PIN_VAL(GPIOB, 10, v)

// Outputs: SPI interface
#define CC2520_CSN_OPIN(v)				MCU_SET_PIN_VAL(GPIOB, 12, v)
#define CC2520_SCLK_OPIN(v)				MCU_SET_PIN_VAL(GPIOB, 13, v)
#define CC2520_MOSI_OPIN(v)				MCU_SET_PIN_VAL(GPIOB, 15, v)

// Inputs: GPIO
#define CC2520_GPIO0_IPIN				palReadPad(GPIOA, 7)
#define CC2520_GPIO1_IPIN				palReadPad(GPIOC, 4)
#define CC2520_GPIO2_IPIN				palReadPad(GPIOC, 5)
#define CC2520_GPIO3_IPIN				palReadPad(GPIOB, 0)
#define CC2520_GPIO4_IPIN				palReadPad(GPIOB, 1)
#define CC2520_GPIO5_IPIN				palReadPad(GPIOB, 10)

// Interrupt
#define HAL_INT_OFF()					chSysLock();
#define HAL_INT_ON()					chSysUnlock();

// Inputs: SPI interface
#define CC2520_MISO_IPIN				palReadPad(CC2520_SPI_MISO_PORT, CC2520_SPI_MISO_PIN)
#define CC2520_MISO_OPIN(v)				MCU_SET_PIN_VAL(CC2520_SPI_MISO_PORT, CC2520_SPI_MISO_PIN, v)
#define CC2520_MISO_DIR_IN()			palSetPadMode(CC2520_SPI_MISO_PORT, CC2520_SPI_MISO_PIN, PAL_MODE_INPUT);
#if SPI_USE_SW
#define CC2520_MISO_DIR_OUT()			palSetPadMode(CC2520_SPI_MISO_PORT, CC2520_SPI_MISO_PIN, PAL_MODE_INPUT)
#else
#define CC2520_MISO_DIR_OUT()			palSetPadMode(CC2520_SPI_MISO_PORT, CC2520_SPI_MISO_PIN, PAL_MODE_ALTERNATE(GPIO_AF_SPI))
#endif

// PA Macros
#define PA_EN_OUT()							palSetPadMode(GPIOA, 6, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST)
#define PA_LNA_EN_OUT()						palSetPadMode(GPIOA, 5, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST)
#define PA_HGM_OUT()						palSetPadMode(GPIOA, 4, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST)
#define PA_EN(v)							MCU_SET_PIN_VAL(GPIOA, 6, v)
#define PA_LNA_EN(v)						MCU_SET_PIN_VAL(GPIOA, 5, v)
#define PA_HGM(v)							MCU_SET_PIN_VAL(GPIOA, 4, v)

// SPI access macros
#if SPI_USE_SW
#define CC2520_SPI_BEGIN()					spi_sw_begin()
#define CC2520_SPI_TXRX(x)					halSpiExc(x)
#define CC2520_INS_WR_ARRAY(count, pData)	spi_sw_transfer(0, (char*)pData, count)
#define CC2520_INS_RD_ARRAY(count, pData)	spi_sw_transfer((char*)pData, 0, count)
#define CC2520_SPI_END()					spi_sw_end()
#else
#define CC2520_SPI_BEGIN()					spiSelect(&CC2520_SPI)
#define CC2520_SPI_TXRX(x)					halSpiExc(x)
#define CC2520_INS_WR_ARRAY(count, pData)	(count > 0 ? spiSend(&CC2520_SPI, count, pData):0)
#define CC2520_INS_RD_ARRAY(count, pData)	(count > 0 ? spiReceive(&CC2520_SPI, count, pData):0)
#define CC2520_SPI_END()					spiUnselect(&CC2520_SPI)
#endif

/***********************************************************************************
 * GLOBAL VARIABLES
 */


/***********************************************************************************
 * PUBLIC FUNCTIONS
 */
void halAssyInit(void);
unsigned char halSpiExc(unsigned char x);

#endif
