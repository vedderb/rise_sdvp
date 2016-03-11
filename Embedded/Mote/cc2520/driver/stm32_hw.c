/***********************************************************************************
 * INCLUDES
 */
#include "stm32_hw.h"


/***********************************************************************************
 * GLOBAL VARIABLES
 */


/***********************************************************************************
 * PRIVATE VARIABLES
 */
static const SPIConfig spicfg = {
		NULL,
		CC2520_SPI_CSN_PORT,
		CC2520_SPI_CSN_PIN,
		SPI_CR1_BR_1 // 5.25 MHz
};


/***********************************************************************************
 * FUNCTIONS
 */
static void halRadioSpiInit(void);
static void halMcuRfInterfaceInit(void);

/***********************************************************************************
 * @fn          halRadioSpiInit
 *
 * @brief       Initalise Radio SPI interface
 *
 * @param       none
 *
 * @return      none
 */
static void halRadioSpiInit(void)
{
	palSetPad(CC2520_SPI_CSN_PORT, CC2520_SPI_CSN_PIN);
	palSetPadMode(CC2520_SPI_CSN_PORT, CC2520_SPI_CSN_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	CC2520_ENABLE_SPI_FUNC();

#if !SPI_USE_SW
	spiStart(&CC2520_SPI, &spicfg);
#endif
}


/***********************************************************************************
 * @fn      halMcuRfInterfaceInit
 *
 * @brief   Initialises SPI interface to CC2520 and configures reset and vreg
 *          signals as MCU outputs.
 *
 * @param   none
 *
 * @return  none
 */
static void halMcuRfInterfaceInit(void)
{
	// Initialize the CC2520 interface
	CC2520_RESET_OPIN(0);
	CC2520_BASIC_IO_DIR_INIT();
}


/***********************************************************************************
 * @fn      halAssyInit
 *
 * @brief   Initialize interfaces between radio and MCU
 *
 * @param   none
 *
 * @return  none
 */
void halAssyInit(void)
{
	CC2520_GPIO0_DIR_OUT();
	CC2520_GPIO1_DIR_OUT();
	CC2520_GPIO2_DIR_OUT();
	CC2520_GPIO3_DIR_OUT();
	CC2520_GPIO4_DIR_OUT();
	CC2520_GPIO5_DIR_OUT();

	halRadioSpiInit();
	halMcuRfInterfaceInit();
#ifndef MRFI_CC2520
	//halDigioConfig(&pinRadio_GPIO0);
#endif
}

unsigned char halSpiExc(unsigned char x) {
	unsigned char rx;
#if SPI_USE_SW
	spi_sw_transfer((char*)&rx, (char*)&x, 1);
#else
	spiExchange(&CC2520_SPI, 1, &x, &rx);
#endif
	return rx;
}
