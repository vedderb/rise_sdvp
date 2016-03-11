/*********************************************************************
	Filename:       hal_rf_util.c

	Description:    Support for anergy detection applications.

*********************************************************************/


/*********************************************************************
* INCLUDES
*/
#include "hal_cc2520.h"
#include "hal_rf_util.h"

/*********************************************************************
* CONSTANTS
*/


/*********************************************************************
* MACROS
*/

/*********************************************************************
* TYPEDEFS
*/


/*********************************************************************
* GLOBAL VARIABLES
*/



/*********************************************************************
* FUNCTIONS
*/

/***********************************************************************************
* @fn          halSampleED
*
* @brief      Sample Energy Detect
*
* @param      uint8 channel - channel between 11 and 26
*             uint16 sampleTime - sample time in us
*            
* @return     int8 - sampled RSSI value      
*/
int8 halSampleED(uint8 channel, uint16 sampleTime)
{
    int8 rssi=0;
    
    CC2520_REGWR8(CC2520_FREQCTRL, 0x0B + ( (channel-11) * 5));
    CC2520_SRXON();
    while (!CC2520_REGRD8(CC2520_RSSISTAT));
    
    // Enable ED scan mode
    CC2520_BSET(CC2520_MAKE_BIT_ADDR(CC2520_FRMCTRL0, 4));
    
    // Spend sampleTime us accumulating the peak RSSI value
//    halMcuWaitUs(sampleTime);
    chThdSleepMicroseconds(sampleTime);
    rssi = CC2520_REGRD8(CC2520_RSSI);
    
    // Exit the current channel
    CC2520_SRFOFF();
    // Disable ED scan mode
    CC2520_BCLR(CC2520_MAKE_BIT_ADDR(CC2520_FRMCTRL0, 4));
    
    return rssi;
}

/***********************************************************************************
* @fn          halSetRxScanMode
*
* @brief       Set chip in RX scanning mode
*
* @param       none 
*            
*
* @return     none
*/
void halSetRxScanMode(void)
{
    // Make CC2520 enter infinite RX mode
    CC2520_REGWR8(CC2520_FRMCTRL0, 0x0C);
}
