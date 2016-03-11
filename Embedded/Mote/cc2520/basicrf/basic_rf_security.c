/***********************************************************************************

  Filename:     basic_rf_security.c

  Description:  Basic RF security library

***********************************************************************************/

#ifdef SECURITY_CCM

/***********************************************************************************
* INCLUDES
*/
#include "basic_rf_security.h"
#include "hal_rf_security.h"

/***********************************************************************************
* CONSTANTS AND DEFINES
*/
#define FLAG_FIELD                          0x09
#define NONCE_SIZE                          16

/***********************************************************************************
* LOCAL VARIABLES
*/

static uint8 nonceTx[NONCE_SIZE];
static uint8 nonceRx[NONCE_SIZE];



/***********************************************************************************
* GLOBAL FUNCTIONS
*/

/***********************************************************************************
* @fn          basicRfSecurityInit
*
* @brief       Initialise key and nonces and write to radio
*
* @param       pConfig - file scope variable holding configuration data for
*              basic RF
*
* @return      none
*/
void basicRfSecurityInit(basicRfCfg_t* pConfig)
{
    uint8 i;

    // Initialise nonce bytes to 0
    for(i=0;i<NONCE_SIZE;i++)
    {
        nonceRx[i] = 0;
        nonceTx[i] = 0;
    }

    // Set nonce flag field (Byte 0)
    nonceRx[0] = FLAG_FIELD;
    nonceTx[0] = FLAG_FIELD;

    // Set byte 7 and 8 of nonce to myAddr
    nonceTx[8] = (uint8)pConfig->myAddr;
    nonceTx[7] = (uint8)(pConfig->myAddr>>8);

    // Set Security mode field of nonces (Byte 13)
    nonceRx[13] = SECURITY_CONTROL;
    nonceTx[13] = SECURITY_CONTROL;

    halRfSecurityInit(pConfig->securityKey, nonceRx, nonceTx);
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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

#endif
