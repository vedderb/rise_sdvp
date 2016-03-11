/***********************************************************************************

  Filename:     hal_rf.h

  Description:  HAL radio interface header file

***********************************************************************************/


#ifndef HAL_RF_H
#define HAL_RF_H

/***********************************************************************************
* INCLUDES
*/
#include "hal_cc2520.h"

/***********************************************************************************
* TYPEDEFS
*/

/***********************************************************************************
* CONSTANTS AND DEFINES
*/

// Chip ID's
#define HAL_RF_CHIP_ID_CC1100               0x00
#define HAL_RF_CHIP_ID_CC1110               0x01
#define HAL_RF_CHIP_ID_CC1111               0x11
#define HAL_RF_CHIP_ID_CC2420               0x02
#define HAL_RF_CHIP_ID_CC2500               0x80
#define HAL_RF_CHIP_ID_CC2510               0x81
#define HAL_RF_CHIP_ID_CC2511               0x91
#define HAL_RF_CHIP_ID_CC2550               0x82
#define HAL_RF_CHIP_ID_CC2520               0x84
#define HAL_RF_CHIP_ID_CC2430               0x85
#define HAL_RF_CHIP_ID_CC2431               0x89
#define HAL_RF_CHIP_ID_CC2530               0xA5
#define HAL_RF_CHIP_ID_CC2531               0xB5
#define HAL_RF_CHIP_ID_CC2540               0x8D

// CC2590/91 gain modes
#define HAL_RF_GAIN_LOW                     0
#define HAL_RF_GAIN_HIGH                    1

// IEEE 802.15.4 defined constants (2.4 GHz logical channels)
#define MIN_CHANNEL 				        11    // 2405 MHz
#define MAX_CHANNEL                         26    // 2480 MHz
#define CHANNEL_SPACING                     5     // MHz


/***********************************************************************************
* GLOBAL FUNCTIONS
*/

void halRfExtCb(EXTDriver *extp, expchannel_t channel);

// Generic RF interface
uint8 halRfInit(void);
uint8 halRfSetTxPower(uint8 power);
uint8 halRfTransmit(void);
void  halRfSetGain(uint8 gainMode);     // With CC2590/91 only

uint8 halRfGetChipId(void);
uint8 halRfGetChipVer(void);
uint8 halRfGetRandomByte(void);
uint8 halRfGetRssiOffset(void);

void  halRfWriteTxBuf(uint8* pData, uint8 length);
void  halRfReadRxBuf(uint8* pData, uint8 length);
void  halRfWaitTransceiverReady(void);
uint8 halRfReadMemory(uint16 addr, uint8* pData, uint8 length);
uint8 halRfWriteMemory(uint16 addr, uint8* pData, uint8 length);

void  halRfReceiveOn(void);
void  halRfReceiveOff(void);
void halRfFlushRx(void);
void halIntOn(void);
void halIntOff(void);
void halRfRxInterruptConfig(void(*func)(void));
void  halRfDisableRxInterrupt(void);
void  halRfEnableRxInterrupt(void);



// IEEE 802.15.4 specific interface
void  halRfSetChannel(uint8 channel);
void  halRfSetShortAddr(uint16 shortAddr);
void  halRfSetPanId(uint16 PanId);


#endif

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
