/***********************************************************************************

  Filename:       hal_rf_security.c

  Description:    CC2520 CCM security

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include "hal_cc2520.h"
#include "hal_rf_security.h"

/***********************************************************************************
* CONSTANTS AND DEFINES
*/

#define ADDR_RX                   0x200
#define ADDR_TX                   0x280
#define ADDR_KEY                  0x300                 // Key address
#define ADDR_NONCE_RX             0x310                 // Nonce for incoming packets
#define ADDR_NONCE_TX             0x320                 // Nonce for outgoing packets

#define AUTHSTAT_H_BM             0x08                  // AUTHSTAT_H bit of DPUSTAT

#define HIGH_PRIORITY             1
#define LOW_PRIORITY              0

#define PKT_LEN_MAX               127


/***********************************************************************************
* @fn      halRfSecurityInit
*
* @brief   Security init. Write nonces and key to chip.
*
* @param   none
*
* @return  none
*/
void halRfSecurityInit(uint8* key, uint8* nonceRx, uint8* nonceTx)
{
    // Write key
    CC2520_MEMWR(ADDR_KEY,KEY_LENGTH,key);

    // Write nonce RX
    CC2520_MEMWR(ADDR_NONCE_RX,NONCE_LENGTH,nonceRx);

    // Write nonce TX
    CC2520_MEMWR(ADDR_NONCE_TX,NONCE_LENGTH,nonceTx);

    // Reverse key
    CC2520_MEMCPR(HIGH_PRIORITY,KEY_LENGTH,ADDR_KEY,ADDR_KEY);

    // Reverse nonces
    CC2520_MEMCPR(HIGH_PRIORITY,NONCE_LENGTH,ADDR_NONCE_RX,ADDR_NONCE_RX);
    CC2520_MEMCPR(HIGH_PRIORITY,NONCE_LENGTH,ADDR_NONCE_TX,ADDR_NONCE_TX);
}


/***********************************************************************************
* @fn      halRfReadRxBufSecure
*
* @brief   Decrypts and reverse authenticates with CCM then reads out received
*          frame
*
* @param   uint8* data - data buffer. This must be allocated by caller.
*          uint8 length - number of bytes
*          uint8 encrLength - number of bytes to decrypt
*          uint8 authLength - number of bytes to reverse authenticate
*          uuint8 m - integrity code (m=1,2,3 gives lenght of integrity
*                   field 4,8,16)
*
* @return  SUCCESS or FAILED
*/
uint8 halRfReadRxBufSecure(uint8* data, uint8 length, uint8 encrLength, \
    uint8 authLength, uint8 m)
{
    uint8 dpuStat;

    CC2520_RXBUFMOV(HIGH_PRIORITY, ADDR_RX, length, NULL);
    WAIT_DPU_DONE_H();

    // Find Framecounter value in received packet starting from 10th byte
    // Copy in to nonce bytes (3-6) frame counter bytes
    // Incoming frame uses nonce Rx
    CC2520_MEMCP(HIGH_PRIORITY, 4, ADDR_RX+10, ADDR_NONCE_RX+3);
    WAIT_DPU_DONE_H();

    // Copy in short address to nonce bytes (7-8)
    CC2520_MEMCP(HIGH_PRIORITY, 2, ADDR_RX+7, ADDR_NONCE_RX+7);
    WAIT_DPU_DONE_H();

    // Perform decryption and authentication
    CC2520_UCCM(HIGH_PRIORITY,ADDR_KEY/16, encrLength, ADDR_NONCE_RX/16, ADDR_RX, ADDR_RX+authLength, authLength, m);
    WAIT_DPU_DONE_H();

    // Check authentication status
    dpuStat = CC2520_REGRD8(CC2520_DPUSTAT);

    // Read from RX work buffer into data buffer
    CC2520_MEMRD(ADDR_RX, length, data);

    if( (dpuStat & AUTHSTAT_H_BM) != AUTHSTAT_H_BM ) {
        // Authentication failed
        return FAILED;
    } else {
        return !FAILED;
    }
}


/***********************************************************************************
* @fn      halRfWriteTxBufSecure
*
* @brief   Encrypt and authenticate plaintext then fill TX buffer
*
* @param   uint8* data - data buffer. This must be allocated by caller.
*          uint8 length - number of bytes
*          uint8 encrLength - number of bytes to decrypt
*          uint8 authLength - number of bytes to reverse authenticate
*          uint8 m - integrity code (m=1,2,3 gives lenght of integrity
*                   field 4,8,16)
*
* @return  none
*/
void halRfWriteTxBufSecure(uint8* data, uint8 length, uint8 encrLength, uint8 authLength, uint8 m)
{
    uint8 micLength;

    // Check range of m
    HAL_ASSERT(m<=4);

    if(m>0) {
        micLength = 0x2<<m;
    }
    else if(m==0) {
        micLength=0;
    }

    // Write packet to work buffer
    CC2520_MEMWR(ADDR_TX, length, data);

    // skip the length byte and start from the next byte in TXBUF
    // Outgoing frame uses nonce_tx
    CC2520_CCM(HIGH_PRIORITY,ADDR_KEY/16, encrLength, ADDR_NONCE_TX/16, ADDR_TX+1, 0, authLength, m);
    WAIT_DPU_DONE_H();

    // copy from work buffer to TX FIFO
    CC2520_TXBUFCP(HIGH_PRIORITY, ADDR_TX, length+micLength, NULL);
    WAIT_DPU_DONE_H();

}


/***********************************************************************************
* @fn      halRfIncNonceTx
*
* @brief   Increments frame counter field of stored nonce TX
*
* @param   none
*
* @return  none
*/
void halRfIncNonceTx(void)
{
    // Increment frame counter field of 16 byte nonce TX
    // Frame counter field is 4 bytes long

    // Increment framecounter bytes (3-6) of nonce TX
    CC2520_INC(HIGH_PRIORITY, 2, ADDR_NONCE_TX+3);
    WAIT_DPU_DONE_H();
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
