/***********************************************************************************

    Filename:     util.c

    Description:  Utility library

 ***********************************************************************************/

/***********************************************************************************
 * INCLUDES
 */

#include "util.h"
#include "hal_rf.h"

/***********************************************************************************
 * GLOBAL FUNCTIONS
 */

/***********************************************************************************
 * @fn          utilChipIdToStr
 *
 * @brief       Converts a chip ID to a text string.
 *
 * @param       uint8 chipID
 *
 * @return      none
 */
const char* utilChipIdToStr(uint8 chipID)
{
	const char* szId;

	switch(chipID) {
	case HAL_RF_CHIP_ID_CC2420:
		szId= "2420";
		break;
	case HAL_RF_CHIP_ID_CC2430:
		szId= "2430";
		break;
	case HAL_RF_CHIP_ID_CC2431:
		szId= "2431";
		break;
	case HAL_RF_CHIP_ID_CC2520:
		szId= "2520";
		break;
	case HAL_RF_CHIP_ID_CC2530:
		szId= "2530";
		break;
	case HAL_RF_CHIP_ID_CC2531:
		szId= "2531";
		break;
	case HAL_RF_CHIP_ID_CC2510:
		szId= "2510";
		break;
	case HAL_RF_CHIP_ID_CC2511:
		szId= "2511";
		break;
	case HAL_RF_CHIP_ID_CC1110:
		szId= "1110";
		break;
	case HAL_RF_CHIP_ID_CC1111:
		szId= "1111";
		break;
	default:
		szId= "----";
		break;
	};

	return szId;
}

/***********************************************************************************
 * @fn          convInt32ToText
 *
 * @brief       Converts 32 bit int to text
 *
 * @param       int32 value
 *
 * @return      char* - pointer to text buffer which is a file scope allocated array
 */
char* convInt32ToText(int32 value)
{
	static char pValueToTextBuffer[12];
	char *pLast;
	char *pFirst;
	char last;
	uint8 negative;

	pLast = pValueToTextBuffer;

	// Record the sign of the value
	negative = (value < 0);
	value = ABS(value);

	// Print the value in the reverse order
	do {
		*(pLast++) = '0' + (uint8)(value % 10);
		value /= 10;
	} while (value);

	// Add the '-' when the number is negative, and terminate the string
	if (negative) *(pLast++) = '-';
	*(pLast--) = 0x00;

	// Now reverse the string
	pFirst = pValueToTextBuffer;
	while (pLast > pFirst) {
		last = *pLast;
		*(pLast--) = *pFirst;
		*(pFirst++) = last;
	}

	return pValueToTextBuffer;
}

#ifndef WIN32
/***********************************************************************************
 * @fn          min
 *
 * @brief       Return minimum of two values
 *
 * @param       uint8 v1 - value 1
 *              uint8 v2 - value 2
 *
 * @return      uint8 - minimum of two values
 */
uint8 min(uint8 v1, uint8 v2)
{
	if(v1 < v2)
		return v1;
	else return v2;
}

/***********************************************************************************
 * @fn          utilReverseBuf
 *
 * @brief       reverse buffer
 *
 * @param       uint8 pBuf - pointer to buffer
 *              uint8 length - length of buffer
 *
 * @return      void
 */
void utilReverseBuf(uint8* pBuf, uint8 length)
{
	uint8 temp;
	uint8* pBufLast = (pBuf + length - 1);

	while(pBufLast > pBuf){
		temp = *pBuf;
		*pBuf++ = *pBufLast;
		*pBufLast-- = temp;
	}
}
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
