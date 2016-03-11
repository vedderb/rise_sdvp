/*********************************************************************
    Filename:       hal_util_rf.h

    Description:    

    Notes:			

    Copyright (c) 2006 by Texas Instruments, Inc.
    All Rights Reserved.  Permission to use, reproduce, copy, prepare
    derivative works, modify, distribute, perform, display or sell this
    software and/or its documentation for any purpose is prohibited
    without the express written consent of Texas Instruments, Inc.
*********************************************************************/
#ifndef UTIL_RF
#define UTIL_RF

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */



/*********************************************************************
 * CONSTANTS
 */




/*********************************************************************
 * TYPEDEFS
 */



/*********************************************************************
 * PUBLIC FUNCTIONS
 */
int8 halSampleED(uint8 channel, uint16 sampleTime);
void halSetRxScanMode(void);

#ifdef __cplusplus
}
#endif

#endif

