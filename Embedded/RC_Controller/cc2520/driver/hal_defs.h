/***********************************************************************************
  Filename:     hal_defs.h

  Description:  HAL defines

***********************************************************************************/

#ifndef HAL_DEFS_H
#define HAL_DEFS_H

/***********************************************************************************
* CONSTANTS AND DEFINES
*/

#ifndef TRUE
#define TRUE 1
#else
#ifdef __IAR_SYSTEMS_ICC__
#warning "Macro TRUE already defined"
#endif
#endif

#ifndef FALSE
#define FALSE 0
#else
#ifdef __IAR_SYSTEMS_ICC__
#warning "Macro FALSE already defined"
#endif
#endif

#ifndef NULL
#define NULL (void *)0
#else
#ifdef __IAR_SYSTEMS_ICC__
#warning "Macro NULL already defined"
#endif
#endif

#ifndef SUCCESS
#define SUCCESS 0
#else
#warning "Macro SUCCESS already defined"
#endif

#ifndef FAILED
#ifndef WIN32
#define FAILED  1
#endif
#else
#ifdef __IAR_SYSTEMS_ICC__
#warning "Macro FAILED already defined"
#endif
#endif

/***********************************************************************************
* MACROS
*/

#ifndef BV
#define BV(n)      (1 << (n))
#endif

#ifndef BM
#define BM(n)      (1 << (n))
#endif

#ifndef BF
#define BF(x,b,s)  (((x) & (b)) >> (s))
#endif

#ifndef MIN
#define MIN(n,m)   (((n) < (m)) ? (n) : (m))
#endif

#ifndef MAX
#define MAX(n,m)   (((n) < (m)) ? (m) : (n))
#endif

#ifndef ABS
#define ABS(n)     (((n) < 0) ? -(n) : (n))
#endif


/* uint32 processing */
#define BREAK_UINT32( var, ByteNum ) \
    (uint8)((uint32)(((var) >>((ByteNum) * 8)) & 0x00FF))

#define BUILD_UINT32(Byte0, Byte1, Byte2, Byte3) \
    ((uint32)((uint32)((Byte0) & 0x00FF) \
        + ((uint32)((Byte1) & 0x00FF) << 8) \
            + ((uint32)((Byte2) & 0x00FF) << 16) \
                + ((uint32)((Byte3) & 0x00FF) << 24)))

#define HI_UINT32(a) ((uint16) (((uint32)(a)) >> 16))
#define LO_UINT32(a) ((uint16) ((uint32)(a)))


/* uint16 processing */
#define BUILD_UINT16(loByte, hiByte) \
    ((uint16)(((loByte) & 0x00FF) + (((hiByte) & 0x00FF) << 8)))

#define HI_UINT16(a) (((uint16)(a) >> 8) & 0xFF)
#define LO_UINT16(a) ((uint16)(a) & 0xFF)


/* uint16 processing */
#define BUILD_UINT8(hiByte, loByte) \
    ((uint8)(((loByte) & 0x0F) + (((hiByte) & 0x0F) << 4)))

#define HI_UINT8(a) (((uint8)(a) >> 4) & 0x0F)
#define LO_UINT8(a) ((uint8)(a) & 0x0F)

/***********************************************************************************
 * Host to network byte order macros
 */
#ifdef BIG_ENDIAN
#define UINT16_HTON(x)  st( utilReverseBuf((uint8*)&x, sizeof(uint16)); )
#define UINT16_NTOH(x)  st( utilReverseBuf((uint8*)&x, sizeof(uint16)); )

#define UINT32_HTON(x)  st( utilReverseBuf((uint8*)&x, sizeof(uint32)); )
#define UINT32_NTOH(x)  st( utilReverseBuf((uint8*)&x, sizeof(uint32)); )
#else
#define UINT16_HTON(x)
#define UINT16_NTOH(x)

#define UINT32_HTON(x)
#define UINT32_NTOH(x)
#endif


/*
*  This macro is for use by other macros to form a fully valid C statement.
*  Without this, the if/else conditionals could show unexpected behavior.
*
*  For example, use...
*    #define SET_REGS()  st( ioreg1 = 0; ioreg2 = 0; )
*  instead of ...
*    #define SET_REGS()  { ioreg1 = 0; ioreg2 = 0; }
*  or
*    #define  SET_REGS()    ioreg1 = 0; ioreg2 = 0;
*  The last macro would not behave as expected in the if/else construct.
*  The second to last macro will cause a compiler error in certain uses
*  of if/else construct
*
*  It is not necessary, or recommended, to use this macro where there is
*  already a valid C statement.  For example, the following is redundant...
*    #define CALL_FUNC()   st(  func();  )
*  This should simply be...
*    #define CALL_FUNC()   func()
*
* (The while condition below evaluates false without generating a
*  constant-controlling-loop type of warning on most compilers.)
*/
#define st(x)      do { x } while (__LINE__ == -1)


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
