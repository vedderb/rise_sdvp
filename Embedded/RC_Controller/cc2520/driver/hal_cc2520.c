/***********************************************************************************

  Filename:     hal_cc2520.c

  Description:  This file implements the CC2520 Radio HAL.

 ***********************************************************************************/


/***********************************************************************************
 * INCLUDES
 */
#include "hal_cc2520.h"


/***********************************************************************************
 * CONSTANTS AND DEFINES
 */
#define BLOCK_SIZE  255


/***********************************************************************************
 * DATATYPES
 */
typedef union {
	uint16 w;
	struct {
		uint8 b0;
		uint8 b1;
	} b;
} eword_t;


typedef union {
	uint32 dw;
	struct {
		uint8 b0;
		uint8 b1;
		uint8 b2;
		uint8 b3;
	} b;
} edword_t;

/***********************************************************************************
 * LOCAL FUNCTIONS
 */
//static uint8 GotException(uint8 index);
//static uint8 waitexception(uint8 exc, uint16 timeout);
//static void clearexception(uint32 dwMap);
static uint8 CC2520_INS_MEMCP_COMMON(uint8 instr, uint8 pri, uint16 count, \
		uint16 src, uint16 dest);

/***********************************************************************************
 * @fn      GotException
 *
 * @brief   Check if exception is set
 *
 * @param  uint8 index
 *
 * @return  uint8
 */
//static uint8 Gotexception(uint8 index)
//{
//	uint8 exceptions = cc2520_regrd8(cc2520_excflag0 + (index >> 3));
//	if (exceptions & bv(index & 0x07)) {
//		return true;
//	} else {
//		return false;
//	}
//}


/***********************************************************************************
 * @fn      waitException
 *
 * @brief   Wait for exception
 *
 * @param  uint8 exc
 *         uint16 timeOut
 *
 * @return  uint8
 */
//static uint8 waitException(uint8 exc, uint16 timeOut)
//{
//	timeOut /= 10;
//
//	while (--timeOut > 0) {
//		if (GotException(exc))
//			break;
////		halMcuWaitUs(10);
//		chThdSleepMicroseconds(100);
//	}
//	return timeOut>0;
//}


/***********************************************************************************
 * @fn      clearException
 *
 * @brief   Clear exception flag
 *
 * @param  uint32 dwMap
 *
 * @return  none
 */
//static void clearexception(uint32 dwmap)
//{
//	cc2520_regwr24(cc2520_excflag0, ~dwmap);
//}


/***********************************************************************************
 * @fn      CC2520_INS_MEMCP_COMMON
 *
 * @brief   memory copy
 *
 * @param   uint8 instr -
 *          uint8 pri -
 *          uint16 count -
 *          uint16 src -
 *          uint16 dest -
 *
 * @return  uint8 - status byte
 */
static uint8 CC2520_INS_MEMCP_COMMON(uint8 instr, uint8 pri, uint16 count, \
		uint16 src, uint16 dest)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(instr | pri);
	CC2520_SPI_TXRX(count);
	CC2520_SPI_TXRX((HI_UINT16(src) << 4) | HI_UINT16(dest));
	CC2520_SPI_TXRX(LO_UINT16(src));
	CC2520_SPI_TXRX(LO_UINT16(dest));
	CC2520_SPI_END();
	return s;
}

uint8 WAIT_DPU_DONE_H(void) {
	int tries = 0;

	while (CC2520_INS_STROBE(CC2520_INS_SNOP) & DPU_ACTIVE_H_BM) {
		tries++;
		if (tries > 10000) {
			return 0;
		}
	}

	return 1;
}

/***********************************************************************************
 * @fn      CC2520_INS_STROBE
 *
 * @brief   send strobe command
 *
 * @param   uint8 strobe - strobe command
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_INS_STROBE(uint8 strobe)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(strobe);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_SNOP
 *
 * @brief   SNOP instruction strobe
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_SNOP(void)
{
	return CC2520_INS_STROBE(CC2520_INS_SNOP);
}


/***********************************************************************************
 * @fn      CC2520_SIBUFEX
 *
 * @brief   SIBUFEX instruction strobe
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_SIBUFEX(void)
{
	return CC2520_INS_STROBE(CC2520_INS_SIBUFEX);
}


/***********************************************************************************
 * @fn      CC2520_SSAMPLECCA
 *
 * @brief   Sample CCA instruction strobe
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_SSAMPLECCA(void)
{
	return CC2520_INS_STROBE(CC2520_INS_SSAMPLECCA);
}


/***********************************************************************************
 * @fn      CC2520_SXOSCON
 *
 * @brief   SXOSCON instruction strobe
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_SXOSCON(void)
{
	return CC2520_INS_STROBE(CC2520_INS_SXOSCON);
}


/***********************************************************************************
 * @fn      CC2520_STXCAL
 *
 * @brief   STXCAL instruction strobe
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_STXCAL(void)
{
	return CC2520_INS_STROBE(CC2520_INS_STXCAL);
}


/***********************************************************************************
 * @fn      CC2520_SRXON
 *
 * @brief   SRXON instruction strobe
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_SRXON(void)
{
	return CC2520_INS_STROBE(CC2520_INS_SRXON);
}


/***********************************************************************************
 * @fn      CC2520_STXON
 *
 * @brief   STXON instruction strobe
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_STXON(void)
{
	return CC2520_INS_STROBE(CC2520_INS_STXON);
}


/***********************************************************************************
 * @fn      CC2520_STXONCCA
 *
 * @brief   STXONCCA instruction strobe
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_STXONCCA(void)
{
	return CC2520_INS_STROBE(CC2520_INS_STXONCCA);
}


/***********************************************************************************
 * @fn      CC2520_SRFOFF
 *
 * @brief   SRFOFF instruction strobe
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_SRFOFF(void)
{
	return CC2520_INS_STROBE(CC2520_INS_SRFOFF);
}


/***********************************************************************************
 * @fn      CC2520_SXOSCOFF
 *
 * @brief   SXOSCOFF instruction strobe
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_SXOSCOFF(void)
{
	return CC2520_INS_STROBE(CC2520_INS_SXOSCOFF);
}


/***********************************************************************************
 * @fn      CC2520_SFLUSHRX
 *
 * @brief   SFLUSHRX instruction strobe
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_SFLUSHRX(void)
{
	return CC2520_INS_STROBE(CC2520_INS_SFLUSHRX);
}


/***********************************************************************************
 * @fn      CC2520_SFLUSHTX
 *
 * @brief   SFLUSHTX instruction strobe
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_SFLUSHTX(void)
{
	return CC2520_INS_STROBE(CC2520_INS_SFLUSHTX);
}


/***********************************************************************************
 * @fn      CC2520_SACK
 *
 * @brief   SACK instruction strobe
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_SACK(void)
{
	return CC2520_INS_STROBE(CC2520_INS_SACK);
}


/***********************************************************************************
 * @fn      CC2520_SACKPEND
 *
 * @brief   SACKPEND instruction strobe
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_SACKPEND(void)
{
	return CC2520_INS_STROBE(CC2520_INS_SACKPEND);
}


/***********************************************************************************
 * @fn      CC2520_SNACK
 *
 * @brief   SNACK instruction strobe
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_SNACK(void)
{
	return CC2520_INS_STROBE(CC2520_INS_SNACK);
}


/***********************************************************************************
 * @fn      CC2520_SRXMASKBITSET
 *
 * @brief   SRXMASKBITSET instruction strobe
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_SRXMASKBITSET(void)
{
	return CC2520_INS_STROBE(CC2520_INS_SRXMASKBITSET);
}


/***********************************************************************************
 * @fn      CC2520_SRXMASKBITCLR
 *
 * @brief   SRXMASKBITCLR instruction strobe
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_SRXMASKBITCLR(void)
{
	return CC2520_INS_STROBE(CC2520_INS_SRXMASKBITCLR);
}


/***********************************************************************************
 * @fn      CC2520_IBUFLD
 *
 * @brief   IBUFLD - load instruction to instruction buffer
 *
 * @param   uint8 i -
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_IBUFLD(uint8 i)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_IBUFLD);
	CC2520_SPI_TXRX(i);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_SRES
 *
 * @brief   SRES - Reset device except SPI interface
 *
 * @param   none
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_SRES(void)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_SRES);
	CC2520_SPI_TXRX(0x00);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_MEMRD
 *
 * @brief   Read memory
 *
 * @param   uint16 addr
 *          uint16 count
 *          uint8  *pData
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_MEMRD(uint16 addr, uint16 count, uint8  *pData)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_MEMRD | HI_UINT16(addr));
	CC2520_SPI_TXRX(LO_UINT16(addr));
	CC2520_INS_RD_ARRAY(count, pData);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_MEMRD8
 *
 * @brief   Read memory 8 bits
 *
 * @param   uint16 addr
 *
 * @return  uint8 - result
 */
uint8 CC2520_MEMRD8(uint16 addr)
{
	uint8 value;
	CC2520_SPI_BEGIN();
	CC2520_SPI_TXRX(CC2520_INS_MEMRD | HI_UINT16(addr));
	CC2520_SPI_TXRX(LO_UINT16(addr));
	value = CC2520_SPI_TXRX(0x00);
	CC2520_SPI_END();
	return value;
}


/***********************************************************************************
 * @fn      CC2520_MEMRD16
 *
 * @brief   Read memory 16 bits
 *
 * @param   uint16 addr
 *
 * @return  uint16 - result
 */
uint16 CC2520_MEMRD16(uint16 addr)
{
	eword_t value;
	CC2520_SPI_BEGIN();
	CC2520_SPI_TXRX(CC2520_INS_MEMRD | HI_UINT16(addr));
	CC2520_SPI_TXRX(LO_UINT16(addr));
	value.b.b0 = CC2520_SPI_TXRX(0x00);
	value.b.b1 = CC2520_SPI_TXRX(0x00);
	CC2520_SPI_END();
	return value.w;
}


/***********************************************************************************
 * @fn      CC2520_MEMRD24
 *
 * @brief   Read memory 24 bits
 *
 * @param   uint16 addr
 *
 * @return  uint32 - result
 */
uint32 CC2520_MEMRD24(uint16 addr)
{
	edword_t value;
	CC2520_SPI_BEGIN();
	CC2520_SPI_TXRX(CC2520_INS_MEMRD | HI_UINT16(addr));
	CC2520_SPI_TXRX(LO_UINT16(addr));
	value.b.b0 = CC2520_SPI_TXRX(0x00);
	value.b.b1 = CC2520_SPI_TXRX(0x00);
	value.b.b2 = CC2520_SPI_TXRX(0x00);
	value.b.b3 = 0x00;
	CC2520_SPI_END();
	return value.dw;
}


/***********************************************************************************
 * @fn      CC2520_MEMWR
 *
 * @brief   Write memory
 *
 * @param   uint16 addr
 *          uint16 count
 *          uint8  *pData
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_MEMWR(uint16 addr, uint16 count, uint8  *pData)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_MEMWR | HI_UINT16(addr));
	CC2520_SPI_TXRX(LO_UINT16(addr));
	CC2520_INS_WR_ARRAY(count, pData);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_MEMWR8
 *
 * @brief   Write memory 8 bits
 *
 * @param   uint16 addr
 *          uint8 value
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_MEMWR8(uint16 addr, uint8 value)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_MEMWR | HI_UINT16(addr));
	CC2520_SPI_TXRX(LO_UINT16(addr));
	CC2520_SPI_TXRX(value);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_MEMWR16
 *
 * @brief   Write memory 16 bits
 *
 * @param   uint16 addr
 *          uint16 value
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_MEMWR16(uint16 addr, uint16 value)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_MEMWR | HI_UINT16(addr));
	CC2520_SPI_TXRX(LO_UINT16(addr));
	CC2520_SPI_TXRX(LO_UINT16(value));
	CC2520_SPI_TXRX(HI_UINT16(value));
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_MEMWR24
 *
 * @brief   Write memory 24 bits
 *
 * @param   uint16 addr
 *          uint32 value
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_MEMWR24(uint16 addr, uint32 value)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_MEMWR | HI_UINT16(addr));
	CC2520_SPI_TXRX(LO_UINT16(addr));
	CC2520_SPI_TXRX(LO_UINT16(LO_UINT32(value)));
	CC2520_SPI_TXRX(HI_UINT16(LO_UINT32(value)));
	CC2520_SPI_TXRX(LO_UINT16(HI_UINT32(value)));
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_RXBUF
 *
 * @brief   Read bytes from RX buffer
 *
 * @param   uint8 count
 *          uint8  *pData
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_RXBUF(uint8 count, uint8  *pData)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_RXBUF);
	CC2520_INS_RD_ARRAY(count, pData);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_RXBUF8
 *
 * @brief   Read 8 bits from RX buffer
 *
 * @param   none
 *
 * @return  uint8 - result
 */
uint8 CC2520_RXBUF8(void)
{
	uint8 value;
	CC2520_SPI_BEGIN();
	CC2520_SPI_TXRX(CC2520_INS_RXBUF);
	value = CC2520_SPI_TXRX(0x00);
	CC2520_SPI_END();
	return value;
}


/***********************************************************************************
 * @fn      CC2520_RXBUF16
 *
 * @brief   Read 16 bits from RX buffer
 *
 * @param   none
 *
 * @return  uint16 - result
 */
uint16 CC2520_RXBUF16(void)
{
	eword_t value;
	CC2520_SPI_BEGIN();
	CC2520_SPI_TXRX(CC2520_INS_RXBUF);
	value.b.b0 = CC2520_SPI_TXRX(0x00);
	value.b.b1 = CC2520_SPI_TXRX(0x00);
	CC2520_SPI_END();
	return value.w;
}


/***********************************************************************************
 * @fn      CC2520_RXBUFCP_BEGIN
 *
 * @brief   Copy RX buf to memory. Call this routine before CC2520_RXBUFCP_END
 *
 * @param   uint16 addr - copy destination
 *          uint8 *pCurrCount - number of bytes in RX buf
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_RXBUFCP_BEGIN(uint16 addr, uint8 *pCurrCount)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_RXBUFCP);
	if (pCurrCount) {
		*pCurrCount = CC2520_SPI_TXRX(HI_UINT16(addr));
	} else {
		CC2520_SPI_TXRX(HI_UINT16(addr));
	}
	return s;
}


/***********************************************************************************
 * @fn      CC2520_RXBUFCP_END
 *
 * @brief   Copy RX buf to memory and read into buffer.
 *          Call this routine after CC2520_RXBUFCP_BEGIN.
 *
 * @param   uint16 addr - copy destination
 *          uint8 count - number of bytes
 *          uint8  *pData - data buffer
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_RXBUFCP_END(uint16 addr, uint8 count, uint8  *pData)
{
	uint8 s;
	s = CC2520_SPI_TXRX(LO_UINT16(addr));
	CC2520_INS_RD_ARRAY(count, pData);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_RXBUFMOV
 *
 * @brief   RXBUFMOV instruction - moves oldest bytes from RX buffer to the memory
 *          location addr.
 *
 * @param   uint8 pri - instruction priority
 *          uint16 addr - memory location
 *          uint8 count - number of bytes
 *          uint8 *pCurrCount
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_RXBUFMOV(uint8 pri, uint16 addr, uint8 count, uint8 *pCurrCount)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_RXBUFMOV | pri);
	if (pCurrCount) {
		*pCurrCount = CC2520_SPI_TXRX(count);
	} else {
		CC2520_SPI_TXRX(count);
	}
	CC2520_SPI_TXRX(HI_UINT16(addr));
	CC2520_SPI_TXRX(LO_UINT16(addr));
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_TXBUF
 *
 * @brief   Write data to TX buffer
 *
 * @param   uint8 count - number of bytes
 *          uint8  *pData - pointer to data buffer
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_TXBUF(uint8 count, uint8  *pData)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_TXBUF);
	CC2520_INS_WR_ARRAY(count, pData);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_TXBUF8
 *
 * @brief   Write 8 bits to TX buffer
 *
 * @param   uint8 data
 *
 * @return  none
 */
void CC2520_TXBUF8(uint8 data)
{
	CC2520_SPI_BEGIN();
	CC2520_SPI_TXRX(CC2520_INS_TXBUF);
	CC2520_SPI_TXRX(data);
	CC2520_SPI_END();
}


/***********************************************************************************
 * @fn      CC2520_TXBUF16
 *
 * @brief   Write 16 bits to TX buffer
 *
 * @param   uint16 data
 *
 * @return  none
 */
void CC2520_TXBUF16(uint16 data)
{
	CC2520_SPI_BEGIN();
	CC2520_SPI_TXRX(CC2520_INS_TXBUF);
	CC2520_SPI_TXRX(LO_UINT16(data));
	CC2520_SPI_TXRX(HI_UINT16(data));
	CC2520_SPI_END();
}


/***********************************************************************************
 * @fn      CC2520_TXBUFCP
 *
 * @brief   Copy data from memory location addr to TX buf
 *
 * @param   uint8 pri - priority
 *          uint16 addr - memory location
 *          uint8 count - number of bytes
 *          uint8 *pCurrCount
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_TXBUFCP(uint8 pri, uint16 addr, uint8 count, uint8 *pCurrCount)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_TXBUFCP | pri);
	if (pCurrCount) {
		*pCurrCount = CC2520_SPI_TXRX(count);
	} else {
		CC2520_SPI_TXRX(count);
	}
	CC2520_SPI_TXRX(HI_UINT16(addr));
	CC2520_SPI_TXRX(LO_UINT16(addr));
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_RANDOM
 *
 * @brief   Random generated bytes
 *
 * @param   uint8 count
 *          uint8  *pData
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_RANDOM(uint8 count, uint8  *pData)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_RANDOM);
	CC2520_SPI_TXRX(0x00);
	CC2520_INS_RD_ARRAY(count, pData);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_RANDOM8
 *
 * @brief   Random generated byte
 *
 * @param  none
 *
 * @return  uint8
 */
uint8 CC2520_RANDOM8(void)
{
	uint8 value;
	CC2520_SPI_BEGIN();
	CC2520_SPI_TXRX(CC2520_INS_RANDOM);
	CC2520_SPI_TXRX(0x00);
	value = CC2520_SPI_TXRX(0x00);
	CC2520_SPI_END();
	return value;
}


/***********************************************************************************
 * @fn      CC2520_RANDOM16
 *
 * @brief   Random generated bytes
 *
 * @param  none
 *
 * @return  uint16
 */
uint16 CC2520_RANDOM16(void)
{
	eword_t value;
	CC2520_SPI_BEGIN();
	CC2520_SPI_TXRX(CC2520_INS_RANDOM);
	CC2520_SPI_TXRX(0x00);
	value.b.b0 = CC2520_SPI_TXRX(0x00);
	value.b.b1 = CC2520_SPI_TXRX(0x00);
	CC2520_SPI_END();
	return value.w;
}


/***********************************************************************************
 * @fn      CC2520_RXMASKOR
 *
 * @brief   RXMASKOR instruction - bitwise OR between RX enable mask and orMask
 *
 * @param  uint16 orMask -
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_RXMASKOR(uint16 orMask)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_RXMASKOR);
	CC2520_SPI_TXRX(HI_UINT16(orMask));
	CC2520_SPI_TXRX(LO_UINT16(orMask));
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_RXMASKAND
 *
 * @brief   RXMASKAND - bitwise AND between RX enable mask and andMask
 *
 * @param  uint16 andMask
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_RXMASKAND(uint16 andMask)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_RXMASKAND);
	CC2520_SPI_TXRX(HI_UINT16(andMask));
	CC2520_SPI_TXRX(LO_UINT16(andMask));
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_MEMCP
 *
 * @brief   MEMCP - Copy data between memory blocks
 *
 * @param  uint8 pri - priority
 *         uint16 count - number of bytes
 *         uint16 src - source address
 *         uint16 dest - destination address
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_MEMCP(uint8 pri, uint16 count, uint16 src, uint16 dest)
{
	return CC2520_INS_MEMCP_COMMON(CC2520_INS_MEMCP, pri, count, src, dest);
}


/***********************************************************************************
 * @fn      CC2520_MEMCPR
 *
 * @brief   MEMCPR - Copy data between memory blocks and revert endianess
 *
 * @param  uint8 pri - priority
 *         uint16 count - number of bytes
 *         uint16 src - source address
 *         uint16 dest - destination address
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_MEMCPR(uint8 pri, uint16 count, uint16 src, uint16 dest)
{
	return CC2520_INS_MEMCP_COMMON(CC2520_INS_MEMCPR, pri, count, src, dest);
}


/***********************************************************************************
 * @fn      CC2520_MEMXCP
 *
 * @brief   MEMXCP - XOR one memory block with another memory block
 *
 * @param  uint8 pri - priority
 *         uint16 count - number of bytes
 *         uint16 src - source address
 *         uint16 dest - destination address
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_MEMXCP(uint8 pri, uint16 count, uint16 src, uint16 dest)
{
	return CC2520_INS_MEMCP_COMMON(CC2520_INS_MEMXCP, pri, count, src, dest);
}


/***********************************************************************************
 * @fn      CC2520_MEMXWR
 *
 * @brief   MEMXWR - XOR memory
 *
 * @param  uint16 addr - memory address
 *         uint16 count - number of bytes
 *         uint8  *pData - unaltered data
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_MEMXWR(uint16 addr, uint16 count, uint8  *pData)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_MEMXWR);
	CC2520_SPI_TXRX(HI_UINT16(addr));
	CC2520_SPI_TXRX(LO_UINT16(addr));
	CC2520_INS_WR_ARRAY(count, pData);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_BSET
 *
 * @brief   BSET - set a single bit
 *
 * @param  uint8 bitAddr - address
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_BSET(uint8 bitAddr)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_BSET);
	CC2520_SPI_TXRX(bitAddr);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_BCLR
 *
 * @brief   BCLR - Clear a single bit
 *
 * @param  uint8 bitAddr - address
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_BCLR(uint8 bitAddr)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_BCLR);
	CC2520_SPI_TXRX(bitAddr);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_CTR
 *
 * @brief   CTR - Counter mode encryption
 *
 * @param  uint8 pri - priority
 *         uint8 k - address of key
 *         uint8 c - number of plaintext bytes
 *         uint8 n - address of nonce
 *         uint16 src - starting address of plaintext
 *         uint16 dest - destination address, if dest=0 destination address is set
 *                       equal to src address.
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_CTR(uint8 pri, uint8 k, uint8 c, uint8 n, uint16 src, uint16 dest)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_CTR | pri);
	CC2520_SPI_TXRX(k);
	CC2520_SPI_TXRX(c);
	CC2520_SPI_TXRX(n);
	CC2520_SPI_TXRX((HI_UINT16(src) << 4) | HI_UINT16(dest));
	CC2520_SPI_TXRX(LO_UINT16(src));
	CC2520_SPI_TXRX(LO_UINT16(dest));
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_UCTR
 *
 * @brief   UCTR - Counter mode decryption.
 *
 * @param  uint8 pri - priority
 *         uint8 k - address of key
 *         uint8 c - number of plaintext bytes
 *         uint8 n - address of nonce
 *         uint16 src - starting address of ciphertext
 *         uint16 dest - destination address
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_UCTR(uint8 pri, uint8 k, uint8 c, uint8 n, uint16 src, uint16 dest)
{
	return CC2520_CTR(pri, k, c, n, src, dest);
}


/***********************************************************************************
 * @fn     CC2520_CBCMAC
 *
 * @brief  CBCMAC - authentication using CBC-MAC
 *
 * @param  uint8 pri - priority
 *         uint8 k - address of key
 *         uint8 c - number of plaintext bytes
 *         uint16 src - starting address of plaintext
 *         uint16 dest - destination address
 *         uint8 m - sets length of integrity code (m=1,2,3 gives lenght of integrity
 *                   field 4,8,16)
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_CBCMAC(uint8 pri, uint8 k, uint8 c, uint16 src, uint16 dest, uint8 m)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_CBCMAC | pri);
	CC2520_SPI_TXRX(k);
	CC2520_SPI_TXRX(c);
	CC2520_SPI_TXRX((HI_UINT16(src) << 4) | HI_UINT16(dest));
	CC2520_SPI_TXRX(LO_UINT16(src));
	CC2520_SPI_TXRX(LO_UINT16(dest));
	CC2520_SPI_TXRX(m);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn     CC2520_UCBCMAC
 *
 * @brief  UCBCMAC - reverse authentication using CBC-MAC
 *
 * @param  uint8 pri - priority
 *         uint8 k - address of key
 *         uint8 c - number of plaintext bytes
 *         uint16 src - starting address of plaintext
 *         uint8 m - sets length of integrity code (m=1,2,3 gives lenght of integrity
 *                   field 4,8,16)
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_UCBCMAC(uint8 pri, uint8 k, uint8 c, uint16 src, uint8 m)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_UCBCMAC | pri);
	CC2520_SPI_TXRX(k);
	CC2520_SPI_TXRX(c);
	CC2520_SPI_TXRX(HI_UINT16(src));
	CC2520_SPI_TXRX(LO_UINT16(src));
	CC2520_SPI_TXRX(m);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_CCM
 *
 * @brief   CCM - encryption and authentication using CCM
 *
 * @param  uint8 pri - priority
 *         uint8 k - address of key
 *         uint8 c - number of plaintext bytes to authenticate and encrypt
 *         uint8 n - address of nonce
 *         uint16 src - starting address of plaintext
 *         uint16 dest - destination address
 *         uint8 f - number of plaintext bytes to authenticate
 *         uint8 m - sets length of integrity code (m=1,2,3 gives lenght of integrity
 *                   field 4,8,16)
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_CCM(uint8 pri, uint8 k, uint8 c, uint8 n, uint16 src, \
		uint16 dest, uint8 f, uint8 m)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_CCM | pri);
	CC2520_SPI_TXRX(k);
	CC2520_SPI_TXRX(c);
	CC2520_SPI_TXRX(n);
	CC2520_SPI_TXRX((HI_UINT16(src) << 4) | HI_UINT16(dest));
	CC2520_SPI_TXRX(LO_UINT16(src));
	CC2520_SPI_TXRX(LO_UINT16(dest));
	CC2520_SPI_TXRX(f);
	CC2520_SPI_TXRX(m);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_UCCM
 *
 * @brief   UCCM - decryption and reverse authentication using CCM
 *
 * @param  uint8 pri - priority
 *         uint8 k - address of key
 *         uint8 c - number of bytes to decrypt and authenticate
 *         uint8 n - address of nonce
 *         uint16 src - starting address
 *         uint16 dest - destination address
 *         uint8 f - number of bytes to authenticate
 *         uint8 m - ets length of integrity code (m=1,2,3 gives lenght of integrity
 *                   field 4,8,16)
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_UCCM(uint8 pri, uint8 k, uint8 c, uint8 n, uint16 src, \
		uint16 dest, uint8 f, uint8 m)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_UCCM | pri);
	CC2520_SPI_TXRX(k);
	CC2520_SPI_TXRX(c);
	CC2520_SPI_TXRX(n);
	CC2520_SPI_TXRX((HI_UINT16(src) << 4) | HI_UINT16(dest));
	CC2520_SPI_TXRX(LO_UINT16(src));
	CC2520_SPI_TXRX(LO_UINT16(dest));
	CC2520_SPI_TXRX(f);
	CC2520_SPI_TXRX(m);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_ECB
 *
 * @brief   ECB encryption
 *
 * @param  uint8 pri - priority
 *         uint8 k - address of key
 *         uint8 c - encrypts 16-C bytes of data in a 16 byte block
 *         uint16 src - source address
 *         uint16 dest - destination address
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_ECB(uint8 pri, uint8 k, uint8 c, uint16 src, uint16 dest)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_ECB | pri);
	CC2520_SPI_TXRX(k);
	CC2520_SPI_TXRX((c << 4) | HI_UINT16(src));
	CC2520_SPI_TXRX(LO_UINT16(src));
	CC2520_SPI_TXRX(HI_UINT16(dest));
	CC2520_SPI_TXRX(LO_UINT16(dest));
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_ECBO
 *
 * @brief   ECB encryption overwriting plaintext
 *
 * @param  uint8 pri - priority
 *         uint8 k - address of key
 *         uint8 c - encrypts 16-C bytes of data in a 16 byte block
 *         uint16 addr - source and destination address
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_ECBO(uint8 pri, uint8 k, uint8 c, uint16 addr)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_ECBO | pri);
	CC2520_SPI_TXRX(k);
	CC2520_SPI_TXRX((c << 4) | HI_UINT16(addr));
	CC2520_SPI_TXRX(LO_UINT16(addr));
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_ECBX
 *
 * @brief   ECB encryption and XOR
 *
 * @param  uint8 pri - priority
 *         uint8 k - address of key
 *         uint8 c - encrypts 16-C bytes of data in a 16 byte block
 *         uint16 src - source address
 *         uint16 dest - destination address
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_ECBX(uint8 pri, uint8 k, uint8 c, uint16 src, uint16 dest)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_ECBX | pri);
	CC2520_SPI_TXRX(k);
	CC2520_SPI_TXRX((c << 4) | HI_UINT16(src));
	CC2520_SPI_TXRX(LO_UINT16(src));
	CC2520_SPI_TXRX(HI_UINT16(dest));
	CC2520_SPI_TXRX(LO_UINT16(dest));
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_ECBXO
 *
 * @brief   ECBO and XOR
 *
 * @param  uint8 pri - priority
 *         uint8 k - address of key
 *         uint8 c - encrypts 16-C bytes of data in a 16 byte block
 *         uint16 addr - source and destination address
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_ECBXO(uint8 pri, uint8 k, uint8 c, uint16 addr)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_ECBXO | pri);
	CC2520_SPI_TXRX(k);
	CC2520_SPI_TXRX((c << 4) | HI_UINT16(addr));
	CC2520_SPI_TXRX(LO_UINT16(addr));
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_INC
 *
 * @brief   INC - increment instruction. Increments 2^c byte word with least
 *                significant byte at address.
 *
 * @param  uint8 pri - priority
 *         uint8 c - increments 2^c byte word
 *         uint16 addr - address
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_INC(uint8 pri, uint8 c, uint16 addr)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_INC | pri);
	CC2520_SPI_TXRX((c << 4) | HI_UINT16(addr));
	CC2520_SPI_TXRX(LO_UINT16(addr));
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_ABORT
 *
 * @brief   Abort ongoing data management or security instruction
 *
 * @param  uint8 c - abort mode (see datasheet)
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_ABORT(uint8 c)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_ABORT);
	CC2520_SPI_TXRX(c);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_REGRD
 *
 * @brief   Register read. Can only be started from addresses below 0x40
 *
 * @param  uint8 addr - address
 *         uint8 count - number of bytes
 *         uint8  *pValues - buffer to store result
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_REGRD(uint8 addr, uint8 count, uint8  *pValues)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_REGRD | addr);
	CC2520_INS_RD_ARRAY(count, pValues);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_REGRD8
 *
 * @brief   Read one register byte
 *
 * @param  uint8 addr - address
 *
 * @return  uint8 - result
 */
uint8 CC2520_REGRD8(uint8 addr)
{
	uint8 value;
	CC2520_SPI_BEGIN();
	CC2520_SPI_TXRX(CC2520_INS_REGRD | addr);
	value = CC2520_SPI_TXRX(0x00);
	CC2520_SPI_END();
	return value;
}


/***********************************************************************************
 * @fn      CC2520_REGRD16
 *
 * @brief   Read two register bytes
 *
 * @param  uint8 addr - address
 *
 * @return  uint16 - result
 */
uint16 CC2520_REGRD16(uint8 addr)
{
	eword_t value;
	CC2520_SPI_BEGIN();
	CC2520_SPI_TXRX(CC2520_INS_REGRD | addr);
	value.b.b0 = CC2520_SPI_TXRX(0x00);
	value.b.b1 = CC2520_SPI_TXRX(0x00);
	CC2520_SPI_END();
	return value.w;
}


/***********************************************************************************
 * @fn      CC2520_REGRD24
 *
 * @brief   Read three register bytes
 *
 * @param  uint8 addr - address
 *
 * @return  uint32 - result
 */
uint32 CC2520_REGRD24(uint8 addr)
{
	edword_t value;
	CC2520_SPI_BEGIN();
	CC2520_SPI_TXRX(CC2520_INS_REGRD | addr);
	value.b.b0 = CC2520_SPI_TXRX(0x00);
	value.b.b1 = CC2520_SPI_TXRX(0x00);
	value.b.b2 = CC2520_SPI_TXRX(0x00);
	value.b.b3 = 0x00;
	CC2520_SPI_END();
	return value.dw;
}


/***********************************************************************************
 * @fn      CC2520_REGWR
 *
 * @brief   Register write. Can only be started from addresses below 0x40
 *
 * @param  uint8 addr - address
 *         uint8 count - number of bytes
 *         uint8  *pValues - data buffer
 *
 * @return  uint8 - status byte
 */
uint8 CC2520_REGWR(uint8 addr, uint8 count, uint8  *pValues)
{
	uint8 s;
	CC2520_SPI_BEGIN();
	s = CC2520_SPI_TXRX(CC2520_INS_REGWR | addr);
	CC2520_INS_WR_ARRAY(count, pValues);
	CC2520_SPI_END();
	return s;
}


/***********************************************************************************
 * @fn      CC2520_REGWR8
 *
 * @brief   Write one register byte
 *
 * @param  uint8 addr - address
 *         uint8 value
 *
 * @return  none
 */
void CC2520_REGWR8(uint8 addr, uint8 value)
{
	CC2520_SPI_BEGIN();
	CC2520_SPI_TXRX(CC2520_INS_REGWR | addr);
	CC2520_SPI_TXRX(value);
	CC2520_SPI_END();
	return;
}


/***********************************************************************************
 * @fn      CC2520_REGWR16
 *
 * @brief   Write two register bytes
 *
 * @param  uint8 addr - address
 *         uint16 value
 *
 * @return  none
 */
void CC2520_REGWR16(uint8 addr, uint16 value)
{
	CC2520_SPI_BEGIN();
	CC2520_SPI_TXRX(CC2520_INS_REGWR | addr);
	CC2520_SPI_TXRX(LO_UINT16(value));
	CC2520_SPI_TXRX(HI_UINT16(value));
	CC2520_SPI_END();
}


/***********************************************************************************
 * @fn      CC2520_REGWR24
 *
 * @brief   Write three register bytes
 *
 * @param  uint8 addr
 *         uint32 value
 *
 * @return  none
 */
void CC2520_REGWR24(uint8 addr, uint32 value)
{
	CC2520_SPI_BEGIN();
	CC2520_SPI_TXRX(CC2520_INS_REGWR | addr);
	CC2520_SPI_TXRX(LO_UINT16(LO_UINT32(value)));
	CC2520_SPI_TXRX(HI_UINT16(LO_UINT32(value)));
	CC2520_SPI_TXRX(LO_UINT16(HI_UINT32(value)));
	CC2520_SPI_END();
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

