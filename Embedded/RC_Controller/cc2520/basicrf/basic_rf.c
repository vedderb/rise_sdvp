/***********************************************************************************
  Filename:     basic_rf.c

  Description:  Basic RF library

 ***********************************************************************************/

/***********************************************************************************
 * INCLUDES
 */
#include "hal_rf.h"
#ifdef SECURITY_CCM
#include "hal_rf_security.h"
#endif

#include "basic_rf.h"
#ifdef SECURITY_CCM
#include "basic_rf_security.h"
#endif

#include "util.h"               // Using min()
#include <string.h>

/***********************************************************************************
 * CONSTANTS AND DEFINES
 */

// Packet and packet part lengths
#define PKT_LEN_MIC                         8
#define PKT_LEN_SEC                         PKT_LEN_UNSEC + PKT_LEN_MIC
#define PKT_LEN_AUTH                        8
#define PKT_LEN_ENCR                        24

// Packet overhead ((frame control field, sequence number, PAN ID,
// destination and source) + (footer))
// Note that the length byte itself is not included included in the packet length
#define BASIC_RF_PACKET_OVERHEAD_SIZE       ((2 + 1 + 2 + 2 + 2) + (2))
#define BASIC_RF_MAX_PAYLOAD_SIZE	        (127 - BASIC_RF_PACKET_OVERHEAD_SIZE - \
		BASIC_RF_AUX_HDR_LENGTH - BASIC_RF_LEN_MIC)
#define BASIC_RF_ACK_PACKET_SIZE	        5
#define BASIC_RF_FOOTER_SIZE                2
#define BASIC_RF_HDR_SIZE                   10

// The time it takes for the acknowledgment packet to be received after the
// data packet has been transmitted.
#define BASIC_RF_ACK_DURATION		        (0.5 * 32 * 2 * ((4 + 1) + (1) + (2 + 1) + (2)))
#define BASIC_RF_SYMBOL_DURATION	        (32 * 0.5)

// The length byte
#define BASIC_RF_PLD_LEN_MASK               0x7F

// Frame control field
#define BASIC_RF_FCF_NOACK                  0x8841
#define BASIC_RF_FCF_ACK                    0x8861
#define BASIC_RF_FCF_ACK_BM                 0x0020
#define BASIC_RF_FCF_BM                     (~BASIC_RF_FCF_ACK_BM)
#define BASIC_RF_SEC_ENABLED_FCF_BM         0x0008

// Frame control field LSB
#define BASIC_RF_FCF_NOACK_L                LO_UINT16(BASIC_RF_FCF_NOACK)
#define BASIC_RF_FCF_ACK_L                  LO_UINT16(BASIC_RF_FCF_ACK)
#define BASIC_RF_FCF_ACK_BM_L               LO_UINT16(BASIC_RF_FCF_ACK_BM)
#define BASIC_RF_FCF_BM_L                   LO_UINT16(BASIC_RF_FCF_BM)
#define BASIC_RF_SEC_ENABLED_FCF_BM_L       LO_UINT16(BASIC_RF_SEC_ENABLED_FCF_BM)

// Auxiliary Security header
#define BASIC_RF_AUX_HDR_LENGTH             5
#define BASIC_RF_LEN_AUTH                   BASIC_RF_PACKET_OVERHEAD_SIZE + \
		BASIC_RF_AUX_HDR_LENGTH - BASIC_RF_FOOTER_SIZE
#define BASIC_RF_SECURITY_M                 2
#define BASIC_RF_LEN_MIC                    8
#ifdef SECURITY_CCM
#undef BASIC_RF_HDR_SIZE
#define BASIC_RF_HDR_SIZE                   15
#endif

// Footer
#define BASIC_RF_CRC_OK_BM                  0x80

/***********************************************************************************
 * TYPEDEFS
 */
// The receive struct
typedef struct {
	uint8 seqNumber;
	uint16 srcAddr;
	uint16 srcPanId;
	int8 length;
	uint8* pPayload;
	uint8 ackRequest;
	int8 rssi;
	volatile uint8 isReady;
	uint8 status;
} basicRfRxInfo_t;

// Tx state
typedef struct {
	uint8 txSeqNumber;
	volatile uint8 ackReceived;
	uint8 receiveOn;
	uint32 frameCounter;
} basicRfTxState_t;


// Basic RF packet header (IEEE 802.15.4)
typedef struct {
	uint8   packetLength;
	uint8   fcf0;           // Frame control field LSB
	uint8   fcf1;           // Frame control field MSB
	uint8   seqNumber;
	uint16  panId;
	uint16  destAddr;
	uint16  srcAddr;
#ifdef SECURITY_CCM
	uint8   securityControl;
	uint8  frameCounter[4];
#endif
} basicRfPktHdr_t;


/***********************************************************************************
 * LOCAL VARIABLES
 */
static volatile basicRfRxInfo_t  rxi; // Make sure sequence numbers are
static volatile basicRfTxState_t txState; // initialised and distinct.

static volatile basicRfCfg_t* pConfig;
static uint8 txMpdu[BASIC_RF_MAX_PAYLOAD_SIZE+BASIC_RF_PACKET_OVERHEAD_SIZE+1];
static uint8 rxMpdu[128];
static mutex_t rf_mutex;
static mutex_t rf_send_mutex;
static THD_WORKING_AREA(basicRf_thread_wa, 256);

/***********************************************************************************
 * GLOBAL VARIABLES
 */


/***********************************************************************************
 * LOCAL FUNCTIONS
 */

/**
 * This thread only checks if the CC2520 has gotten the RX_OVERFLOW exception and
 * flushes the rx buffer in that case. This is because it will stop receiving
 * packets otherwise.
 */
static THD_FUNCTION(basicRf_thread, arg) {
	(void)arg;

	chRegSetThreadName("CC2520 EXC");

	for(;;) {
		if (CC2520_RX_OVERFLOW_PIN) {
			chMtxLock(&rf_mutex);
			halRfFlushRx();
			CC2520_CLEAR_EXC(CC2520_EXC_RX_OVERFLOW);
			chMtxUnlock(&rf_mutex);
		}

		chThdSleepMilliseconds(1);
	}
}

/***********************************************************************************
 * @fn          basicRfBuildHeader
 *
 * @brief       Builds packet header according to IEEE 802.15.4 frame format
 *
 * @param       buffer - Pointer to buffer to write the header
 *              destAddr - destination short address
 *              payloadLength - length of higher layer payload
 *
 * @return      uint8 - length of header
 */
static uint8 basicRfBuildHeader(uint8* buffer, uint16 destAddr, uint8 payloadLength) {
	basicRfPktHdr_t *pHdr;
	uint16 fcf;

	pHdr= (basicRfPktHdr_t*)buffer;

	// Populate packet header
	pHdr->packetLength = payloadLength + BASIC_RF_PACKET_OVERHEAD_SIZE;
	//pHdr->frameControlField = pConfig->ackRequest ? BASIC_RF_FCF_ACK : BASIC_RF_FCF_NOACK;
	fcf= pConfig->ackRequest ? BASIC_RF_FCF_ACK : BASIC_RF_FCF_NOACK;
	pHdr->fcf0 = LO_UINT16(fcf);
	pHdr->fcf1 = HI_UINT16(fcf);
	pHdr->seqNumber= txState.txSeqNumber;
	pHdr->panId= pConfig->panId;
	pHdr->destAddr= destAddr;
	pHdr->srcAddr= pConfig->myAddr;

#ifdef SECURITY_CCM

	// Add security to FCF, length and security header
	pHdr->fcf0 |= BASIC_RF_SEC_ENABLED_FCF_BM_L;
	pHdr->packetLength += PKT_LEN_MIC;
	pHdr->packetLength += BASIC_RF_AUX_HDR_LENGTH;

	pHdr->securityControl= SECURITY_CONTROL;
	pHdr->frameCounter[0]=   LO_UINT16(LO_UINT32(txState.frameCounter));
	pHdr->frameCounter[1]=   HI_UINT16(LO_UINT32(txState.frameCounter));
	pHdr->frameCounter[2]=   LO_UINT16(HI_UINT32(txState.frameCounter));
	pHdr->frameCounter[3]=   HI_UINT16(HI_UINT32(txState.frameCounter));

#endif

	// Make sure bytefields are network byte order
	UINT16_HTON(pHdr->panId);
	UINT16_HTON(pHdr->destAddr);
	UINT16_HTON(pHdr->srcAddr);

	return BASIC_RF_HDR_SIZE;
}


/***********************************************************************************
 * @fn          basicRfBuildMpdu
 *
 * @brief       Builds mpdu (MAC header + payload) according to IEEE 802.15.4
 *              frame format
 *
 * @param       destAddr - Destination short address
 *              pPayload - pointer to buffer with payload
 *              payloadLength - length of payload buffer
 *
 * @return      uint8 - length of mpdu
 */
static uint8 basicRfBuildMpdu(uint16 destAddr, uint8* pPayload, uint8 payloadLength) {
	uint8 hdrLength, n;

	hdrLength = basicRfBuildHeader(txMpdu, destAddr, payloadLength);

	for(n=0;n<payloadLength;n++) {
		txMpdu[hdrLength+n] = pPayload[n];
	}

	return hdrLength + payloadLength; // total mpdu length
}


/***********************************************************************************
 * @fn          basicRfRxFrmDoneIsr
 *
 * @brief       Interrupt service routine for received frame from radio
 *              (either data or acknowlegdement)
 *
 * @param       rxi - file scope variable info extracted from the last incoming
 *                    frame
 *              txState - file scope variable that keeps tx state info
 *
 * @return      none
 */
static void basicRfRxFrmDoneIsr(void) {
	basicRfPktHdr_t *pHdr;
	uint8 *pStatusWord;
#ifdef SECURITY_CCM
	uint8 authStatus=0;
#endif

	chMtxLock(&rf_mutex);

	// Map header to packet buffer
	pHdr= (basicRfPktHdr_t*)rxMpdu;

	// Clear interrupt
	CLEAR_EXC_RX_FRM_DONE();

	// Read payload length.
	halRfReadRxBuf(&pHdr->packetLength,1);
	pHdr->packetLength &= BASIC_RF_PLD_LEN_MASK; // Ignore MSB

	// Is this an acknowledgment packet?
	// Only ack packets may be 5 bytes in total.
	if (pHdr->packetLength == BASIC_RF_ACK_PACKET_SIZE) {

		// Read the packet
		halRfReadRxBuf(&rxMpdu[1], pHdr->packetLength);

		// Make sure byte fields are changed from network to host byte order
		UINT16_NTOH(pHdr->panId);
		UINT16_NTOH(pHdr->destAddr);
		UINT16_NTOH(pHdr->srcAddr);
#ifdef SECURITY_CCM
		UINT32_NTOH(pHdr->frameCounter);
#endif

		rxi.ackRequest = !!(pHdr->fcf0 & BASIC_RF_FCF_ACK_BM_L);

		// Read the status word and check for CRC OK
		pStatusWord= rxMpdu + 4;

		// Indicate the successful ACK reception if CRC and sequence number OK
		if ((pStatusWord[1] & BASIC_RF_CRC_OK_BM) && (pHdr->seqNumber == txState.txSeqNumber)) {
			txState.ackReceived = TRUE;
		}

		// No, it is data
	} else {

		// It is assumed that the radio rejects packets with invalid length.
		// Subtract the number of bytes in the frame overhead to get actual payload.

		rxi.length = pHdr->packetLength - BASIC_RF_PACKET_OVERHEAD_SIZE;

#ifdef SECURITY_CCM
		rxi.length -= (BASIC_RF_AUX_HDR_LENGTH + BASIC_RF_LEN_MIC);
		authStatus = halRfReadRxBufSecure(&rxMpdu[1], pHdr->packetLength, rxi.length,
				BASIC_RF_LEN_AUTH, BASIC_RF_SECURITY_M);
#else
		halRfReadRxBuf(&rxMpdu[1], pHdr->packetLength);
#endif

		// Make sure byte fields are changed from network to host byte order
		UINT16_NTOH(pHdr->panId);
		UINT16_NTOH(pHdr->destAddr);
		UINT16_NTOH(pHdr->srcAddr);
#ifdef SECURITY_CCM
		UINT32_NTOH(pHdr->frameCounter);
#endif

		rxi.ackRequest = !!(pHdr->fcf0 & BASIC_RF_FCF_ACK_BM_L);

		// Read the source address
		rxi.srcAddr= pHdr->srcAddr;

		// Read the packet payload
		rxi.pPayload = rxMpdu + BASIC_RF_HDR_SIZE;

		// Read the FCS to get the RSSI and CRC
		pStatusWord= rxi.pPayload+rxi.length;
#ifdef SECURITY_CCM
		pStatusWord+= BASIC_RF_LEN_MIC;
#endif
		rxi.rssi = pStatusWord[0];

		// Notify the application about the received data packet if the CRC is OK
		// Throw packet if the previous packet had the same sequence number
		if( (pStatusWord[1] & BASIC_RF_CRC_OK_BM) && (rxi.seqNumber != pHdr->seqNumber) ) {
			// If security is used check also that authentication passed
#ifdef SECURITY_CCM
			if( authStatus==SUCCESS ) {
				if ( (pHdr->fcf0 & BASIC_RF_FCF_BM_L) ==
						(BASIC_RF_FCF_NOACK_L | BASIC_RF_SEC_ENABLED_FCF_BM_L)) {
					rxi.isReady = TRUE;
				}
			}
#else
			if ( ((pHdr->fcf0 & (BASIC_RF_FCF_BM_L)) == BASIC_RF_FCF_NOACK_L) ) {
				rxi.isReady = TRUE;
			}
#endif
		}
		rxi.seqNumber = pHdr->seqNumber;
	}

	chMtxUnlock(&rf_mutex);
}


/***********************************************************************************
 * GLOBAL FUNCTIONS
 */

/***********************************************************************************
 * @fn          basicRfInit
 *
 * @brief       Initialise basic RF datastructures. Sets channel, short address and
 *              PAN id in the chip and configures interrupt on packet reception
 *
 * @param       pRfConfig - pointer to BASIC_RF_CONFIG struct.
 *                          This struct must be allocated by higher layer
 *              txState - file scope variable that keeps tx state info
 *              rxi - file scope variable info extracted from the last incoming
 *                    frame
 *
 * @return      none
 */
uint8 basicRfInit(basicRfCfg_t* pRfConfig) {
	if (halRfInit() == FAILED) {
		return FAILED;
	}

	chMtxObjectInit(&rf_mutex);
	chMtxObjectInit(&rf_send_mutex);

	// Set the protocol configuration
	pConfig = pRfConfig;
	rxi.pPayload   = NULL;

	txState.receiveOn = TRUE;
	txState.frameCounter = 0;

	// Set channel
	halRfSetChannel(pConfig->channel);

	// Write the short address and the PAN ID to the CC2520 RAM
	halRfSetShortAddr(pConfig->myAddr);
	halRfSetPanId(pConfig->panId);

	// if security is enabled, write key and nonce
#ifdef SECURITY_CCM
	basicRfSecurityInit((basicRfCfg_t*)pConfig);
#endif

	// Set up receive interrupt (received data or acknowlegment)
	halRfRxInterruptConfig(basicRfRxFrmDoneIsr);

	chThdCreateStatic(basicRf_thread_wa, sizeof(basicRf_thread_wa), NORMALPRIO, basicRf_thread, NULL);

	return SUCCESS;
}

/***********************************************************************************
 * @fn          basicRfSendPacket
 *
 * @brief       Send packet
 *
 * @param       destAddr - destination short address
 *              pPayload - pointer to payload buffer. This buffer must be
 *                         allocated by higher layer.
 *              length - length of payload
 *              txState - file scope variable that keeps tx state info
 *              mpdu - file scope variable. Buffer for the frame to send
 *
 * @return      basicRFStatus_t - SUCCESS or FAILED
 */
uint8 basicRfSendPacket(uint16 destAddr, uint8* pPayload, uint8 length) {
	uint8 mpduLength;
	uint8 status = SUCCESS;

	chMtxLock(&rf_send_mutex);

	// Turn on receiver if its not on
	if(!txState.receiveOn) {
		basicRfReceiveOn();
	}

	// Check packet length
	length = min(length, BASIC_RF_MAX_PAYLOAD_SIZE);
	mpduLength = basicRfBuildMpdu(destAddr, pPayload, length);

	// Wait until the transceiver is idle
	chMtxLock(&rf_mutex);
	halRfWaitTransceiverReady();

#ifdef SECURITY_CCM
	halRfWriteTxBufSecure(txMpdu, mpduLength, length, BASIC_RF_LEN_AUTH, BASIC_RF_SECURITY_M);
	txState.frameCounter++;     // Increment frame counter field
#else
	halRfWriteTxBuf(txMpdu, mpduLength);
#endif

	// Send frame with CCA. return FAILED if not successful
	if(halRfTransmit() != SUCCESS) {
		status = FAILED;
	}

	chMtxUnlock(&rf_mutex);

	// Wait for the acknowledge to be received, if any
	if (pConfig->ackRequest && status == SUCCESS) {
		txState.ackReceived = FALSE;

		// We'll enter RX automatically, so just wait until we can be sure that the ack reception should have finished
		// The timeout consists of a 12-symbol turnaround time, the ack packet duration, and a small margin
		// TODO: Sleep may be to un-accurate?
		//		halMcuWaitUs((12 * BASIC_RF_SYMBOL_DURATION) + (BASIC_RF_ACK_DURATION) + (2 * BASIC_RF_SYMBOL_DURATION) + 10);
		chThdSleepMicroseconds((12 * BASIC_RF_SYMBOL_DURATION) + (BASIC_RF_ACK_DURATION) + (2 * BASIC_RF_SYMBOL_DURATION) + 10);

		// If an acknowledgment has been received (by RxFrmDoneIsr), the ackReceived flag should be set
		status = txState.ackReceived ? SUCCESS : FAILED;
	}

	// Turn off the receiver if it should not continue to be enabled
	if (!txState.receiveOn) {
		basicRfReceiveOff();
	}

	txState.txSeqNumber++;

#ifdef SECURITY_CCM
	chMtxLock(&rf_mutex);
	halRfIncNonceTx();          // Increment nonce value
	chMtxUnlock();
#endif

	chMtxUnlock(&rf_send_mutex);

	return status;
}


/***********************************************************************************
 * @fn          basicRfPacketIsReady
 *
 * @brief       Check if a new packet is ready to be read by next higher layer
 *
 * @param       none
 *
 * @return      uint8 - TRUE if a packet is ready to be read by higher layer
 */
uint8 basicRfPacketIsReady(void) {
	return rxi.isReady;
}


/**********************************************************************************
 * @fn          basicRfReceive
 *
 * @brief       Copies the payload of the last incoming packet into a buffer
 *
 * @param       pRxData - pointer to data buffer to fill. This buffer must be
 *                        allocated by higher layer.
 *              len - Number of bytes to read in to buffer
 *              rxi - file scope variable holding the information of the last
 *                    incoming packet
 *
 * @return      uint8 - number of bytes actually copied into buffer
 */
uint8 basicRfReceive(uint8* pRxData, uint8 len, int16* pRssi)
{
	// Accessing shared variables -> this is a critical region
	// Critical region start
	chMtxLock(&rf_mutex);

	memcpy(pRxData, rxi.pPayload, min(rxi.length, len));
	if(pRssi != NULL) {
		*pRssi = basicRfGetRssi();
	}
	rxi.isReady = FALSE;

	// Critical region end
	chMtxUnlock(&rf_mutex);

	return min(rxi.length, len);
}


uint16 basicRfGetSouceAddress(void) {
	return rxi.srcAddr;
}

/**********************************************************************************
 * @fn          basicRfGetRssi
 *
 * @brief       Copies the payload of the last incoming packet into a buffer
 *
 * @param       none

 * @return      int8 - RSSI value
 */
int8 basicRfGetRssi(void) {
	return rxi.rssi - halRfGetRssiOffset();
}

/***********************************************************************************
 * @fn          basicRfReceiveOn
 *
 * @brief       Turns on receiver on radio
 *
 * @param       txState - file scope variable
 *
 * @return      none
 */
void basicRfReceiveOn(void) {
	chMtxLock(&rf_mutex);
	txState.receiveOn = TRUE;
	halRfReceiveOn();
	chMtxUnlock(&rf_mutex);
}


/***********************************************************************************
 * @fn          basicRfReceiveOff
 *
 * @brief       Turns off receiver on radio
 *
 * @param       txState - file scope variable
 *
 * @return      none
 */
void basicRfReceiveOff(void) {
	chMtxLock(&rf_mutex);
	txState.receiveOn = FALSE;
	halRfReceiveOff();
	chMtxUnlock(&rf_mutex);
}



uint8 basicRfGetExceptionRegister(uint8 index) {
	chMtxLock(&rf_mutex);
	uint8_t b = CC2520_REGRD8(CC2520_EXCFLAG0 + index);
	chMtxUnlock(&rf_mutex);
	return b;
}

void basicRfClearException(uint8 exception) {
	chMtxLock(&rf_mutex);
	CC2520_CLEAR_EXC(exception);
	chMtxUnlock(&rf_mutex);
}

void basicRfFlushRx(void) {
	chMtxLock(&rf_mutex);
	halRfFlushRx();
	chMtxUnlock(&rf_mutex);
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

