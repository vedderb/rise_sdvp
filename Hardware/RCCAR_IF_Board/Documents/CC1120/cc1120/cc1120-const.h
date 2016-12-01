/*
 * Copyright (c) 2012, Thingsquare, http://www.thingsquare.com/.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CC1120_CONST_H
#define CC1120_CONST_H

/* Configuration registers ("register space") */
#define CC11xx_IOCFG3  0x00
#define CC11xx_IOCFG2  0x01
#define CC11xx_IOCFG1  0x02
#define CC11xx_IOCFG0  0x03
#define CC11xx_SYNC3   0x04
#define CC11xx_SYNC2   0x05
#define CC11xx_SYNC1   0x06
#define CC11xx_SYNC0   0x07
#define CC11xx_SYNC_CFG1   0x08
#define CC11xx_SYNC_CFG0   0x09
#define CC11xx_DEVIATION_M   0x0A
#define CC11xx_MODCFG_DEV_E  0x0B
#define CC11xx_DCFILT_CFG  0x0C
#define CC11xx_PREAMBLE_CFG1   0x0D
#define CC11xx_PREAMBLE_CFG0   0x0E
#define CC11xx_FREQ_IF_CFG   0x0F
#define CC11xx_IQIC  0x10
#define CC11xx_CHAN_BW   0x11
#define CC11xx_MDMCFG1   0x12
#define CC11xx_MDMCFG0   0x13
#define CC11xx_DRATE2  0x14
#define CC11xx_DRATE1  0x15
#define CC11xx_DRATE0  0x16
#define CC11xx_AGC_REF   0x17
#define CC11xx_AGC_CS_THR  0x18
#define CC11xx_AGC_GAIN_ADJUST   0x19
#define CC11xx_AGC_CFG3  0x1A
#define CC11xx_AGC_CFG2  0x1B
#define CC11xx_AGC_CFG1  0x1C
#define CC11xx_AGC_CFG0  0x1D
#define CC11xx_FIFO_CFG  0x1E
#define CC11xx_DEV_ADDR  0x1F
#define CC11xx_SETTLING_CFG  0x20
#define CC11xx_FS_CFG  0x21
#define CC11xx_WOR_CFG1  0x22
#define CC11xx_WOR_CFG0  0x23
#define CC11xx_WOR_EVENT0_MSB  0x24
#define CC11xx_WOR_EVENT0_LSB  0x25
#define CC11xx_PKT_CFG2  0x26
#define CC11xx_PKT_CFG1  0x27
#define CC11xx_PKT_CFG0  0x28
#define CC11xx_RFEND_CFG1  0x29
#define CC11xx_RFEND_CFG0  0x2A
#define CC11xx_PA_CFG2   0x2B
#define CC11xx_PA_CFG1   0x2C
#define CC11xx_PA_CFG0   0x2D
#define CC11xx_PKT_LEN   0x2E
#define CC11xx_PKTLEN  CC11xx_PKT_LEN /* XXX backwards compatibility, remove me? */
#define CC11xx_EXTENDED_MEMORY_ACCESS  0x2F

#define CC11xx_FIRST_REGISTER_ADDRESS 0x00
#define CC11xx_LAST_REGISTER_ADDRESS 0x2E
#define CC11xx_NR_REGISTERS (CC11xx_LAST_REGISTER_ADDRESS - CC11xx_FIRST_REGISTER_ADDRESS + 1)


/* Strobe commands: same for CC1101 and CC1120 ("command strobes") */
#define CC11xx_SRES         0x30        // Reset chip.
#define CC11xx_SFSTXON      0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
// If in RX/TX: Go to a wait state where only the synthesizer is
// running (for quick RX / TX turnaround).

#define CC11xx_SXOFF        0x32        // Turn off crystal oscillator.
#define CC11xx_SCAL         0x33        // Calibrate frequency synthesizer and turn it off
// (enables quick start).

#define CC11xx_SRX          0x34        // Enable RX. Perform calibration first if coming from IDLE and
// MCSM0.FS_AUTOCAL=1.

#define CC11xx_STX          0x35        // In IDLE state: Enable TX. Perform calibration first if
// MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
// Only go to TX if channel is clear.

#define CC11xx_SIDLE        0x36        // Exit RX / TX, turn off frequency synthesizer and exit
// Wake-On-Radio mode if applicable.

#define CC11xx_SAFC         0x37        // Perform AFC adjustment of the frequency synthesizer
#define CC11xx_SWOR         0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CC11xx_SPWD         0x39        // Enter power down mode when CSn goes high.
#define CC11xx_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CC11xx_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CC11xx_SWORRST      0x3C        // Reset real time clock.
#define CC11xx_SNOP         0x3D        // No operation. May be used to pad strobe commands to two
// bytes for simpler software.


/* Status registers */
/* Extended register space, accessed via CC11xx_EXTENDED_MEMORY_ACCESS */
/* NOTE: Below addresses have been increased by 0x2F00, to allow detection of
 * extended address space registers. */

#define CC11xx_IF_MIX_CFG  0x2F00
#define CC11xx_FREQOFF_CFG   0x2F01
#define CC11xx_TOC_CFG   0x2F02
#define CC11xx_MARC_SPARE  0x2F03
#define CC11xx_ECG_CFG   0x2F04
#define CC11xx_SOFT_TX_DATA_CFG  0x2F05
#define CC11xx_EXT_CTRL  0x2F06
#define CC11xx_RCCAL_FINE  0x2F07
#define CC11xx_RCCAL_COARSE  0x2F08
#define CC11xx_RCCAL_OFFSET  0x2F09
#define CC11xx_FREQOFF1  0x2F0A
#define CC11xx_FREQOFF0  0x2F0B
#define CC11xx_FREQ2   0x2F0C
#define CC11xx_FREQ1   0x2F0D
#define CC11xx_FREQ0   0x2F0E
#define CC11xx_IF_ADC2   0x2F0F
#define CC11xx_IF_ADC1   0x2F10
#define CC11xx_IF_ADC0   0x2F11
#define CC11xx_FS_DIG1   0x2F12
#define CC11xx_FS_DIG0   0x2F13
#define CC11xx_FS_CAL3   0x2F14
#define CC11xx_FS_CAL2   0x2F15
#define CC11xx_FS_CAL1   0x2F16
#define CC11xx_FS_CAL0   0x2F17
#define CC11xx_FS_CHP  0x2F18
#define CC11xx_FS_DIVTWO   0x2F19
#define CC11xx_FS_DSM1   0x2F1A
#define CC11xx_FS_DSM0   0x2F1B
#define CC11xx_FS_DVC1   0x2F1C
#define CC11xx_FS_DVC0   0x2F1D
#define CC11xx_FS_LBI  0x2F1E
#define CC11xx_FS_PFD  0x2F1F
#define CC11xx_FS_PRE  0x2F20
#define CC11xx_FS_REG_DIV_CML  0x2F21
#define CC11xx_FS_SPARE  0x2F22
#define CC11xx_FS_VCO4   0x2F23

#define CC11xx_FS_VCO3   0x2F24
#define CC11xx_FS_VCO2   0x2F25
#define CC11xx_FS_VCO1   0x2F26
#define CC11xx_FS_VCO0   0x2F27
#define CC11xx_GBIAS6  0x2F28
#define CC11xx_GBIAS5  0x2F29
#define CC11xx_GBIAS4  0x2F2A
#define CC11xx_GBIAS3  0x2F2B
#define CC11xx_GBIAS2  0x2F2C
#define CC11xx_GBIAS1  0x2F2D
#define CC11xx_GBIAS0  0x2F2E
#define CC11xx_IFAMP   0x2F2F
#define CC11xx_LNA   0x2F30
#define CC11xx_RXMIX   0x2F31
#define CC11xx_XOSC5   0x2F32
#define CC11xx_XOSC4   0x2F33
#define CC11xx_XOSC3   0x2F34
#define CC11xx_XOSC2   0x2F35
#define CC11xx_XOSC1   0x2F36
#define CC11xx_XOSC0   0x2F37
#define CC11xx_ANALOG_SPARE  0x2F38
#define CC11xx_PA_CFG3   0x2F39
#define CC11xx_WOR_TIME1   0x2F64
#define CC11xx_WOR_TIME0   0x2F65
#define CC11xx_WOR_CAPTURE1  0x2F66
#define CC11xx_WOR_CAPTURE0  0x2F67
#define CC11xx_BIST  0x2F68
#define CC11xx_DCFILTOFFSET_I1   0x2F69
#define CC11xx_DCFILTOFFSET_I0   0x2F6A
#define CC11xx_DCFILTOFFSET_Q1   0x2F6B
#define CC11xx_DCFILTOFFSET_Q0   0x2F6C
#define CC11xx_IQIE_I1   0x2F6D
#define CC11xx_IQIE_I0   0x2F6E

#define CC11xx_IQIE_Q1   0x2F6F
#define CC11xx_IQIE_Q0   0x2F70
#define CC11xx_RSSI1   0x2F71
#define CC11xx_RSSI0   0x2F72
#define CC11xx_MARCSTATE   0x2F73
#define CC11xx_LQI_VAL   0x2F74
#define CC11xx_PQT_SYNC_ERR  0x2F75
#define CC11xx_DEM_STATUS  0x2F76
#define CC11xx_FREQOFF_EST1  0x2F77
#define CC11xx_FREQOFF_EST0  0x2F78
#define CC11xx_AGC_GAIN3   0x2F79
#define CC11xx_AGC_GAIN2   0x2F7A
#define CC11xx_AGC_GAIN1   0x2F7B
#define CC11xx_AGC_GAIN0   0x2F7C
#define CC11xx_SOFT_RX_DATA_OUT  0x2F7D
#define CC11xx_SOFT_TX_DATA_IN   0x2F7E
#define CC11xx_ASK_SOFT_RX_DATA  0x2F7F
#define CC11xx_RNDGEN  0x2F80
#define CC11xx_MAGN2   0x2F81
#define CC11xx_MAGN1   0x2F82
#define CC11xx_MAGN0   0x2F83
#define CC11xx_ANG1  0x2F84
#define CC11xx_ANG0  0x2F85
#define CC11xx_CHFILT_I2   0x2F86
#define CC11xx_CHFILT_I1   0x2F87
#define CC11xx_CHFILT_I0   0x2F88
#define CC11xx_CHFILT_Q2   0x2F89
#define CC11xx_CHFILT_Q1   0x2F8A
#define CC11xx_CHFILT_Q0   0x2F8B
#define CC11xx_GPIO_STATUS   0x2F8C

#define CC11xx_FSCAL_CTRL  0x2F8D
#define CC11xx_PHASE_ADJUST  0x2F8E
#define CC11xx_PARTNUMBER  0x2F8F
#define CC11xx_PARTVERSION   0x2F90
#define CC11xx_SERIAL_STATUS   0x2F91
#define CC11xx_RX_STATUS   0x2F92
#define CC11xx_TX_STATUS   0x2F93
#define CC11xx_MARC_STATUS1  0x2F94
#define CC11xx_MARC_STATUS0  0x2F95
#define CC11xx_PA_IFAMP_TEST   0x2F96
#define CC11xx_FSRF_TEST   0x2F97
#define CC11xx_PRE_TEST  0x2F98
#define CC11xx_PRE_OVR   0x2F99
#define CC11xx_ADC_TEST  0x2F9A
#define CC11xx_DVC_TEST  0x2F9B
#define CC11xx_ATEST   0x2F9C
#define CC11xx_ATEST_LVDS  0x2F9D
#define CC11xx_ATEST_MODE  0x2F9E
#define CC11xx_XOSC_TEST1  0x2F9F
#define CC11xx_XOSC_TEST0  0x2FA0
#define CC11xx_RXFIRST 0x2FD2
#define CC11xx_TXFIRST 0x2FD3
#define CC11xx_RXLAST 0x2FD4
#define CC11xx_TXLAST 0x2FD5
#define CC11xx_NUM_TXBYTES 0x2FD6
#define CC11xx_NUM_RXBYTES 0x2FD7
#define CC11xx_FIFO_NUM_TXBYTES 0x2FD8
#define CC11xx_FIFO_NUM_RXBYTES 0x2FD9

#define CC11xx_TXFIFO 0x3F
#define CC11xx_RXFIFO 0x3F

#define CC11xx_STATE_SLEEP             0
#define CC11xx_STATE_IDLE              1
#define CC11xx_STATE_XOFF              2
#define CC11xx_STATE_VCOON_MC          3
#define CC11xx_STATE_REGON_MC          4
#define CC11xx_STATE_MANCAL            5
#define CC11xx_STATE_VCOON             6
#define CC11xx_STATE_REGON             7
#define CC11xx_STATE_STARTCAL          8
#define CC11xx_STATE_BWBOOST           9
#define CC11xx_STATE_FS_LOCK          10
#define CC11xx_STATE_IFADCON          11
#define CC11xx_STATE_ENDCAL           12
#define CC11xx_STATE_RX               13
#define CC11xx_STATE_RX_END           14
#define CC11xx_STATE_RX_RST           15
#define CC11xx_STATE_TXRX_SWITCH      16
#define CC11xx_STATE_RXFIFO_OVERFLOW  17
#define CC11xx_STATE_FSTXON           18
#define CC11xx_STATE_TX               19
#define CC11xx_STATE_TX_END           20
#define CC11xx_STATE_RXTX_SWITCH      21
#define CC11xx_STATE_TXFIFO_UNDERFLOW 22

/* Driver compatibility */
#define CC11xx_PARTNUM CC11xx_PARTNUMBER
#define CC11xx_VERSION CC11xx_PARTVERSION
#define CC11xx_TXBYTES CC11xx_NUM_TXBYTES
#define CC11xx_RXBYTES CC11xx_NUM_RXBYTES
#define CC11xx_RSSI CC11xx_RSSI1
#define CC11xx_LQI CC11xx_LQI_VAL

/* ----- some GPIO configurations ----- */
/* Bit fields */
/* invert assert/de-assert */
#define IOCFG_GPIO_CFG_INVERT             BV(6)
/* set pin as "Analog transfer" (==pin not used as GPIO) */
#define IOCFG_GPIO_CFG_ATRAN              BV(7)

/* GPIO functionality */
/* Assert when THR is reached or EOP; de-assert on empty RxFIFO */
#define IOCFG_GPIO_CFG_RXFIFO_THR_PKT     1   
/* assert on SYNC recv/sent, de-assert on EOP */
#define IOCFG_GPIO_CFG_PKT_SYNC_RXTX      6  
/* CS valid ? assert : de-assert */
#define IOCFG_GPIO_CFG_CS_VALID           16  
/* Carrier Sense ? assert : de-assert */
#define IOCFG_GPIO_CFG_CS                 17  
/* assert when in TX, de-assert in Rx/IDLE/settling */ 
#define IOCFG_GPIO_CFG_RXIDLE_OR_TX       26  
/* assert if in rx or tx, de-assert if idle/settling (MARC_2PIN_STATUS[0]) */ 
#define IOCFG_GPIO_CFG_RXTX_OR_IDLE       38
/* high impedance */
#define IOCFG_GPIO_CFG_HIGHZ              48  



#endif /* CC1120_CONST_H */
