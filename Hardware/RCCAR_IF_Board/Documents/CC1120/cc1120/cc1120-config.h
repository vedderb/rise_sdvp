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

#ifndef CC1120_CONFIG_H
#define CC1120_CONFIG_H

/* ---- START OF TI SMART STUDIO EXPORTED CONFIG ---- */
/* 200 kbps, 4-GFSK, max throughput; GPIOs are configured further down */
#define CC11xx_SETTING_SYNC3                0x93
#define CC11xx_SETTING_SYNC2                0x0B
#define CC11xx_SETTING_SYNC1                0x51
#define CC11xx_SETTING_SYNC0                0xDE
#define CC11xx_SETTING_SYNC_CFG1            0x07
#define CC11xx_SETTING_SYNC_CFG0            0x17
#define CC11xx_SETTING_DEVIATION_M          0x53
#define CC11xx_SETTING_MODCFG_DEV_E         0x2F
#define CC11xx_SETTING_DCFILT_CFG           0x04
#define CC11xx_SETTING_PREAMBLE_CFG1        0x18
#define CC11xx_SETTING_PREAMBLE_CFG0        0x2A
#define CC11xx_SETTING_FREQ_IF_CFG          0x00
#define CC11xx_SETTING_IQIC                 0x00
#define CC11xx_SETTING_CHAN_BW              0x01
#define CC11xx_SETTING_MDMCFG1              0x46
#define CC11xx_SETTING_MDMCFG0              0x05
#define CC11xx_SETTING_DRATE2               0xA9
#define CC11xx_SETTING_DRATE1               0x99
#define CC11xx_SETTING_DRATE0               0x99
#define CC11xx_SETTING_AGC_REF              0x3C
#define CC11xx_SETTING_AGC_CS_THR           0xEC
#define CC11xx_SETTING_AGC_GAIN_ADJUST      0x00
#define CC11xx_SETTING_AGC_CFG3             0x83
#define CC11xx_SETTING_AGC_CFG2             0x60
#define CC11xx_SETTING_AGC_CFG1             0xA9
#define CC11xx_SETTING_AGC_CFG0             0xC0
#define CC11xx_SETTING_FIFO_CFG             0x00
#define CC11xx_SETTING_DEV_ADDR             0x00
#define CC11xx_SETTING_SETTLING_CFG         0x03
#define CC11xx_SETTING_FS_CFG               0x12
#define CC11xx_SETTING_WOR_CFG1             0x08
#define CC11xx_SETTING_WOR_CFG0             0x21
#define CC11xx_SETTING_WOR_EVENT0_MSB       0x00
#define CC11xx_SETTING_WOR_EVENT0_LSB       0x00
#define CC11xx_SETTING_PKT_CFG2             0x04
#define CC11xx_SETTING_PKT_CFG1             0x05
#define CC11xx_SETTING_PKT_CFG0             0x20
#define CC11xx_SETTING_RFEND_CFG1           0x0F
#define CC11xx_SETTING_RFEND_CFG0           0x00
#define CC11xx_SETTING_PA_CFG2              0x7F
#define CC11xx_SETTING_PA_CFG1              0x56
#define CC11xx_SETTING_PA_CFG0              0x01
#define CC11xx_SETTING_PKT_LEN              0xFF
#define CC11xx_SETTING_IF_MIX_CFG           0x00
#define CC11xx_SETTING_FREQOFF_CFG          0x20
#define CC11xx_SETTING_TOC_CFG              0x0A
#define CC11xx_SETTING_MARC_SPARE           0x00
#define CC11xx_SETTING_ECG_CFG              0x00
#define CC11xx_SETTING_SOFT_TX_DATA_CFG     0x00
#define CC11xx_SETTING_EXT_CTRL             0x01
#define CC11xx_SETTING_RCCAL_FINE           0x00
#define CC11xx_SETTING_RCCAL_COARSE         0x00
#define CC11xx_SETTING_RCCAL_OFFSET         0x00
#define CC11xx_SETTING_FREQOFF1             0x00
#define CC11xx_SETTING_FREQOFF0             0x00
#define CC11xx_SETTING_FREQ2                0x70
#define CC11xx_SETTING_FREQ1                0xC0
#define CC11xx_SETTING_FREQ0                0x00
#define CC11xx_SETTING_IF_ADC2              0x02
#define CC11xx_SETTING_IF_ADC1              0xA6
#define CC11xx_SETTING_IF_ADC0              0x04
#define CC11xx_SETTING_FS_DIG1              0x00
#define CC11xx_SETTING_FS_DIG0              0x5F
#define CC11xx_SETTING_FS_CAL3              0x00
#define CC11xx_SETTING_FS_CAL2              0x20
#define CC11xx_SETTING_FS_CAL1              0x40
#define CC11xx_SETTING_FS_CAL0              0x0E
#define CC11xx_SETTING_FS_CHP               0x28
#define CC11xx_SETTING_FS_DIVTWO            0x03
#define CC11xx_SETTING_FS_DSM1              0x00
#define CC11xx_SETTING_FS_DSM0              0x33
#define CC11xx_SETTING_FS_DVC1              0xFF
#define CC11xx_SETTING_FS_DVC0              0x17
#define CC11xx_SETTING_FS_LBI               0x00
#define CC11xx_SETTING_FS_PFD               0x50
#define CC11xx_SETTING_FS_PRE               0x6E
#define CC11xx_SETTING_FS_REG_DIV_CML       0x14
#define CC11xx_SETTING_FS_SPARE             0xAC
#define CC11xx_SETTING_FS_VCO4              0x14
#define CC11xx_SETTING_FS_VCO3              0x00
#define CC11xx_SETTING_FS_VCO2              0x00
#define CC11xx_SETTING_FS_VCO1              0x00
#define CC11xx_SETTING_FS_VCO0              0xB4
#define CC11xx_SETTING_GBIAS6               0x00
#define CC11xx_SETTING_GBIAS5               0x02
#define CC11xx_SETTING_GBIAS4               0x00
#define CC11xx_SETTING_GBIAS3               0x00
#define CC11xx_SETTING_GBIAS2               0x10
#define CC11xx_SETTING_GBIAS1               0x00
#define CC11xx_SETTING_GBIAS0               0x00
#define CC11xx_SETTING_IFAMP                0x01
#define CC11xx_SETTING_LNA                  0x01
#define CC11xx_SETTING_RXMIX                0x01
#define CC11xx_SETTING_XOSC5                0x0E
#define CC11xx_SETTING_XOSC4                0xA0
#define CC11xx_SETTING_XOSC3                0x03
#define CC11xx_SETTING_XOSC2                0x04
#define CC11xx_SETTING_XOSC1                0x03
#define CC11xx_SETTING_XOSC0                0x00
#define CC11xx_SETTING_ANALOG_SPARE         0x00
#define CC11xx_SETTING_PA_CFG3              0x00
#define CC11xx_SETTING_IRQ0M                0x00
#define CC11xx_SETTING_IRQ0F                0x00
#define CC11xx_SETTING_WOR_TIME1            0x00
#define CC11xx_SETTING_WOR_TIME0            0x00
#define CC11xx_SETTING_WOR_CAPTURE1         0x00
#define CC11xx_SETTING_WOR_CAPTURE0         0x00
#define CC11xx_SETTING_BIST                 0x00
#define CC11xx_SETTING_DCFILTOFFSET_I1      0x00
#define CC11xx_SETTING_DCFILTOFFSET_I0      0x00
#define CC11xx_SETTING_DCFILTOFFSET_Q1      0x00
#define CC11xx_SETTING_DCFILTOFFSET_Q0      0x00
#define CC11xx_SETTING_IQIE_I1              0x00
#define CC11xx_SETTING_IQIE_I0              0x00
#define CC11xx_SETTING_IQIE_Q1              0x00
#define CC11xx_SETTING_IQIE_Q0              0x00
#define CC11xx_SETTING_RSSI1                0x80
#define CC11xx_SETTING_RSSI0                0x00
#define CC11xx_SETTING_MARCSTATE            0x41
#define CC11xx_SETTING_LQI_VAL              0x00
#define CC11xx_SETTING_PQT_SYNC_ERR         0xFF
#define CC11xx_SETTING_DEM_STATUS           0x00
#define CC11xx_SETTING_FREQOFF_EST1         0x00
#define CC11xx_SETTING_FREQOFF_EST0         0x00
#define CC11xx_SETTING_AGC_GAIN3            0x00
#define CC11xx_SETTING_AGC_GAIN2            0xD1
#define CC11xx_SETTING_AGC_GAIN1            0x00
#define CC11xx_SETTING_AGC_GAIN0            0x3F
#define CC11xx_SETTING_SOFT_RX_DATA_OUT     0x00
#define CC11xx_SETTING_SOFT_TX_DATA_IN      0x00
#define CC11xx_SETTING_ASK_SOFT_RX_DATA     0x30
#define CC11xx_SETTING_RNDGEN               0x7F
#define CC11xx_SETTING_MAGN2                0x00
#define CC11xx_SETTING_MAGN1                0x00
#define CC11xx_SETTING_MAGN0                0x00
#define CC11xx_SETTING_ANG1                 0x00
#define CC11xx_SETTING_ANG0                 0x00
#define CC11xx_SETTING_CHFILT_I2            0x08
#define CC11xx_SETTING_CHFILT_I1            0x00
#define CC11xx_SETTING_CHFILT_I0            0x00
#define CC11xx_SETTING_CHFILT_Q2            0x00
#define CC11xx_SETTING_CHFILT_Q1            0x00
#define CC11xx_SETTING_CHFILT_Q0            0x00
#define CC11xx_SETTING_GPIO_STATUS          0x00
#define CC11xx_SETTING_FSCAL_CTRL           0x01
#define CC11xx_SETTING_PHASE_ADJUST         0x00
#define CC11xx_SETTING_PARTNUMBER           0x00
#define CC11xx_SETTING_PARTVERSION          0x00
#define CC11xx_SETTING_SERIAL_STATUS        0x00
#define CC11xx_SETTING_RX_STATUS            0x01
#define CC11xx_SETTING_TX_STATUS            0x00
#define CC11xx_SETTING_MARC_STATUS1         0x00
#define CC11xx_SETTING_MARC_STATUS0         0x00
#define CC11xx_SETTING_PA_IFAMP_TEST        0x00
#define CC11xx_SETTING_FSRF_TEST            0x00
#define CC11xx_SETTING_PRE_TEST             0x00
#define CC11xx_SETTING_PRE_OVR              0x00
#define CC11xx_SETTING_ADC_TEST             0x00
#define CC11xx_SETTING_DVC_TEST             0x0B
#define CC11xx_SETTING_ATEST                0x40
#define CC11xx_SETTING_ATEST_LVDS           0x00
#define CC11xx_SETTING_ATEST_MODE           0x00
#define CC11xx_SETTING_XOSC_TEST1           0x3C
#define CC11xx_SETTING_XOSC_TEST0           0x00
#define CC11xx_SETTING_RXFIRST              0x00
#define CC11xx_SETTING_TXFIRST              0x00
#define CC11xx_SETTING_RXLAST               0x00
#define CC11xx_SETTING_TXLAST               0x00
#define CC11xx_SETTING_NUM_TXBYTES          0x00
#define CC11xx_SETTING_NUM_RXBYTES          0x00
#define CC11xx_SETTING_FIFO_NUM_TXBYTES     0x0F
#define CC11xx_SETTING_FIFO_NUM_RXBYTES     0x00

/* ---- END OF TI SMART STUDIO EXPORTED CONFIG ---- */

/* Override exported config, the cc11xx driver depends on below settings! */

/* Variable length packets */
#define PKT_CFG0_LENGTH_CONFIG_VARIABLE     BV(5)
#undef CC11xx_SETTING_PKT_CFG0
#define CC11xx_SETTING_PKT_CFG0             PKT_CFG0_LENGTH_CONFIG_VARIABLE

/* Calculate and append CRC to packets */
#define PKT_CFG1_APPEND_STATUS              BV(0)
#define PKT_CFG1_CRC_CFG                    BV(2)
#undef CC11xx_SETTING_PKT_CFG1
#define CC11xx_SETTING_PKT_CFG1             (PKT_CFG1_APPEND_STATUS|PKT_CFG1_CRC_CFG)

/* CCA triggers on RSSI and RX mode */
#define PKT_CFG2_CCA_MODE_BELOW_OR_RX       (BV(2)|BV(3))
#undef CC11xx_SETTING_PKT_CFG2
#define CC11xx_SETTING_PKT_CFG2             PKT_CFG2_CCA_MODE_BELOW_OR_RX

/* Return to RX after TX */
#define RFEND_CFG0_TXOFF_MODE_RETURN_TO_RX  (BV(4)|BV(5))
#undef CC11xx_SETTING_RFEND_CFG0
#define CC11xx_SETTING_RFEND_CFG0           RFEND_CFG0_TXOFF_MODE_RETURN_TO_RX

/* Return to RX after RX */
#define RFEND_CFG1_RXOFF_MODE_RETURN_TO_RX  (BV(4)|BV(5))
#undef CC11xx_SETTING_RFEND_CFG1
#define CC11xx_SETTING_RFEND_CFG1           (0x0F | RFEND_CFG1_RXOFF_MODE_RETURN_TO_RX)

/* FIFO threshold 3 bytes, same as an ack */
#undef CC11xx_SETTING_FIFO_CFG
#define CC11xx_SETTING_FIFO_CFG             3

/* GPIO configuration */
#define CC11xx_SETTING_IOCFG3               IOCFG_GPIO_CFG_CS
#define CC11xx_SETTING_IOCFG2               IOCFG_GPIO_CFG_RXTX_OR_IDLE
#define CC11xx_SETTING_IOCFG1               0xB0  /* GPIO1 is always used for SPI */
#define CC11xx_SETTING_IOCFG0               (IOCFG_GPIO_CFG_PKT_SYNC_RXTX | IOCFG_GPIO_CFG_INVERT)

#endif /* CC1120_CONFIG_H */
