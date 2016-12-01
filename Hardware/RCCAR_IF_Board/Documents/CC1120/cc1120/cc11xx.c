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

/**
 * \file
 *         TI CC1120 driver.
 * \author
 *         Adam Dunkels <adam@thingsquare.com>
 *         Fredrik Osterlind <fredrik@thingsquare.com>
 */

#include "contiki.h"

/* Radio type sanity check */
#if !CC11xx_CC1101 && !CC11xx_CC1120
#error Unsupported radio type, define either of CC11xx_CC1101 and CC11xx_CC1120
#endif /* !CC11xx_CC1101 && !CC11xx_CC1120 */
#if CC11xx_CC1101 && CC11xx_CC1120
#error Radio type configuration error: both CC11xx_CC1101 or CC11xx_CC1120 are defined
#endif /* CC11xx_CC1101 && CC11xx_CC1120 */

#include "cc11xx.h"
#include "cc11xx-arch.h"

#if CC11xx_CC1101
#include "cc1101-const.h"
#include "cc1101-config.h"
#endif /* CC11xx_CC1101 */
#if CC11xx_CC1120
#include "cc1120-const.h"
#include "cc1120-config.h"
#endif /* CC11xx_CC1120 */

#include "dev/leds.h"

#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"
#include "net/mac/frame802154.h"

#include <string.h>
#include <stdio.h>

#define DEBUG 0

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
    do {                                                                  \
      rtimer_clock_t t0;                                                  \
      t0 = RTIMER_NOW();                                                  \
      while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time))) {/*printf(".");*/} \
    } while(0)

#define RSSI_OFFSET 74
#define AUX_LEN 2
#define ACK_LEN 3

#define CC11xx_MAX_PAYLOAD 125 /* Note that FIFO also contains length and footer bytes */

/* Flag indicating whether non-interrupt routines are using SPI */
static volatile uint8_t spi_locked = 0;
static volatile uint8_t interrupt_pending = 0;
#define LOCK_SPI() do { spi_locked++; } while(0)
#define SPI_IS_LOCKED() (spi_locked != 0)
#define RELEASE_SPI() do { spi_locked--; if(!SPI_IS_LOCKED() && interrupt_pending) { cc11xx_rx_interrupt(); } } while(0)

/* CRC errors counter */
uint16_t cc11xx_crc_errors = 0;

/* Packet buffer for reception */
static uint8_t packet_tx[CC11xx_MAX_PAYLOAD];
static uint8_t packet_rx[1 + CC11xx_MAX_PAYLOAD + AUX_LEN];
static volatile uint16_t packet_rx_len = 0;

static volatile uint8_t is_transmitting;

static int request_set_channel = -1;

/* If set, all packets are received, even those not intended for us */
static char promiscuous_mode = 0;

#define PERFORM_MANUAL_CALIBRATION 1 /* Manual calibration at init() */
#if PERFORM_MANUAL_CALIBRATION
static void calibrate_manual(void);
#endif /* PERFORM_MANUAL_CALIBRATION */

#if CC11xx_CC1101
// TODO XXX Let's put the configuration in the _init() function like in the cc1120!
// (or even better, in the _arch_init()...)
static const unsigned char cc1101_register_config[CC11xx_NR_REGISTERS] = {
    CC11xx_SETTING_IOCFG2,
    CC11xx_SETTING_IOCFG1,
    CC11xx_SETTING_IOCFG0,
    CC11xx_SETTING_FIFOTHR,
    0xD3,
    0x91,
    CC11xx_SETTING_PKTLEN,
    CC11xx_SETTING_PKTCTRL1,
    CC11xx_SETTING_PKTCTRL0,
    CC11xx_SETTING_ADDR,
    CC11xx_SETTING_CHANNR,
    CC11xx_SETTING_FSCTRL1,
    CC11xx_SETTING_FSCTRL0,
    CC11xx_SETTING_FREQ2,
    CC11xx_SETTING_FREQ1,
    CC11xx_SETTING_FREQ0,
    CC11xx_SETTING_MDMCFG4,
    CC11xx_SETTING_MDMCFG3,
    CC11xx_SETTING_MDMCFG2,
    CC11xx_SETTING_MDMCFG1,
    CC11xx_SETTING_MDMCFG0,
    CC11xx_SETTING_DEVIATN,
    CC11xx_SETTING_MCSM2,
    CC11xx_SETTING_MCSM1,
    CC11xx_SETTING_MCSM0,
    CC11xx_SETTING_FOCCFG,
    CC11xx_SETTING_BSCFG,
    CC11xx_SETTING_AGCCTRL2,
    CC11xx_SETTING_AGCCTRL1,
    CC11xx_SETTING_AGCCTRL0,
    CC11xx_SETTING_WOREVT1,
    CC11xx_SETTING_WOREVT0,
    CC11xx_SETTING_WORCTRL,
    CC11xx_SETTING_FREND1,
    CC11xx_SETTING_FREND0,
    CC11xx_SETTING_FSCAL3,
    CC11xx_SETTING_FSCAL2,
    CC11xx_SETTING_FSCAL1,
    CC11xx_SETTING_FSCAL0,
    CC11xx_SETTING_RCCTRL1,
    CC11xx_SETTING_RCCTRL0,
    CC11xx_SETTING_FSTEST,
    CC11xx_SETTING_PTEST,
    CC11xx_SETTING_AGCTEST,
    CC11xx_SETTING_TEST2,
    CC11xx_SETTING_TEST1,
    CC11xx_SETTING_TEST0
};
#endif /* CC11xx_CC1101 */

/* Prototypes: */
static void reset(void);
static uint8_t state(void);
static unsigned char strobe(uint8_t strobe);
static unsigned char single_read(uint16_t addr);
static uint8_t single_write(uint16_t addr, uint8_t value);
static void burst_read(uint16_t addr, uint8_t *buffer, uint8_t count);
static void burst_write(uint16_t addr, uint8_t *buffer, uint8_t count);
#if CC11xx_CC1101
static void pa_table_write(uint8_t pa_value);
#endif /* CC11xx_CC1101 */

static void pollhandler(void);

static int on(void);
static int off(void);

static int init(void);

static void restart_input(void);

static int read_packet(void *buf, unsigned short bufsize);
static int send_packet(const void *data, unsigned short len);
static int prepare( const void *data, unsigned short len );
static int transmit(unsigned short len);

static int receiving_packet(void);
static int pending_packet(void);
static int channel_clear(void);
static signed char rssi_dbm(unsigned char temp) ;
static signed char read_rssi(void);
static unsigned char read_lqi(void);
static unsigned char channel_get(void);
static void channel_set(unsigned char channel_number);

void cc11xx_set_promiscuous(char p);

#define MIN(a,b) ((a)<(b)?(a):(b))

#define READ_BIT 0x80
#define WRITE_BIT 0x00
#define BURST_BIT 0x40
#define IS_EXTENDED(x) (x & 0x2F00)

#if CC11xx_CC1101
#define RXFIFO_OVERFLOW 0x80
#define TXFIFO_UNDERFLOW 0x80
#endif /* CC11xx_CC1101 */

/*---------------------------------------------------------------------------*/

PROCESS(cc11xx_process, "CC11xx driver");

const struct radio_driver cc11xx_driver = {
    init,
    prepare,
    transmit,
    send_packet,
    read_packet,
    channel_clear,
    receiving_packet,
    pending_packet,
    on,
    off,
};

/*---------------------------------------------------------------------------*/
static uint8_t
strobe(uint8_t strobe)
{
  uint8_t ret;
  LOCK_SPI();
  CC11xx_ARCH_SPI_ENABLE();
  ret = CC11xx_ARCH_SPI_RW_BYTE(strobe);
  CC11xx_ARCH_SPI_DISABLE();
  RELEASE_SPI();
  return ret;
}
/*---------------------------------------------------------------------------*/
static uint8_t
single_read(uint16_t addr)
{
  uint8_t val;

  LOCK_SPI();
  CC11xx_ARCH_SPI_ENABLE();

  if(IS_EXTENDED(addr)) {
    addr &= ~0x2F00;
    CC11xx_ARCH_SPI_RW_BYTE(CC11xx_EXTENDED_MEMORY_ACCESS | READ_BIT);
    CC11xx_ARCH_SPI_RW_BYTE(addr);
  } else {
    CC11xx_ARCH_SPI_RW_BYTE(addr | READ_BIT);
  }

  val = CC11xx_ARCH_SPI_RW_BYTE(0);

  CC11xx_ARCH_SPI_DISABLE();
  RELEASE_SPI();

  return val;
}
/*---------------------------------------------------------------------------*/
static uint8_t
single_write(uint16_t addr, uint8_t val)
{
  uint8_t ret;
  LOCK_SPI();
  CC11xx_ARCH_SPI_ENABLE();

  if(IS_EXTENDED(addr)) {
    addr &= ~0x2F00;
    CC11xx_ARCH_SPI_RW_BYTE(CC11xx_EXTENDED_MEMORY_ACCESS | WRITE_BIT);
    CC11xx_ARCH_SPI_RW_BYTE(addr);
  } else {
    CC11xx_ARCH_SPI_RW_BYTE(addr | WRITE_BIT);
  }

  ret = CC11xx_ARCH_SPI_RW_BYTE(val);

  CC11xx_ARCH_SPI_DISABLE();
  RELEASE_SPI();
  return ret;
}
/*---------------------------------------------------------------------------*/
#if 0
static void
fifo_direct_read(uint16_t addr, uint8_t *buffer, uint8_t count)
{
  LOCK_SPI();
  CC11xx_ARCH_SPI_ENABLE();

  CC11xx_ARCH_SPI_RW_BYTE(0x3E | READ_BIT | BURST_BIT);
  CC11xx_ARCH_SPI_RW_BYTE(addr);

  CC11xx_ARCH_SPI_RW(buffer, NULL, count);

  CC11xx_ARCH_SPI_DISABLE();
  RELEASE_SPI();
}
/*---------------------------------------------------------------------------*/
static void
debug_print_rxfifo(void)
{
  int i;
  uint8_t first, last, buf[128];

  /* Print RXFIFO (using direct ram access) */
  first = single_read(CC11xx_RXFIRST);
  last = single_read(CC11xx_RXLAST);

  fifo_direct_read(0x80, buf, 128);
  printf("FIFO[0x%02x]: ", 0x80);
  for(i = 0; i < 128; i++) {
    if(i == first && i == last) {
      printf("*");
    } else if(i == first) {
      printf(">");
    } else if(i == last) {
      printf("<");
    } else {
      printf(" ");
    }
    printf("%02x", buf[i]);
  }
  printf("\n");
}
#endif
/*---------------------------------------------------------------------------*/
static void
burst_read(uint16_t addr, uint8_t *buffer, uint8_t count)
{
  LOCK_SPI();
  CC11xx_ARCH_SPI_ENABLE();

  if(IS_EXTENDED(addr)) {
    addr &= ~0x2F00;
    CC11xx_ARCH_SPI_RW_BYTE(CC11xx_EXTENDED_MEMORY_ACCESS | READ_BIT | BURST_BIT);
    CC11xx_ARCH_SPI_RW_BYTE(addr);
  } else {
    CC11xx_ARCH_SPI_RW_BYTE(addr | READ_BIT | BURST_BIT);
  }

  CC11xx_ARCH_SPI_RW(buffer, NULL, count);

  CC11xx_ARCH_SPI_DISABLE();
  RELEASE_SPI();
}
/*---------------------------------------------------------------------------*/
static void
burst_write(uint16_t addr, uint8_t *buffer, uint8_t count)
{
  LOCK_SPI();
  CC11xx_ARCH_SPI_ENABLE();

  if(IS_EXTENDED(addr)) {
    addr &= ~0x2F00;
    CC11xx_ARCH_SPI_RW_BYTE(CC11xx_EXTENDED_MEMORY_ACCESS | WRITE_BIT | BURST_BIT);
    CC11xx_ARCH_SPI_RW_BYTE(addr);
  } else {
    CC11xx_ARCH_SPI_RW_BYTE(addr | WRITE_BIT | BURST_BIT);
  }

  CC11xx_ARCH_SPI_RW(NULL, buffer, count);

  CC11xx_ARCH_SPI_DISABLE();
  RELEASE_SPI();
}
/*---------------------------------------------------------------------------*/
static uint8_t
txbytes(void)
{
  uint8_t txbytes1, txbytes2;

  do {
    burst_read(CC11xx_TXBYTES, &txbytes1, 1);
    burst_read(CC11xx_TXBYTES, &txbytes2, 1);
    if(txbytes1 - 1 == txbytes2 || txbytes1 - 2 == txbytes2) {
      /* XXX Workaround for slow CPU/SPI */
      return txbytes2;
    }
  } while(txbytes1 != txbytes2);

  return txbytes1;
}
/*---------------------------------------------------------------------------*/
static uint8_t
read_rxbytes(void)
{
  uint8_t rxbytes1, rxbytes2;

  do {
    burst_read(CC11xx_RXBYTES, &rxbytes1, 1);
    burst_read(CC11xx_RXBYTES, &rxbytes2, 1);
    if(rxbytes1 + 1 == rxbytes2 || rxbytes1 + 2 == rxbytes2) {
      /* XXX Workaround for slow CPU/SPI */
      return rxbytes2;
    }
  } while(rxbytes1 != rxbytes2);

  return rxbytes1;
}
/*---------------------------------------------------------------------------*/
static void
write_txfifo(uint8_t *data, uint8_t len)
{
  uint8_t status;
  int i;
#define CC11xx_STATUS_STATE_MASK             0x70
#define CC11xx_STATUS_STATE_TXFIFO_UNDERFLOW 0x70

  LOCK_SPI();
  burst_write(CC11xx_TXFIFO, &len, 1);

#define FIRST_TX 8

  i = MIN(len, FIRST_TX);
  burst_write(CC11xx_TXFIFO, data, i);
  strobe(CC11xx_STX);

  if(len > i) {
    CC11xx_ARCH_SPI_ENABLE();
    CC11xx_ARCH_SPI_RW_BYTE(CC11xx_TXFIFO | 0x40);
    for(; i < len; i++) {

      status = CC11xx_ARCH_SPI_RW_BYTE(data[i]);

      if((status & CC11xx_STATUS_STATE_MASK) ==
          CC11xx_STATUS_STATE_TXFIFO_UNDERFLOW) {
        CC11xx_ARCH_SPI_DISABLE();
        /* TX FIFO underflow, acknowledge it with an SFTX (else the
         radio becomes completely unresponsive) followed by an SRX,
         and break the transmission. */
        strobe(CC11xx_SFTX);
        strobe(CC11xx_SRX);
        break;
      } else if((status & 0x0f) < 2) {
        CC11xx_ARCH_SPI_DISABLE();
        BUSYWAIT_UNTIL(txbytes() < 60 || (txbytes() & 0x80) != 0,
                       RTIMER_SECOND / 10);
        if(txbytes() & 0x80) {
          /* TX FIFO underflow. */
          strobe(CC11xx_SFTX);
          strobe(CC11xx_SRX);
          break;
        }
        CC11xx_ARCH_SPI_ENABLE();
        CC11xx_ARCH_SPI_RW_BYTE(CC11xx_TXFIFO | 0x40);
      }
    }
    CC11xx_ARCH_SPI_DISABLE();
  }
  RELEASE_SPI();
}
/*---------------------------------------------------------------------------*/
#if CC11xx_CC1101
static void
pa_table_write(uint8_t val)
{
  uint8_t table[8];
  int i;

  for(i = 0; i < sizeof(table); i++) {
    table[i] = val;
  }

  burst_write(CC11xx_PATABLE, table, 8);
}
#endif /* CC11xx_CC1101 */
/*---------------------------------------------------------------------------*/
static void
check_txfifo(void)
{
#if CC11xx_CC1101
  uint8_t b;
  b = txbytes();
  if((b & TXFIFO_UNDERFLOW) != 0) {
    /* Acknowledge TX FIFO underflow. */
    strobe(CC11xx_SFTX);
    strobe(CC11xx_SRX);
    return;
  }
#endif /* CC11xx_CC1101 */

  if(state() == CC11xx_STATUS_STATE_TXFIFO_UNDERFLOW) {
    /* Acknowledge TX FIFO underflow. */
    strobe(CC11xx_SFTX);
    strobe(CC11xx_SRX);
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc11xx_process, ev, data)
{

  PROCESS_POLLHANDLER(pollhandler());

  PROCESS_BEGIN();

#if 0
  while(1) {
    static struct etimer et;
    uint8_t rxbytes, txbytes;
    etimer_set(&et, CLOCK_SECOND * 4);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    //    cc11xx_rx_interrupt();
    burst_read(CC11xx_RXBYTES, &rxbytes, 1);
    burst_read(CC11xx_TXBYTES, &txbytes, 1);
    printf("state 0x%02x rxbytes 0x%02x txbytes 0x%02x\n",
           state(), rxbytes, txbytes);
    on();
  }
#endif /* 0 */

  PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_EXIT);

  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*
 * process rx packet, when process receives poll
 */
static void
pollhandler(void)
{
  int len;

  /* If we were requested to change radio channels but couldn't since the radio
   * was actively sending or receiving data, we try again now. */
  if(request_set_channel >= 0) {
    cc11xx_channel_set(request_set_channel);
    request_set_channel = -1;
  }

  do {
    //    printf("p(");
    packetbuf_clear();
    len = read_packet(packetbuf_dataptr(), PACKETBUF_SIZE);

    if(len > 0) {
      packetbuf_set_datalen(len);
      /*      printf("RDC input %d\n", len);*/
      NETSTACK_RDC.input();
    }

    /* If we received a packet (or parts thereof) while processing the
       previous packet, we immediately pull it out from the RX
       FIFO. */
    //    printf("[");
    cc11xx_rx_interrupt();
    //    printf("]");
    //    printf(")\n");
  } while(packet_rx_len > 0);
}
/*---------------------------------------------------------------------------*/
static void
flushrx(void)
{
  restart_input();

  LOCK_SPI();
  if(state() == CC11xx_STATE_RXFIFO_OVERFLOW) {
    strobe(CC11xx_SFRX);
  }
  strobe(CC11xx_SIDLE);
  BUSYWAIT_UNTIL((state() == CC11xx_STATE_IDLE), RTIMER_SECOND / 10);
  strobe(CC11xx_SFRX);
  strobe(CC11xx_SRX);
  RELEASE_SPI();
}
/*---------------------------------------------------------------------------*/
static void
send_ack(uint8_t seqno)
{
  uint8_t len;
  uint8_t ackdata[ACK_LEN] = {0, 0, 0};

  if(is_transmitting) {
    /* Trying to send an ACK while transmitting - should not be
       possible, so this check is here only to make sure. */
    return;
  }
  ackdata[0] = FRAME802154_ACKFRAME;
  ackdata[1] = 0;
  ackdata[2] = seqno;
  len = ACK_LEN;

  /* Send packet length */

#if CC11xx_CC1120
  strobe(CC11xx_SIDLE);
#endif /* CC11xx_CC1120 */
  is_transmitting = 1;
  write_txfifo((unsigned char *)ackdata, len);
  check_txfifo();
  is_transmitting = 0;
#if DEBUG
  printf("^");
#endif /* DEBUG */
}
/*---------------------------------------------------------------------------*/
#define PACKET_LIFETIME (CLOCK_SECOND / 2)
static struct {
  struct pt pt;
  uint8_t receiving;
  int len;
  volatile int ptr;
  uint8_t buf[CC11xx_MAX_PAYLOAD + AUX_LEN];
  struct timer timer;
} rxstate;
/*---------------------------------------------------------------------------*/
static void
restart_input(void)
{
  PT_INIT(&rxstate.pt);
  rxstate.receiving = 0;
}
/*---------------------------------------------------------------------------*/
static int
is_receiving(void)
{
  if(timer_expired(&rxstate.timer)) {
    restart_input();
  }
  return rxstate.receiving;
}
/*---------------------------------------------------------------------------*/
static int
is_broadcast_addr(uint8_t mode, uint8_t *addr)
{
  int i = mode == FRAME802154_SHORTADDRMODE ? 2 : 8;
  while(i-- > 0) {
    if(addr[i] != 0xff) {
      return 0;
    }
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
input_byte(uint8_t byte)
{
  int crc;

  if(timer_expired(&rxstate.timer)) {
    restart_input();
  }
  PT_BEGIN(&rxstate.pt);

  /* The first incoming byte is the length of the packet. */

  rxstate.receiving = 1;
  rxstate.len = byte;

  if(rxstate.len + AUX_LEN > sizeof(rxstate.buf) || rxstate.len == 0) {
    printf("Bad len %d, state %d rxbytes %d\n", rxstate.len, state(),
           read_rxbytes());
    flushrx();
    PT_EXIT(&rxstate.pt);
  }

  timer_set(&rxstate.timer, PACKET_LIFETIME);
  for(rxstate.ptr = 0;
      rxstate.ptr < rxstate.len;
      rxstate.ptr++) {

    /* Wait for the next data byte to be received. */
    PT_YIELD(&rxstate.pt);
    if(rxstate.ptr < sizeof(rxstate.buf)) {
      rxstate.buf[rxstate.ptr] = byte;
    }
  }

  /* Receive two more bytes from the FIFO: the RSSI value and the LQI/CRC */
  PT_YIELD(&rxstate.pt);
  rxstate.buf[rxstate.ptr] = byte;
  rxstate.ptr++;
  PT_YIELD(&rxstate.pt);
  rxstate.buf[rxstate.ptr] = byte;
  crc = (byte & 0x80);
  rxstate.ptr++;

  if(crc == 0) {
#if DEBUG
    printf("bad crc\n");
#endif /* DEBUG */

    cc11xx_crc_errors++;
    flushrx();
  } else if(packet_rx_len > 0) {
#if DEBUG
    printf("Packet in the buffer (%d), dropping %d bytes\n", packet_rx_len, rxstate.len);
#endif /* DEBUG */
    flushrx();
    process_poll(&cc11xx_process);
  } else if(rxstate.len < ACK_LEN) {
    /* Drop packets that are way too small: less than ACK_LEN (3) bytes
       long. */
#if DEBUG
    printf("!");
#endif /* DEBUG */
  } else {
    /* Read out the first three bytes to determine if we should send an
       ACK or not. */

    /* Send a link-layer ACK before reading the full packet. */
    if(rxstate.len >= ACK_LEN) {
      /* Try to parse the incoming frame as a 802.15.4 header. */
      frame802154_t info154;
      if(frame802154_parse(rxstate.buf, rxstate.len, &info154) != 0) {
        /* XXX Potential optimization here: we could check if the
    frame is destined for us, or for the broadcast address and
    discard the packet if it isn't for us. */
        if(promiscuous_mode || info154.fcf.frame_type == FRAME802154_ACKFRAME
            || is_broadcast_addr(FRAME802154_SHORTADDRMODE,
                                 (uint8_t *)&info154.dest_addr)
            || is_broadcast_addr(FRAME802154_LONGADDRMODE,
                                 (uint8_t *)&info154.dest_addr)
            || rimeaddr_cmp((rimeaddr_t *)&info154.dest_addr,
                            &rimeaddr_node_addr)) {

          /* For dataframes that has the ACK request bit set and that
       is destined for us, we send an ack. */
          if(info154.fcf.frame_type == FRAME802154_DATAFRAME &&
              info154.fcf.ack_required != 0 &&
              rimeaddr_cmp((rimeaddr_t *)&info154.dest_addr,
                           &rimeaddr_node_addr)) {
            send_ack(info154.seq);

            /* Make sure that we don't put the radio in the IDLE state
         before the ACK has been fully transmitted. */
            BUSYWAIT_UNTIL((state() != CC11xx_STATE_TX), RTIMER_SECOND / 10);
            ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
            ENERGEST_ON(ENERGEST_TYPE_LISTEN);
            if(state() == CC11xx_STATE_TX) {
#if DEBUG
              printf("didn't end ack tx (in %d, txbytes %d)\n", state(), txbytes());
#endif /* DEBUG */
              check_txfifo();
              flushrx();
            }
          }
          memcpy((void *)packet_rx, rxstate.buf,
                 rxstate.len + AUX_LEN);
          packet_rx_len = rxstate.len + AUX_LEN; /* including AUX */

          process_poll(&cc11xx_process);
#if DEBUG 
          printf("#");
#endif /* DEBUG */
        }
      }
    }
  }
  rxstate.receiving = 0;

  PT_END(&rxstate.pt);
}
/*---------------------------------------------------------------------------*/
/**
 * The CC11xx interrupt handler: called by the hardware interrupt
 * handler, which is defined as part of the cc11xx-arch interface.
 */
int
cc11xx_rx_interrupt(void)
{
  /* NB: This function may be called both from an rx interrupt and
   from cc11xx_process */
  uint8_t rxbytes, s;

  if(SPI_IS_LOCKED()) {
#if DEBUG
    printf("/%d", spi_locked);
#endif /* DEBUG */
    process_poll(&cc11xx_process);
    interrupt_pending = 1;
    return 1;
  }
  interrupt_pending = 0;

  s = state();
  if(s == CC11xx_STATE_RXFIFO_OVERFLOW) {
    burst_read(CC11xx_RXBYTES, &rxbytes, 1);
#if DEBUG
    printf("irqflush\n");
    printf("rxbytes 0x%02x\n", rxbytes);
#endif /* DEBUG */
    flushrx();
    return 1;
  }
  if(s == CC11xx_STATE_TXFIFO_UNDERFLOW) {
#if DEBUG
    printf("irqflushtx\n");
#endif /* DEBUG */
    strobe(CC11xx_SFTX);
    strobe(CC11xx_SRX);
    return 1;
  }

  if(is_receiving() && timer_expired(&rxstate.timer)) {
#if DEBUG
    printf("Packet expired, flushing fifo\n");
#endif /* DEBUG */
    flushrx();
    return 1;
  }

  /* Read each byte from the RXFIFO and put it into the input_byte()
   function, which takes care of the reception logic. */
  do {
    uint8_t byte;
    int i, numbytes=0;

    rxbytes = read_rxbytes();

#if CC11xx_CC1101
    if(rxbytes & RXFIFO_OVERFLOW) {
#if DEBUG
      printf("ovf\n");
#endif /* DEBUG */
      flushrx();
      process_poll(&cc11xx_process);
      leds_off(LEDS_GREEN);
      return 1;
    }
#elif CC11xx_CC1120 /* CC11xx_CC1101 */
    if(state() == CC11xx_STATE_RXFIFO_OVERFLOW) {
#if DEBUG
      printf("ovf\n");
#endif /* DEBUG */
      flushrx();
      process_poll(&cc11xx_process);
      leds_off(LEDS_GREEN);
      return 1;
    }
#endif /* CC11xx_CC1101 */

    if(rxbytes == 0) {
      return 1;
    }
    if(rxbytes > 1 + CC11xx_MAX_PAYLOAD + AUX_LEN) {
#if DEBUG
      printf("rxbytes too large %d\n", rxbytes);
#endif /* DEBUG */
      flushrx();
      return 1;
    }

    /* Check if we are receiving a packet. If not, we feed the first
     byte of data, which hold the length of the packet, to the input
     function. This will help us later decide how much to read from
     the rx fifo. */
    if(!is_receiving()) {
      burst_read(CC11xx_RXFIFO, &byte, 1);
      input_byte(byte);
      rxbytes = read_rxbytes();
    }

    if(is_receiving()) {
      if(rxbytes < (rxstate.len + AUX_LEN) - rxstate.ptr) {
        /* If the fifo contains only a part of the packet, we leave
         one byte behind. */
        numbytes = rxbytes - 1;
      } else {
        /* If the full packet can be found in the fifo, we read it out
         in full. */
        numbytes = rxbytes;
      }
    }

    {
      static uint8_t tmpbuf[CC11xx_MAX_PAYLOAD + AUX_LEN + 1];
      if(numbytes > 0 && numbytes <= sizeof(tmpbuf)) {
        burst_read(CC11xx_RXFIFO, tmpbuf, numbytes);

        for(i = 0; i < numbytes; i++) {
          byte = tmpbuf[i];
          input_byte(byte);
        }
      } else {
        flushrx();
        return 1;
      }
    }

    rxbytes = read_rxbytes();
  } while(rxbytes > 1);

#if DEBUG
  printf("-");
#endif /* DEBUG */

  return 1;
}
/*---------------------------------------------------------------------------*/
/**
 * Read received packet in RXFIFO.
 */
static int
read_packet(void *buf, unsigned short bufsize)
{
  int len;

  len = 0;

  if(packet_rx_len > 0) {
    signed char rssi;
    signed char lqi;
    signed char crc;
    rssi = rssi_dbm(packet_rx[packet_rx_len - 2]);
    lqi = (packet_rx[packet_rx_len - 1] & 0x7F);
    crc = (packet_rx[packet_rx_len - 1] & 0x80);
    if(crc == 0) {
      packet_rx_len = 0;
#if DEBUG
      printf("Bad crc (0x%02x)\n", lqi | crc);
#endif /* DEBUG */
      cc11xx_crc_errors++;
      flushrx();
      return 0;
    }

    len = packet_rx_len - AUX_LEN;
    memcpy(buf, (void *)packet_rx, MIN(len, bufsize));

    packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rssi);
    packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, lqi);

    RIMESTATS_ADD(llrx);

    packet_rx_len = 0;
  }
  return MIN(len, bufsize);
}
/*---------------------------------------------------------------------------*/
/**
 * transmit packet - send packet in buffer
 */
static int
transmit(unsigned short len)
{
  if(state() == CC11xx_STATE_RXFIFO_OVERFLOW) {
    flushrx();
  }

#if DEBUG
  printf("tx len %d\n", len);
#endif /* DEBUG */
  if(len > CC11xx_MAX_PAYLOAD) {
#if DEBUG || 1
    printf("cc11xx: too big tx %d\n", len);
#endif /* DEBUG */
    return RADIO_TX_ERR;
  }

  RIMESTATS_ADD(lltx);

  LOCK_SPI();
#if DEBUG 
  printf(".");
#endif /* DEBUG */
  strobe(CC11xx_SIDLE);

  is_transmitting = 1;
  write_txfifo((unsigned char *)packet_tx, len);

  BUSYWAIT_UNTIL((state() == CC11xx_STATE_TX), RTIMER_SECOND / 10);
  ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);

  if(state() != CC11xx_STATE_TX) {
#if DEBUG
    printf("didn't tx (in %d)\n", state());
#endif /* DEBUG */
    check_txfifo();
    flushrx();
    RELEASE_SPI();
    is_transmitting = 0;
    return RADIO_TX_ERR;
  }
  RELEASE_SPI();
  BUSYWAIT_UNTIL((state() != CC11xx_STATE_TX), RTIMER_SECOND / 10);
  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  if(state() == CC11xx_STATE_TX) {
#if DEBUG
    printf("didn't end tx (in %d, txbytes %d)\n", state(), txbytes());
#endif /* DEBUG */
    check_txfifo();
    flushrx();
    is_transmitting = 0;
    return RADIO_TX_ERR;
  }
  check_txfifo();
  is_transmitting = 0;
  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
/**
 * Prepare function - copy packet into buffer
 */
static int
prepare(const void *payload, unsigned short len)
{
  if(state() == CC11xx_STATE_RXFIFO_OVERFLOW) {
    flushrx();
  }

  if(len > CC11xx_MAX_PAYLOAD) {
#if DEBUG
    printf("CC11xx DEBUG: Too big packet, aborting prepare %d\n", len);
#endif /* DEBUG */
    return RADIO_TX_ERR;
  }

  memcpy(packet_tx, payload, len);
  return RADIO_TX_OK;
}
/*---------------------------------------------------------------------------*/
/**
 * send function - copy packet into buffer and send.
 */
static int
send_packet(const void *data, unsigned short len)
{
  int ret;
  if(len > CC11xx_MAX_PAYLOAD) {
#if DEBUG
    printf("CC11xx DEBUG: Too big packet, aborting send %d\n", len);
#endif /* DEBUG */
    return RADIO_TX_ERR;
  }
  prepare(data, len);
  ret = transmit(len);

  return ret;
}
/*---------------------------------------------------------------------------*/
static int
on(void)
{
  if(SPI_IS_LOCKED()) {
    return 0;
  }
  LOCK_SPI();
  flushrx();
  strobe(CC11xx_SRX);
  RELEASE_SPI();

  ENERGEST_ON(ENERGEST_TYPE_LISTEN);

  leds_on(LEDS_RED);
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
off(void)
{
  if(SPI_IS_LOCKED()) {
    return 0;
  }

  LOCK_SPI();
  strobe(CC11xx_SIDLE);
  BUSYWAIT_UNTIL((state() == CC11xx_STATE_IDLE), RTIMER_SECOND / 10);
  strobe(CC11xx_SPWD);
  RELEASE_SPI();

  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  leds_off(LEDS_RED);

  return 1;
}
/*---------------------------------------------------------------------------*/
/**
 * reset radio
 */
static void
reset(void)
{
  LOCK_SPI();
  strobe(CC11xx_SRES);
  strobe(CC11xx_SNOP); /* XXX needed? */

#if CC11xx_CC1101
  burst_write(CC11xx_IOCFG2, (unsigned char *)cc1101_register_config,
              CONF_REG_SIZE);
  pa_table_write(CC11xx_PA_11);
#elif CC11xx_CC1120
  /* TODO Move to *-arch.c or smartstudio.c? */
  single_write(CC11xx_IOCFG3, CC11xx_SETTING_IOCFG3);
  single_write(CC11xx_IOCFG2, CC11xx_SETTING_IOCFG2);
  single_write(CC11xx_IOCFG1, CC11xx_SETTING_IOCFG1);
  single_write(CC11xx_IOCFG0, CC11xx_SETTING_IOCFG0);
  single_write(CC11xx_SYNC_CFG1, CC11xx_SETTING_SYNC_CFG1);
  single_write(CC11xx_DEVIATION_M, CC11xx_SETTING_DEVIATION_M);
  single_write(CC11xx_MODCFG_DEV_E, CC11xx_SETTING_MODCFG_DEV_E);
  single_write(CC11xx_DCFILT_CFG, CC11xx_SETTING_DCFILT_CFG);
  single_write(CC11xx_PREAMBLE_CFG1, CC11xx_SETTING_PREAMBLE_CFG1);
  single_write(CC11xx_FREQ_IF_CFG, CC11xx_SETTING_FREQ_IF_CFG);
  single_write(CC11xx_IQIC, CC11xx_SETTING_IQIC);
  single_write(CC11xx_CHAN_BW, CC11xx_SETTING_CHAN_BW);
  single_write(CC11xx_MDMCFG0, CC11xx_SETTING_MDMCFG0);
  single_write(CC11xx_DRATE2, CC11xx_SETTING_DRATE2);
  single_write(CC11xx_DRATE1, CC11xx_SETTING_DRATE1);
  single_write(CC11xx_DRATE0, CC11xx_SETTING_DRATE0);
  single_write(CC11xx_AGC_REF, CC11xx_SETTING_AGC_REF);
  single_write(CC11xx_AGC_CS_THR, CC11xx_SETTING_AGC_CS_THR);
  single_write(CC11xx_AGC_CFG3, CC11xx_SETTING_AGC_CFG3);
  single_write(CC11xx_AGC_CFG2, CC11xx_SETTING_AGC_CFG2);
  single_write(CC11xx_AGC_CFG1, CC11xx_SETTING_AGC_CFG1);
  single_write(CC11xx_AGC_CFG0, CC11xx_SETTING_AGC_CFG0);
  single_write(CC11xx_FIFO_CFG, CC11xx_SETTING_FIFO_CFG);
  single_write(CC11xx_SETTLING_CFG, CC11xx_SETTING_SETTLING_CFG);
  single_write(CC11xx_FS_CFG, CC11xx_SETTING_FS_CFG);
  single_write(CC11xx_PKT_CFG0, CC11xx_SETTING_PKT_CFG0);
  single_write(CC11xx_RFEND_CFG1, CC11xx_SETTING_RFEND_CFG1);
  single_write(CC11xx_RFEND_CFG0, CC11xx_SETTING_RFEND_CFG0);
  single_write(CC11xx_PA_CFG2, CC11xx_SETTING_PA_CFG2);
  single_write(CC11xx_PA_CFG0, CC11xx_SETTING_PA_CFG0);
  single_write(CC11xx_PKT_LEN, CC11xx_SETTING_PKT_LEN);
  single_write(CC11xx_IF_MIX_CFG, CC11xx_SETTING_IF_MIX_CFG);
  single_write(CC11xx_TOC_CFG, CC11xx_SETTING_TOC_CFG);
  single_write(CC11xx_FREQ2, CC11xx_SETTING_FREQ2);
  single_write(CC11xx_FREQ1, CC11xx_SETTING_FREQ1);
  single_write(CC11xx_FS_DIG1, CC11xx_SETTING_FS_DIG1);
  single_write(CC11xx_FS_DIG0, CC11xx_SETTING_FS_DIG0);
  single_write(CC11xx_FS_CAL1, CC11xx_SETTING_FS_CAL1);
  single_write(CC11xx_FS_CAL0, CC11xx_SETTING_FS_CAL0);
  single_write(CC11xx_FS_DIVTWO, CC11xx_SETTING_FS_DIVTWO);
  single_write(CC11xx_FS_DSM0, CC11xx_SETTING_FS_DSM0);
  single_write(CC11xx_FS_DVC0, CC11xx_SETTING_FS_DVC0);
  single_write(CC11xx_FS_PFD, CC11xx_SETTING_FS_PFD);
  single_write(CC11xx_FS_PRE, CC11xx_SETTING_FS_PRE);
  single_write(CC11xx_FS_REG_DIV_CML, CC11xx_SETTING_FS_REG_DIV_CML);
  single_write(CC11xx_FS_SPARE, CC11xx_SETTING_FS_SPARE);
  single_write(CC11xx_FS_VCO0, CC11xx_SETTING_FS_VCO0);
  single_write(CC11xx_XOSC5, CC11xx_SETTING_XOSC5);
  single_write(CC11xx_XOSC2, CC11xx_SETTING_XOSC2);
  single_write(CC11xx_XOSC1, CC11xx_SETTING_XOSC1);
#endif /* CC11xx_CC1101 or CC11xx_CC1120? */

  RELEASE_SPI();

  on();
}
/*---------------------------------------------------------------------------*/
static uint8_t
channel_get(void)
{
#if CC11xx_CC1101
  return single_read(CC11xx_CHANNR);
#elif CC11xx_CC1120
  printf("channel_get not implemented\n");
  return -1;
#endif /* CC11xx_CC1101 or CC11xx_CC1120? */
}
/*---------------------------------------------------------------------------*/
/* XXX Placeholder awaiting new radio_driver API */
void
cc1101_channel_set(uint8_t c)
{
  channel_set(c);
}
/*---------------------------------------------------------------------------*/
/* XXX Placeholder awaiting new radio_driver API */
void
cc11xx_channel_set(uint8_t c)
{
  channel_set(c);
}
/*---------------------------------------------------------------------------*/
static void
channel_set(uint8_t c)
{
  if(SPI_IS_LOCKED() || is_transmitting || receiving_packet()) {
#if DEBUG||1
    printf("cannot change channel, radio is busy: spi:%d tx:%d rx:%d\n",
           SPI_IS_LOCKED(), is_transmitting, receiving_packet());
#endif /* DEBUG */
    request_set_channel = c;
    process_poll(&cc11xx_process);
    return;
  }
  request_set_channel = -1;

  LOCK_SPI();

#if CC11xx_CC1101
  single_write(CC11xx_CHANNR, c);
#elif CC11xx_CC1120
  /* XXX Requires FS_CFG.FSD_BANDSELECT == 0010b => LO divider 4 */
  /* XXX Requires SETTLING_CFG.FS_AUTOCAL == 01b */
  uint32_t freq, f_vco, freq_regs;

#define FHSS_ETSI_50    0
#define FHSS_FCC_50     1
#if FHSS_FCC_50 && FHSS_ETSI_50
#error Error: FHSS, both FHSS_ETSI_50 and FHSS_FCC_50 defined. Please set only one.
#endif

#if FHSS_ETSI_50
/* ETSI EN 300 220, 50 channels: Channel 0 863 Mhz, channel 49 869.125MHz */
#define CARRIER_FREQUENCY 13808 /* Channel 0: 863 Mhz (in 16th MHz) */
#define CHANNEL_SPACING 2       /* Channel spacing: 125kHz (in 16th MHz) */
#define FREQ_OSC 512            /* in 16th MHz */
#define LO_DIVIDER 4

#elif FHSS_FCC_50
/* FHSS 902 -- 928 MHz (FCC Part 15.247; 15.249) */
#define CARRIER_FREQUENCY 14432 /* Channel 0: 902 Mhz (in 16th MHz) */
#define CHANNEL_SPACING 2       /* Channel spacing: 125kHz (in 16th MHz) */
#define FREQ_OSC 512            /* in 16th MHz */
#define LO_DIVIDER 4

#else
#error Unknown FHSS frequencies, please define FHSS_ETSI_50 or FHSS_FCC_50
#endif

  freq = CARRIER_FREQUENCY + c*CHANNEL_SPACING; /* Channel -> radio frequency */
  f_vco = freq*LO_DIVIDER; /* Radio frequency -> VCO frequency */
  freq_regs = f_vco<<16; /* Multiply by 2^16 */
  freq_regs /= FREQ_OSC; /* Divide by oscillator frequency -> Frequency registers */

  /*printf("channel %d => freq_regs %02x %02x %02x\n", c,
         ((unsigned char*)&freq_regs)[0],
         ((unsigned char*)&freq_regs)[1],
         ((unsigned char*)&freq_regs)[2]);*/
  single_write(CC11xx_FREQ0, ((unsigned char*)&freq_regs)[0]);
  single_write(CC11xx_FREQ1, ((unsigned char*)&freq_regs)[1]);
  single_write(CC11xx_FREQ2, ((unsigned char*)&freq_regs)[2]);
  strobe(CC11xx_SCAL);
  strobe(CC11xx_SIDLE);
  BUSYWAIT_UNTIL((state() == CC11xx_STATE_IDLE), RTIMER_SECOND / 10);
  single_write(CC11xx_FREQ0, ((unsigned char*)&freq_regs)[0]);
  single_write(CC11xx_FREQ1, ((unsigned char*)&freq_regs)[1]);
  single_write(CC11xx_FREQ2, ((unsigned char*)&freq_regs)[2]);
  strobe(CC11xx_SRX);
#endif /* CC11xx_CC1101 or CC11xx_CC1120? */

  RELEASE_SPI();
}
/*---------------------------------------------------------------------------*/
static signed char
rssi_dbm(unsigned char raw_rssi)
{
  int16_t dbm = 0;

  if(raw_rssi >= 128) {
    dbm = (int16_t)((int16_t)(raw_rssi - 256) / 2) - RSSI_OFFSET;
  } else {
    dbm = (raw_rssi / 2) - RSSI_OFFSET;
  }
  return dbm;
}
/*---------------------------------------------------------------------------*/
#if 0
static signed char
rssi_dbm_inv(unsigned char dbm)
{
  uint8_t i;
  for(i = 0; i < 0xff; i++) {
    if((uint8_t)rssi_dbm(i) == (uint8_t)dbm) {
      return i;
    }
  }
  return -1;
}
#endif
/*---------------------------------------------------------------------------*/
/* XXX Placeholder awaiting new radio_driver API */
signed char
cc11xx_read_rssi(void)
{
  return read_rssi();
}
/*---------------------------------------------------------------------------*/
/**
 * read RSSI
 */
static signed char
read_rssi(void)
{
  uint8_t raw;

  burst_read(CC11xx_RSSI, &raw, 1);
  return rssi_dbm(raw);
}
/*---------------------------------------------------------------------------*/
/**
 * rf1a read LQI
 */
static unsigned char
read_lqi(void)
{
  unsigned char temp;

  burst_read(CC11xx_LQI, &temp, 1);
  return temp;
}
/*---------------------------------------------------------------------------*/
/**
 * cc1101 receiving packet function - check Radio SFD
 */
static int
receiving_packet(void)
{
  uint8_t rxbytes;
  if(SPI_IS_LOCKED()) {
    return 0;
  }

  burst_read(CC11xx_RXBYTES, &rxbytes, 1);
  if(rxbytes & 0x80) {
    /* RXFIFO overflow - flush it. */
    flushrx();
#if DEBUG
    printf("recv overflow - nuke\n");
#endif /* DEBUG */
  } else if(rxbytes > 0) {
    /* If we detect a reception here, we'll poll the process just in
       case somehow the interrupt missed it... */
    process_poll(&cc11xx_process);
    return 1;
  }
  return is_receiving();
}
/*---------------------------------------------------------------------------*/
static int
pending_packet(void)
{
  if(packet_rx_len > 0) {
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static uint8_t
state(void)
{
  uint8_t state;

  burst_read(CC11xx_MARCSTATE, &state, 1);
  return state & 0x1f;
}
/*---------------------------------------------------------------------------*/
void
cc11xx_set_promiscuous(char p)
{
  promiscuous_mode = p;
}
/*---------------------------------------------------------------------------*/
/**
 * Return CCA bit in packet status register.
 */
static int
channel_clear(void)
{
  uint8_t radio_was_off, cca, val;

  if(SPI_IS_LOCKED()) {
    return 1;
  }

  if(state() == CC11xx_STATE_IDLE ||
     state() == CC11xx_STATE_SLEEP) {
    radio_was_off = 1;
    on();
    BUSYWAIT_UNTIL((state() == CC11xx_STATE_RX), RTIMER_SECOND / 100);
  } else {
    radio_was_off = 0;
  }

  cca = 1; /* clear if no carrier detected */

#if CC11xx_CC1101

  //  printf("%d ", state());
  if(radio_was_off) {
    /* Why is this needed? */
    burst_read(CC11xx_PKTSTATUS, &val, 1);
    burst_read(CC11xx_PKTSTATUS, &val, 1);
  }

  if(state() == CC11xx_STATE_RX) {
    burst_read(CC11xx_PKTSTATUS, &val, 1);
    //    printf("%02x ", cca_bit);
    cca = ((val & 0x10) >> 4);
  } else {
    //    printf("state %d\n", state());
  }

#elif CC11xx_CC1120

#define CARRIER_SENSE_VALID BV(1)
#define CARRIER_SENSE BV(2)
  burst_read(CC11xx_RSSI0, &val, 1);
  if(!(val & CARRIER_SENSE_VALID)) {
    cca = 1;
  } else if (val & CARRIER_SENSE) {
    cca = 0;
  } else {
    cca = 1;
  }
#endif

  if(radio_was_off) {
    off();
  }
  return cca;
}
/*---------------------------------------------------------------------------*/
/**
 * Initialize CC11xx radio module.
 */
static int
init(void)
{
  cc11xx_arch_init();
  reset();

  process_start(&cc11xx_process, NULL);

  off();
  cc11xx_arch_interrupt_enable();

#if PERFORM_MANUAL_CALIBRATION
  strobe(CC11xx_SIDLE);
  dint(); /* XXX No eint() is performed here! */
  calibrate_manual();
#endif /* PERFORM_MANUAL_CALIBRATION */

  return 1;
}
/*---------------------------------------------------------------------------*/
void
cc11xx_print_state(void)
{
  printf("state marc: %d\n", state());
  printf("state rxbytes: %d\n", read_rxbytes());
  printf("state txbytes: %d\n", txbytes());
}
/*---------------------------------------------------------------------------*/
#if PERFORM_MANUAL_CALIBRATION
/* Below code is adapted from TI's CC112x/CC1175 ERRATA */
#define cc112xSpiWriteReg burst_write
#define cc112xSpiReadReg burst_read
#define trxSpiCmdStrobe strobe
#define CC112X_FS_VCO2 CC11xx_FS_VCO2
#define CC112X_FS_VCO4 CC11xx_FS_VCO4
#define CC112X_FS_CHP CC11xx_FS_CHP
#define CC112X_FS_CAL2 CC11xx_FS_CAL2
#define CC112X_MARCSTATE CC11xx_MARCSTATE
#define SCAL CC11xx_SCAL
/*---------------------------------------------------------------------------*/
#define VCDAC_START_OFFSET  2
#define FS_VCO2_INDEX       0
#define FS_VCO4_INDEX       1
#define FS_CHP_INDEX        2
/*---------------------------------------------------------------------------*/
static void
calibrate_manual(void)
{
  uint8_t original_fs_cal2;
  uint8_t calResults_for_vcdac_start_high[3];
  uint8_t calResults_for_vcdac_start_mid[3];
  uint8_t marcstate;
  uint8_t writeByte;

  // 1) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
  writeByte = 0x00;
  cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

  // 2) Start with high VCDAC (original VCDAC_START + 2):
  cc112xSpiReadReg(CC112X_FS_CAL2, &original_fs_cal2, 1);
  writeByte = original_fs_cal2 + VCDAC_START_OFFSET;
  cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

  // 3) Calibrate and wait for calibration to be done (radio back in IDLE state)
  trxSpiCmdStrobe(SCAL);
  do {
    cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
  } while(marcstate != 0x41);

  // 4) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with high VCDAC_START value
  cc112xSpiReadReg(CC112X_FS_VCO2,
                   &calResults_for_vcdac_start_high[FS_VCO2_INDEX], 1);
  cc112xSpiReadReg(CC112X_FS_VCO4,
                   &calResults_for_vcdac_start_high[FS_VCO4_INDEX], 1);
  cc112xSpiReadReg(CC112X_FS_CHP, &calResults_for_vcdac_start_high[FS_CHP_INDEX],
                   1);

  // 5) Set VCO cap-array to 0 (FS_VCO2 = 0x00)
  writeByte = 0x00;
  cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);

  // 6) Continue with mid VCDAC (original VCDAC_START):
  writeByte = original_fs_cal2;
  cc112xSpiWriteReg(CC112X_FS_CAL2, &writeByte, 1);

  // 7) Calibrate and wait for calibration to be done (radio back in IDLE state)
  trxSpiCmdStrobe(SCAL);
  do {
    cc112xSpiReadReg(CC112X_MARCSTATE, &marcstate, 1);
  } while(marcstate != 0x41);

  // 8) Read FS_VCO2, FS_VCO4 and FS_CHP register obtained with mid VCDAC_START value
  cc112xSpiReadReg(CC112X_FS_VCO2, &calResults_for_vcdac_start_mid[FS_VCO2_INDEX],
                   1);
  cc112xSpiReadReg(CC112X_FS_VCO4, &calResults_for_vcdac_start_mid[FS_VCO4_INDEX],
                   1);
  cc112xSpiReadReg(CC112X_FS_CHP, &calResults_for_vcdac_start_mid[FS_CHP_INDEX],
                   1);

  // 9) Write back highest FS_VCO2 and corresponding FS_VCO and FS_CHP result
  if(calResults_for_vcdac_start_high[FS_VCO2_INDEX]
                                     > calResults_for_vcdac_start_mid[FS_VCO2_INDEX]) {
    writeByte = calResults_for_vcdac_start_high[FS_VCO2_INDEX];
    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
    writeByte = calResults_for_vcdac_start_high[FS_VCO4_INDEX];
    cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
    writeByte = calResults_for_vcdac_start_high[FS_CHP_INDEX];
    cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
  } else {
    writeByte = calResults_for_vcdac_start_mid[FS_VCO2_INDEX];
    cc112xSpiWriteReg(CC112X_FS_VCO2, &writeByte, 1);
    writeByte = calResults_for_vcdac_start_mid[FS_VCO4_INDEX];
    cc112xSpiWriteReg(CC112X_FS_VCO4, &writeByte, 1);
    writeByte = calResults_for_vcdac_start_mid[FS_CHP_INDEX];
    cc112xSpiWriteReg(CC112X_FS_CHP, &writeByte, 1);
  }
}
#endif /* PERFORM_MANUAL_CALIBRATION */
