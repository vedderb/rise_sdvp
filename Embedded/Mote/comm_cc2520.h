#ifndef COMM_CC2520_H_
#define COMM_CC2520_H_

#include "conf_general.h"

// Functions
void comm_cc2520_init(void);
void comm_cc2520_send_buffer(uint8_t *data, unsigned int len);
void comm_cc2520_send_packet(uint8_t *data, uint8_t len);

#endif /* COMM_CC2520_H_ */
