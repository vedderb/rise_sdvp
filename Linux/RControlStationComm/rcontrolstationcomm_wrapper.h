#ifndef RCONTROLSTATIONCOMM_WRAPPER_H
#define RCONTROLSTATIONCOMM_WRAPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "rcontrolstationcomm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bool rcsc_connectTcp(const char* host, int port);
void rcsc_setDebugLevel(int level);
void rcsc_disconnectTcp(void);
bool rcsc_getState(int car, CAR_STATE *state, int timeoutMs);

#ifdef __cplusplus
}
#endif

#endif // RCONTROLSTATIONCOMM_WRAPPER_H
