#ifndef RCONTROLSTATIONCOMM_WRAPPER_H
#define RCONTROLSTATIONCOMM_WRAPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "rcontrolstationcomm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bool rcsc_connectTcp(const char* host, int port);
void rcsc_disconnectTcp(void);
void rcsc_setDebugLevel(int level);
bool rcsc_hasError();
char *rcsc_lastError();
bool rcsc_getState(int car, CAR_STATE *state, int timeoutMs);
bool rcsc_getEnuRef(int car, bool fromMap, double *llh, int timeoutMs);
bool rcsc_setEnuRef(int car, double *llh, int timeoutMs);
bool rcsc_addRoutePoints(int car, ROUTE_POINT *route, int len,
                         bool replace, bool mapOnly,
                         int mapRoute, int timeoutMs);
bool rcsc_clearRoute(int car, int timeoutMs);
bool rcsc_setAutopilotActive(int car, bool active, int timeoutMs);
bool rcsc_rcControl(int car, int mode, double value, double steering);
bool rcsc_getRoutePoints(int car, ROUTE_POINT *route, int *len,
                         int maxLen, int mapRoute, int timeoutMs);

#ifdef __cplusplus
}
#endif

#endif // RCONTROLSTATIONCOMM_WRAPPER_H
