/*
    Copyright 2018 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

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
void rcsc_clearBuffers();
bool rcsc_getState(int car, CAR_STATE *state, int timeoutMs);
bool rcsc_getEnuRef(int car, bool fromMap, double *llh, int timeoutMs);
bool rcsc_setEnuRef(int car, double *llh, int timeoutMs);
bool rcsc_addRoutePoints(int car, ROUTE_POINT *route, int len,
                         bool replace, bool mapOnly,
                         int mapRoute, int timeoutMs);
bool rcsc_clearRoute(int car, int mapRoute, int timeoutMs);
bool rcsc_setAutopilotActive(int car, bool active, int timeoutMs);
bool rcsc_rcControl(int car, int mode, double value, double steering);
bool rcsc_getRoutePoints(int car, ROUTE_POINT *route, int *len,
                         int maxLen, int mapRoute, int timeoutMs);
bool rcsc_sendTerminalCmd(int car, char *cmd, char *reply, int timeoutMs);

#ifdef __cplusplus
}
#endif

#endif // RCONTROLSTATIONCOMM_WRAPPER_H
