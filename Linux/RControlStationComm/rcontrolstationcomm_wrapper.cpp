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

#include "rcontrolstationcomm_wrapper.h"
#include "rcontrolstationcomm.h"

static RControlStationComm rcsc;

bool rcsc_connectTcp(const char *host, int port)
{
    return rcsc.connectTcp(host, port);
}

void rcsc_disconnectTcp()
{
    rcsc.disconnectTcp();
}

void rcsc_setDebugLevel(int level)
{
    rcsc.setDebugLevel(level);
}

bool rcsc_hasError()
{
    return rcsc.hasError();
}

char *rcsc_lastError()
{
    return rcsc.lastError();
}

bool rcsc_getState(int car, CAR_STATE *state, int timeoutMs)
{
    return rcsc.getState(car, state, timeoutMs);
}

bool rcsc_getEnuRef(int car, bool fromMap, double *llh, int timeoutMs)
{
    return rcsc.getEnuRef(car, fromMap, llh, timeoutMs);
}

bool rcsc_setEnuRef(int car, double *llh, int timeoutMs)
{
    return rcsc.setEnuRef(car, llh, timeoutMs);
}

bool rcsc_addRoutePoints(int car, ROUTE_POINT *route, int len,
                         bool replace, bool mapOnly,
                         int mapRoute, int timeoutMs) {
    return rcsc.addRoutePoints(car, route, len, replace, mapOnly, mapRoute, timeoutMs);
}

bool rcsc_clearRoute(int car, int mapRoute, int timeoutMs)
{
    return rcsc.clearRoute(car, mapRoute, timeoutMs);
}

bool rcsc_setAutopilotActive(int car, bool active, int timeoutMs)
{
    return rcsc.setAutopilotActive(car, active, timeoutMs);
}

bool rcsc_rcControl(int car, int mode, double value, double steering)
{
    return rcsc.rcControl(car, mode, value, steering);
}

bool rcsc_getRoutePoints(int car, ROUTE_POINT *route, int *len, int maxLen, int mapRoute, int timeoutMs)
{
    return rcsc.getRoutePoints(car, route, len, maxLen, mapRoute, timeoutMs);
}

bool rcsc_sendTerminalCmd(int car, char *cmd, char *reply, int timeoutMs)
{
    return rcsc.sendTerminalCmd(car, cmd, reply, timeoutMs);
}

void rcsc_clearBuffers()
{
    rcsc.clearBuffers();
}
