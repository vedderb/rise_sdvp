
#include "rcontrolstationcomm_wrapper.h"
#include "rcontrolstationcomm.h"

static RControlStationComm rcsc;

bool rcsc_connectTcp(const char *host, int port)
{
    return rcsc.connectTcp(host, port);
}

void rcsc_setDebugLevel(int level)
{
    rcsc.setDebugLevel(level);
}

void rcsc_disconnectTcp()
{
    rcsc.disconnectTcp();
}

bool rcsc_getState(int car, CAR_STATE *state, int timeoutMs)
{
    return rcsc.getState(car, state, timeoutMs);
}
