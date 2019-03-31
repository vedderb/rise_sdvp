#include <stdio.h>
#include <unistd.h>
#include <rcontrolstationcomm_wrapper.h>

int main(void) {
	int res = rcsc_connectTcp("localhost", 65191);
	
	if (res) {
		printf("Connected!\r\n");
	} else {
		printf("Could not connect\r\n");
	}
	
	CAR_STATE state;
	bool ok = rcsc_getState(0, &state, 5000);
	printf("OK: %d\r\n", ok);
	printf("PX: %.2f, PY: %.2f\r\n", state.px, state.py);
	
	double llh[3];
	ok = rcsc_getEnuRef(0, false, llh, 200);
	printf("OK: %d\r\n", ok);
	printf("Lat: %.7f, Lon: %.7f, Height: %.2f\r\n", llh[0], llh[1], llh[2]);
	
	llh[0] = 57.71495867;
	llh[1] = 12.89134921;
	llh[2] = 219.0;
	
	ok = rcsc_setEnuRef(0, llh, 5000);
	printf("OK: %d\r\n", ok);
	
	while (rcsc_hasError()) {
		printf("RCSC Error: %s\r\n", rcsc_lastError());
	}
	
	ROUTE_POINT route[3];
	route[0].px = 1.2;
	route[0].py = 1.56;
	route[0].speed = 2;
	route[0].time = 171;
	
	route[1].px = 2.2;
	route[1].py = 1.56;
	route[1].speed = 3;
	route[1].time = 191;
	
	route[2].px = 2.2;
	route[2].py = 4.56;
	route[2].speed = 1;
	route[2].time = 241;
	
	ok = rcsc_addRoutePoints(0, route, 3, true, false, 0, 5000);
	printf("OK: %d\r\n", ok);
	
	while (rcsc_hasError()) {
		printf("RCSC Error: %s\r\n", rcsc_lastError());
	}
	
	route[0].px = -1.2;
	route[0].py = -1.56;
	route[1].px = -2.2;
	route[1].py = -1.56;
	route[2].px = -2.2;
	route[2].py = -4.56;
	
	ok = rcsc_addRoutePoints(0, route, 3, true, true, 2, 5000);
	printf("OK: %d\r\n", ok);
	
	while (rcsc_hasError()) {
		printf("RCSC Error: %s\r\n", rcsc_lastError());
	}
	
	char reply[400];
	ok = rcsc_sendTerminalCmd(0, "ping", reply, 1000);
	if (ok) {
		printf("Sent ping. Reply: %s", reply);
	}
	
	ROUTE_POINT route_rx[20];
	int len;
	ok = rcsc_getRoutePoints(0, route_rx, &len, 20, 0, 5000);
	printf("OK: %d, len: %d\r\n", ok, len);
	if (ok) {
		for (int i = 0;i < len;i++) {
			printf("Point [%.2f, %.2f] (%.2f km/h, %d ms)\r\n",
				route_rx[i].px, route_rx[i].py, route_rx[i].speed * 3.6, route_rx[i].time);
		}
	}
	
	return 0;
}

