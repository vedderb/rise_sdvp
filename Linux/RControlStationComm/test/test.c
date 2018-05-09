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
	rcsc_getState(0, &state, 1000);
	printf("PX: %.2f, PY: %.2f\r\n", state.px, state.py);
	
	return 0;
}

