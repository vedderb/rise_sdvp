package rcontrolstationcomm;

import java.util.ArrayList;
import java.util.List;
import org.bridj.Pointer;
import static java.lang.System.out;

public class Utils {
	public static List<ROUTE_POINT> getRoute(int car, int mapRoute, int timeoutMs) {
		List<ROUTE_POINT> ret = new ArrayList<ROUTE_POINT>();
		
		Pointer<ROUTE_POINT> route = Pointer.allocateArray(ROUTE_POINT.class,  500);
		Pointer<Integer> len = Pointer.pointerToInts(0);
		RControlStationCommLibrary.rcsc_getRoutePoints(car, route, 
				len, 500, mapRoute, timeoutMs);
		
		int lenRes = len.get(0);
		for (int i = 0;i < lenRes;i++) {
			ROUTE_POINT a = new ROUTE_POINT();
			a.px(route.get(i).px());
			a.py(route.get(i).py());
			a.speed(route.get(i).speed());
			a.time(route.get(i).time());
			ret.add(a);
		}
		
		return ret;
	}
	
	public static boolean addRoute(int car, List<ROUTE_POINT> route, boolean replace,
			boolean mapOnly, int mapRoute, int timeoutMs) {
		Pointer<ROUTE_POINT> routePtr = Pointer.allocateArray(ROUTE_POINT.class, route.size());
				
		for (int i = 0;i < route.size();i++) {
			routePtr.set(i, route.get(i));
		}
		
		return RControlStationCommLibrary.rcsc_addRoutePoints(car,  routePtr,  route.size(), 
				replace, mapOnly, mapRoute, timeoutMs);
	}
	
	public static CAR_STATE getCarState(int car, int timeoutMs) {
		CAR_STATE st = new CAR_STATE();
		RControlStationCommLibrary.rcsc_getState(0, Pointer.pointerTo(st), timeoutMs);
		return st;
	}
	
	public static void waitUntilRouteAlmostEnded(int car) {
		CAR_STATE st = getCarState(car, 1000);
		
		while (st.ap_route_left() > 4) {
			try {
				Thread.sleep(50);
				st = getCarState(car, 1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				break;
			}
		}
	}
	
	public static void followRecoveryRoute(int car, int recoveryRoute) {
		List<ROUTE_POINT> rec = getRoute(car, recoveryRoute, 1000);
		CAR_STATE st = getCarState(car, 1000);
		ROUTE_POINT first = new ROUTE_POINT();
		first.px(st.px());
		first.py(st.py());
		first.speed(rec.get(0).speed());
		rec.add(0, first);
				
		RControlStationCommLibrary.rcsc_setAutopilotActive(0, false, 1000);
		addRoute(car, rec, false, false, -2, 1000);
		RControlStationCommLibrary.rcsc_setAutopilotActive(0, true, 1000);
		waitPolling(car, 500);
		waitUntilRouteAlmostEnded(car);
		waitPolling(car, 3000);
		
		RControlStationCommLibrary.rcsc_setAutopilotActive(0, false, 1000);
	}
	
	public static void waitPolling(int car, int ms) {
		int timeLeft = ms;
		
		try {
			while (timeLeft > 0) {
				int sleep = 50;
				if (sleep > timeLeft) {
					sleep = timeLeft;
				}
				
				Thread.sleep(sleep);
				getCarState(car, 1000);
				timeLeft -= sleep;
			}
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
