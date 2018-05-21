package rcontrolstationcomm;

import java.util.ArrayList;
import java.util.List;
import org.bridj.Pointer;

public class Utils {
	public static List<ROUTE_POINT> getRoute(int car, int mapRoute, int timeoutMs) {
		List<ROUTE_POINT> ret = new ArrayList<ROUTE_POINT>();
		
		Pointer<ROUTE_POINT> route = Pointer.allocateArray(ROUTE_POINT.class,  500);
		Pointer<Integer> len = Pointer.pointerToInts(0);
		RControlStationCommLibrary.rcsc_getRoutePoints(car, route, 
				len, 500, mapRoute, timeoutMs);
		
		int lenRes = len.get(0);
		for (int i = 0;i < lenRes;i++) {
			ret.add(route.get(i));
		}
		
		return ret;
	}
	
	public static boolean addRoute(int car, List<ROUTE_POINT> route, boolean replace,
			boolean mapOnly, int mapRoute, int timeoutMs) {		
		Pointer<ROUTE_POINT> routePtr = Pointer.allocateArray(ROUTE_POINT.class,  route.size());
		
		for (int i = 0;i < route.size();i++) {
			routePtr.set(i, route.get(i));
		}
		
		return RControlStationCommLibrary.rcsc_addRoutePoints(car,  routePtr,  route.size(), 
				replace, mapOnly, mapRoute, timeoutMs);
	}
	
	public static CAR_STATE getCarState(int car, int timeoutMs) {
		CAR_STATE st = new CAR_STATE();
		RControlStationCommLibrary.rcsc_getState(0, Pointer.pointerTo(st), 1000);
		return st;
	}
}
