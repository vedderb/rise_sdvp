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

package rcontrolstationcomm;

import java.util.ArrayList;
import java.util.List;
import org.bridj.Pointer;
import java.io.Serializable;

import static java.lang.System.out;

public class Utils {
	public static class RpPoint implements Serializable {
		private static final long serialVersionUID = 1L;
		public RpPoint() {
			x = 0.0;
			y = 0.0;
			speed = 0.0;
			time = 0;
		}
		
		public RpPoint(double nX, double nY) {
			x = nX;
			y = nY;
			speed = 0.0;
			time = 0;
		}
		
		public RpPoint(RpPoint p) {
			x = p.px();
			y = p.py();
			speed = p.speed();
			time = p.time();
		}
		
		public RpPoint(ROUTE_POINT p) {
			x = p.px();
			y = p.py();
			speed = p.speed();
			time = p.time();
		}
		
		void setTo(RpPoint other) {
			x = other.x;
			y = other.y;
			speed = other.speed;
			time = other.time;
		}
		
		double px() {
			return x;
		}
		
		double py() {
			return y;
		}
		
		void px(double xNew) {
			x = xNew;
		}
		
		void py(double yNew) {
			y = yNew;
		}
		
		double speed() {
			return speed;
		}
		
		void speed(double newSpeed) {
			speed = newSpeed;
		}
		
		int time() {
			return time;
		}
		
		void time(int newTime) {
			time = newTime;
		}
		
		ROUTE_POINT asRoutePoint() {
			ROUTE_POINT ret = new ROUTE_POINT();
			ret.px(x);
			ret.py(y);
			ret.speed(speed);
			ret.time(time);
			return ret;
		}
		
		public double x;
		public double y;
		public double speed;
		public int time;
	}
	
	public static List<RpPoint> getRoute(int car, int mapRoute, int timeoutMs) {
		List<RpPoint> ret = new ArrayList<RpPoint>();
		
		Pointer<ROUTE_POINT> route = Pointer.allocateArray(ROUTE_POINT.class,  500);
		Pointer<Integer> len = Pointer.pointerToInts(0);
		RControlStationCommLibrary.rcsc_getRoutePoints(car, route, 
				len, 500, mapRoute, timeoutMs);
		
		int lenRes = len.get(0);
		for (int i = 0;i < lenRes;i++) {
			// Creating a new object here is necessary for some reason, otherwise
			// bridj crashes in some situation where a route is used.
			RpPoint a = new RpPoint();
			a.px(route.get(i).px());
			a.py(route.get(i).py());
			a.speed(route.get(i).speed());
			a.time(route.get(i).time());
			ret.add(a);
		}
		
		return ret;
	}
	
	public static boolean addRoute(int car, List<RpPoint> route, boolean replace,
			boolean mapOnly, int mapRoute, int timeoutMs) {
		Pointer<ROUTE_POINT> routePtr = Pointer.allocateArray(ROUTE_POINT.class, route.size());
				
		for (int i = 0;i < route.size();i++) {
			routePtr.set(i, route.get(i).asRoutePoint());
		}
		
		return RControlStationCommLibrary.rcsc_addRoutePoints(car, routePtr, route.size(), 
				replace, mapOnly, mapRoute, timeoutMs);
	}
	
	public static CAR_STATE getCarState(int car, int timeoutMs) {
		CAR_STATE st = new CAR_STATE();
		RControlStationCommLibrary.rcsc_getState(car, Pointer.pointerTo(st), timeoutMs);
		return st;
	}
	
	public static String terminalCmd(int car, String cmd, int timeoutMs) {
		String res = new String();
		
		Pointer<Byte> cmdPtr = Pointer.allocateBytes(cmd.length() + 1);
		for (int i = 0;i < cmd.length();i++) {
			cmdPtr.set(i, (byte)cmd.charAt(i));
		}
		cmdPtr.set(cmd.length(), (byte)0);
		
		Pointer<Byte> reply = Pointer.allocateBytes(500);
		reply.set(0, (byte)0);
		RControlStationCommLibrary.rcsc_sendTerminalCmd(car, cmdPtr, reply, timeoutMs);
		
		for (int i = 0;i < 500;i++) {
			if (reply.get(i) == 0) {
				break;
			}
			
			res += (char)reply.get(i).byteValue();
		}
		
		return res;
	}
	
	/**
	 * Add a fault
	 * 
	 * @param car
	 * The ID of the car
	 * 
	 * @param probe
	 * The probe to inject on
	 * 
	 * @param type
	 * The fault type. Can be
	 * NONE
	 * BITFLIP
	 * OFFSET
	 * AMPLIFICATION
	 * SET_TO
	 * 
	 * @param param
	 * The fault parameter, e.g. amplification factor, the bit to flip.
	 * 
	 * @param start
	 * The iteration where the fault should be triggered. Iterations are
	 * counted up when the fault injection function is called, usually
	 * before the variable to inject the fault on is used.
	 * 
	 * @param duration
	 * The duration of the fault in iterations.
	 * 
	 * @timeoutMs
	 * Timeout for acknowledgement in milliseconds.
	 * 
	 * @return
	 * true for success, false otherwise.
	 */
	public static boolean fiAddFault(int car, String probe, String type,
			double param, int start, int duration, int timeoutMs) {
		terminalCmd(car,
				"fi_add_fault " +
						probe + " " +
						type + " " +
						param + " " +
						start + " " +
						duration,
				timeoutMs);
		
		return true;
	}
	
	public static boolean fiSetEnabled(int car, boolean enabled, int timeoutMs) {
		String en = "0";
		if (enabled) {
			en = "1";
		}
		terminalCmd(car, "fi_set_enabled " + en, timeoutMs);
		return true;
	}
	
	public static boolean fiClearFaults(int car, int timeoutMs) {
		terminalCmd(car, "fi_clear_faults", timeoutMs);
		return true;
	}
	
	/**
	 * Reset fault iteration counter.
	 * 
	 * @param car
	 * Car ID.
	 * 
	 * @param timeoutMs
	 * Timeout in milliseconds.
	 * 
	 * @return
	 * true for success, false otherwise
	 */
	public static boolean fiResetCnt(int car, int timeoutMs) {
		terminalCmd(car, "fi_reset_cnt", timeoutMs);
		return true;
	}
	
	public static boolean resetUwbPos(int car, int timeoutMs) {
		terminalCmd(car, "pos_uwb_reset_pos", timeoutMs);
		return true;
	}
	
	public static boolean waitUntilRouteAlmostEnded(int car, int points_left) {
		boolean res = true;
		double maxUwbDiff = 0.0;
		
		CAR_STATE st = getCarState(car, 1000);
		double uwbDiff = Math.sqrt(Math.pow((st.px() - st.px_uwb()), 2) +
				Math.pow((st.py() - st.py_uwb()), 2));
		if (uwbDiff > maxUwbDiff) {
			maxUwbDiff = uwbDiff;
		}
		
		while (st.ap_route_left() > points_left) {
			try {
				Thread.sleep(50);
				st = getCarState(car, 1000);
				uwbDiff = Math.sqrt(Math.pow((st.px() - st.px_uwb()), 2) +
						Math.pow((st.py() - st.py_uwb()), 2));
				if (uwbDiff > maxUwbDiff) {
					maxUwbDiff = uwbDiff;
				}
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				break;
			}
		}
		
		out.println("Max UWB diff: " + maxUwbDiff + " m");
		
		if (maxUwbDiff > 1.0) {
			res = false;
			out.println("[Error] Too large difference between the UWB-based"
					+ " and RTKGNSS-based positions.");
		}
		
		return res;
	}
	
	public static void followRecoveryRoute(int car, int recoveryRoute) {
		// TODO: Make sure that the route does not cross the edge when
		// the edges are defined by a convex polygon, or when there are
		// cutout areas.
		
		List<RpPoint> rec = getRoute(car, recoveryRoute, 1000);
		CAR_STATE st = getCarState(car, 1000);
		RpPoint first = new RpPoint();
		first.px(st.px());
		first.py(st.py());
		first.speed(rec.get(0).speed());
		rec.add(0, first);
				
		RControlStationCommLibrary.rcsc_setAutopilotActive(car, false, 1000);
		addRoute(car, rec, false, false, -2, 1000);
		RControlStationCommLibrary.rcsc_setAutopilotActive(car, true, 1000);
		waitPolling(car, 500);
		waitUntilRouteAlmostEnded(car, 3);
		
		RControlStationCommLibrary.rcsc_setAutopilotActive(car, false, 1000);
	}
	
	public static boolean followRecoveryRouteV2(int car, int recoveryRoute, RouteInfo ri,
			int carRoute, int aheadMargin, int genAttempts) {
		return followRecoveryRouteV2(car, recoveryRoute, ri, carRoute, aheadMargin, genAttempts, false);
	}
	
	public static boolean followRecoveryRouteV2(int car, int recoveryRoute, RouteInfo ri,
			int carRoute, int aheadMargin, int genAttempts, boolean genOnly) {
		
		// Attempt to generate valid route that connects with recovery route. May connect anywhere on
		// the recovery route as long as there are at least 3 points left.
		
		// Settings
		boolean debugPrint = false;
		int maxParts = 50;
		int genPoints = 4;
		
		boolean res = false;
		List<RpPoint> rec = getRoute(car, recoveryRoute, 1000);
		CAR_STATE st = getCarState(car, 1000);
		
		double ang = -st.yaw() * Math.PI / 180.0;
		double speed = rec.get(0).speed();
		int validRoutes = 0;
		int discardedRoutes = 0;
		
		RpPoint r0 = new RpPoint();
		r0.px(st.px());
		r0.py(st.py());
		r0.speed(speed);
		
		RpPoint r1 = new RpPoint();
		r1.px(st.px() + 0.01 * Math.cos(ang));
		r1.py(st.py() + 0.01 * Math.sin(ang));
		r1.speed(speed);
		
		List<RpPoint> recStart = new ArrayList<RpPoint>();
		List<RpPoint> recStartNow = new ArrayList<RpPoint>();
		List<RpPoint> recStartNew = new ArrayList<RpPoint>();
		List<RpPoint> recStartBest = new ArrayList<RpPoint>();
		
		recStart.add(r0);
		recStart.add(r1);
		
		RControlStationCommLibrary.rcsc_setAutopilotActive(car, false, 1000);
		
		if (ri.isRouteOk(recStart)) {
			int recIndexNow = 0;
			int recIndexBest = 0;
			double recLenLeftNow = 0.0;
			double recLenLeftBest = 0.0;
			
			for (int i = 0;i < genAttempts;i++) {
				recStartNow.clear();
				recStartNow.addAll(recStart);
				boolean ok = false;
				boolean discarded = false;

				for (int j = 0;j < maxParts;j++) {
					for (int k = 0;k < rec.size() - 3;k++) {
						for (int add = 0;add < genPoints;add++) {
							List<RpPoint> test = new ArrayList<RpPoint>();
							test.addAll(recStartNow);
							
							for (int rem = 0;rem < (genPoints - add - 1);rem++) {
								if (test.size() > 2) {
									test.remove(test.size() - 1);
								}
							}
							
							test.add(rec.get(k));
							test.add(rec.get(k + 1));

							if (ri.isRouteOk(test.subList(test.size() - 4, test.size()))) {
								recStartNew.clear();
								recStartNew.addAll(test.subList(0, test.size() - 2));
								
								ok = true;
								recIndexNow = k;
								recLenLeftNow = routeLen(rec.subList(k, rec.size()));
								break;
							}
						}
					}

					if (ok) {
						validRoutes++;
						break;
					}
					
					double minLenLeft = routeLen(rec.subList(rec.size() - 3, rec.size()));
					if (!recStartBest.isEmpty() &&
							(routeLen(recStartNow) + minLenLeft) > (routeLen(recStartBest) + recLenLeftBest)) {
						// Already longer than the best attempt so far, discard...
						discardedRoutes++;
						discarded = true;
						break;
					}
					
					boolean debug  = ri.debugEnabled();
					ri.setDebug(false);
					int sizeOld = recStartNow.size();
					recStartNow = ri.generateRouteWithin(genPoints, recStartNow, speed, aheadMargin);
					ri.setDebug(debug);
					if (sizeOld == recStartNow.size()) {
						// No new points added, most likely stuck. Start over...
						break;
					}
				}
				
				if (ok) {
					if (recStartBest.isEmpty()) {
						recStartBest.addAll(recStartNew);
						recIndexBest = recIndexNow;
						recLenLeftBest = recLenLeftNow;
					} else {
						if ((routeLen(recStartNew) + recLenLeftNow) < (routeLen(recStartBest) + recLenLeftBest)) {
							recStartBest.clear();
							recStartBest.addAll(recStartNew);
							recIndexBest = recIndexNow;
							recLenLeftBest = recLenLeftNow;
						}
					}
				}
				
				if (debugPrint) {
					String okTxt = "" + ok;
					if (discarded) {
						okTxt = "discarded";
					}
					out.println("Attempt " + i + "/" + genAttempts + " OK: " + okTxt);
				}
			}
			
			if (!recStartBest.isEmpty()) {
				for (int i = 0;i < recIndexBest;i++) {
					rec.remove(0);
				}
				
				recStartBest.add(rec.get(0));
				rec.remove(0);
				addRoute(car, recStartBest, false, false, carRoute, 1000);
				
				res = true;
			} else {
				out.println("[ERROR] Unable to generate valid recovery route start");
			}
		} else {
			out.println("[ERROR] Car seems to be outside of valid polygon");
		}

		if (res) {
			out.println("Found " + validRoutes + " valid recovery routes (" + discardedRoutes +
					" discarded). Using the shortest one (" + 
					(routeLen(recStartBest) + routeLen(rec)) + " m).");
			if (!genOnly) {
				addRoute(car, rec, false, false, -2, 1000);
				RControlStationCommLibrary.rcsc_setAutopilotActive(car, true, 2000);
				waitPolling(car, 500);
				waitUntilRouteAlmostEnded(car, 3);
				RControlStationCommLibrary.rcsc_setAutopilotActive(car, false, 2000);
			}
		}
		
		return res;
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
	
	public static double routeLen(List<RpPoint> r) {
		double res = 0.0;
		
		for (int i = 1;i < r.size();i++) {
			double x0 = r.get(i - 1).px();
			double y0 = r.get(i - 1).py();
			double x1 = r.get(i).px();
			double y1 = r.get(i).py();
			res += Math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
		}
		
		return res;
	}
}
