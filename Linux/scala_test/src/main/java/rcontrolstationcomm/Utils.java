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
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import org.bridj.Pointer;
import org.w3c.dom.Document;
import org.w3c.dom.Element;

import java.io.Serializable;
import java.text.NumberFormat;
import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.System.out;

import java.io.File;
import java.io.PrintWriter;

public final class Utils {
	public static final class RpPoint implements Serializable {
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

		public RpPoint(double nX, double nY, double nSpeed, int nTime) {
			x = nX;
			y = nY;
			speed = nSpeed;
			time = nTime;
		}

		public RpPoint(RpPoint p) {
			x = p.x;
			y = p.y;
			speed = p.speed;
			time = p.time;
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

		private double x;
		private double y;
		private double speed;
		private int time;
	}

	public static List<RpPoint> getRoute(int car, int mapRoute, int timeoutMs) {
		List<RpPoint> ret = new ArrayList<RpPoint>();

		Pointer<ROUTE_POINT> route = Pointer.allocateArray(ROUTE_POINT.class,  500);
		Pointer<Integer> len = Pointer.pointerToInts(0);
		RControlStationCommLibrary.rcsc_getRoutePoints(car, route, 
				len, 500, mapRoute, timeoutMs);

		int lenRes = len.get(0);
		for (int i = 0;i < lenRes;i++) {
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

	public static class WaitRouteResult {
		public double maxUwbDiff;
		public double time;
		public double maxSpeed;
	}

	public static void waitUntilRouteAlmostEnded(int car, int points_left, WaitRouteResult result) {
		double maxUwbDiff = 0.0;
		double maxSpeed = 0.0;
		long timeStart = System.nanoTime();

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
				if (st.speed() > maxSpeed) {
					maxSpeed = st.speed();
				}
			} catch (InterruptedException e) {
				e.printStackTrace();
				break;
			}
		}

		out.println("Max UWB diff: " + maxUwbDiff + " m");

		if (result != null) {
			result.maxUwbDiff = maxUwbDiff;
			result.maxSpeed = maxSpeed;
			result.time = (double)(System.nanoTime() - timeStart) * 1.0e-9;
		}
	}

	public static void waitUntilRouteAlmostEndedAndLog(int car, int points_left, String filename) {
		double accDist = 0.0;
		PrintWriter writer;

		RpPoint pLast = new RpPoint();
		double yaw; 

		try {
			writer = new PrintWriter(filename, "UTF-8");
		} catch (java.io.FileNotFoundException e) {
			e.printStackTrace();
			return;
		} catch (java.io.UnsupportedEncodingException e) {
			e.printStackTrace();
			return;
		}

		CAR_STATE st = getCarState(car, 1000);
		pLast.x = st.px();
		pLast.y = st.py();
		yaw = st.yaw(); 
		
		writer.println("");
		writer.println("% Accumulated distance, Abs error distance," +
						"Yaw diff, Speed, Abs error x, Abs error y, ms_today");
		writer.println("log_data = [");
		
		while (st.ap_route_left() > points_left) {
			try {
				Thread.sleep(50);
				st = getCarState(car, 1000);
				RpPoint pNow = new RpPoint();
				RpPoint pNowUwb = new RpPoint();
				pNow.x = st.px();
				pNow.y = st.py();
				pNowUwb.x = st.px_uwb();
				pNowUwb.y = st.py_uwb();
				double dist = pointDistance(pLast,pNow);
				accDist += dist;

				double uwbDiff = pointDistance(pNow, pNowUwb);
				double speed   = st.speed();
				
				double xDiff = Math.abs(st.px() - st.px_uwb());
				double yDiff = Math.abs(st.py() - st.py_uwb());

				double angChange = Math.abs(st.yaw() - yaw);
				yaw = st.yaw();

				writer.println(String.valueOf(accDist) + ", " +
						String.valueOf(uwbDiff) + ", " +
						String.valueOf(angChange) + ", " +
						String.valueOf(speed) + ", " +
						String.valueOf(xDiff) + ", " +
						String.valueOf(yDiff) + ", " +
						String.valueOf(st.ms_today()) + "; ");
				writer.flush();

				pLast.x = st.px();
				pLast.y = st.py();
			} catch (InterruptedException e) {
				e.printStackTrace();
				break;
			}
		}
		
		writer.println("];");
		writer.println("");
		writer.close();
/*
		out.println("Max UWB diff: " + maxUwbDiff + " m");

		if (result != null) {
			result.maxUwbDiff = maxUwbDiff;
			result.maxSpeed = maxSpeed;
			result.time = (double)(System.nanoTime() - timeStart) * 1.0e-9;
		}
		*/
	}
	
	public static void saveRouteToXml(int car, int mapRoute, String fileName) {
		List<RpPoint> routePoints = getRoute(car, mapRoute, 1000);
		
		if (!fileName.toLowerCase().endsWith(".xml")) {
			fileName += ".xml";
		}
		
		if (mapRoute < 0) {
			mapRoute = 0;
		}

		try {
			DocumentBuilderFactory documentFactory = DocumentBuilderFactory.newInstance();
			DocumentBuilder documentBuilder = documentFactory.newDocumentBuilder();
			Document document = documentBuilder.newDocument();
			
			Element routes = document.createElement("routes");
            document.appendChild(routes);
            
            Element route = document.createElement("route");
            routes.appendChild(route);
            
            Element id = document.createElement("id");
            id.appendChild(document.createTextNode("" + mapRoute));
            route.appendChild(id);
            
            for (RpPoint p: routePoints) {
            	Element point = document.createElement("point");
            	route.appendChild(point);
            	
            	Element x = document.createElement("x");
                x.appendChild(document.createTextNode("" + p.x));
                point.appendChild(x);
                
                Element y = document.createElement("y");
                y.appendChild(document.createTextNode("" + p.y));
                point.appendChild(y);
                
                Element speed = document.createElement("speed");
                speed.appendChild(document.createTextNode("" + p.speed));
                point.appendChild(speed);
                
                Element time = document.createElement("time");
                time.appendChild(document.createTextNode("" + p.time));
                point.appendChild(time);
            }
			
			TransformerFactory transformerFactory = TransformerFactory.newInstance();
            Transformer transformer = transformerFactory.newTransformer();
            
            transformer.setOutputProperty(OutputKeys.INDENT, "yes");
            transformer.setOutputProperty(OutputKeys.METHOD, "xml");
            transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "4");
            
            DOMSource domSource = new DOMSource(document);
            StreamResult streamResult = new StreamResult(new File(fileName));
 
            transformer.transform(domSource, streamResult);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public static void waitUntilRouteAlmostEnded(int car, int points_left) {
		waitUntilRouteAlmostEnded(car, points_left, null);
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
			e.printStackTrace();
		}
	}

	public static void waitUntilStopPolling(int car, int timeoutMs) {
		int timeLeft = timeoutMs;

		try {
			while (timeLeft > 0) {
				int sleep = 50;
				if (sleep > timeLeft) {
					sleep = timeLeft;
				}

				Thread.sleep(sleep);
				CAR_STATE s = getCarState(car, 1000);
				timeLeft -= sleep;

				if (s.speed() < 0.1) {
					break;
				}
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	public static void brakeAndWaitUntilStoppedPolling(int car, double brakeCurrent) {
		RControlStationCommLibrary.rcsc_setAutopilotActive(car, false, 2000);
		RControlStationCommLibrary.rcsc_rcControl(car, 3, brakeCurrent, 0.0);
		waitUntilStopPolling(car, 5000);
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
			int carRoute, int aheadMargin, int genAttempts, boolean tryShorten) {
		return followRecoveryRouteV2(car, recoveryRoute, ri, carRoute, aheadMargin, genAttempts, tryShorten, false);
	}

	/**
	 * Generate and follow a route that connects with the recovery route, and then follow the recovery route.
	 *
	 * @param car
	 * The car ID.
	 *
	 * @param recoveryRoute
	 * The ID of the recovery route.
	 *
	 * @param ri
	 * RouteInfo to make sure that the connecting route is valid.
	 *
	 * @param carRoute
	 * The route number to use when uploading the route to the car.
	 *
	 * @param aheadMargin
	 * Margin when generating random segments in an attempt to connect with the recovery route.
	 *
	 * @param genAttempts
	 * Attempts to generate. More attempts are likely to generate a shorter connecting route.
	 *
	 * @param genOnly
	 * If set the route is only generated and shown on the map, but the autopilot is not activated.
	 *
	 * @param tryShorten
	 * Try to shorten the generated recovery route.
	 *
	 * @return
	 * True for success, false if it was not possible to generate a connecting route or if the car
	 * was outside the valid area in the RouteInfo.
	 */
	public static boolean followRecoveryRouteV2(int car, int recoveryRoute, RouteInfo ri,
			int carRoute, int aheadMargin, int genAttempts, boolean tryShorten, boolean genOnly) {

		long timeStart = System.nanoTime();

		// Settings
		boolean debugPrint = false;
		int maxParts = 80;
		int genPoints = 4;

		boolean res = false;
		List<RpPoint> rec = getRoute(car, recoveryRoute, 1000);
		CAR_STATE st = getCarState(car, 1000);

		double ang = -st.yaw() * Math.PI / 180.0;
		double speed = rec.get(0).speed();
		int validRoutes = 0;
		int discardedRoutes = 0;
		boolean direct = true;

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
					recStartNew.clear();
					recStartNew.addAll(recStartNow);
					ok = tryConnect(recStartNew, rec, ri);
					if (ok) {
						recStartNew.remove(recStartNew.size() - 1);
						recStartNew.remove(recStartNew.size() - 1);
						recIndexNow = 0;
						recLenLeftNow = routeLen(rec);
						validRoutes++;
						break;
					}

					double minLenLeft = routeLen(rec) +
							pointDistance(recStartNew.get(recStartNew.size() - 1), rec.get(0));

					if (!recStartBest.isEmpty() &&
							(routeLen(recStartNew) + minLenLeft) > (routeLen(recStartBest) + recLenLeftBest)) {
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
					direct = false;
					if (sizeOld == recStartNow.size()) {
						// No new points added, most likely stuck. Start over...
						break;
					}
				}

				if (ok) {
					//					shortenRouteMore(recStartNew, ri);
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

				if (ok && direct) {
					// Direct path found
					break;
				}
			}

			if (!recStartBest.isEmpty()) {
				for (int i = 0;i < recIndexBest;i++) {
					rec.remove(0);
				}

				recStartBest.add(rec.get(0));
				rec.remove(0);

				if (tryShorten) {
					shortenRouteMore(recStartBest, ri);
				}

				addRoute(car, recStartBest, false, false, carRoute, 1000);

				res = true;
			} else {
				out.println("[ERROR] Unable to generate valid recovery route start");
			}
		} else {
			out.println("[ERROR] Car seems to be outside of valid polygon");
		}

		if (res) {
			NumberFormat nf2 = NumberFormat.getNumberInstance(Locale.UK);
			nf2.setMaximumFractionDigits(2);
			nf2.setMinimumFractionDigits(2);
			nf2.setGroupingUsed(false);
			if (direct) {
				out.println("Found direct valid recovery route in " +
						nf2.format((System.nanoTime() - timeStart) / 1.0e6) + " ms");
			} else {
				out.println("Found " + validRoutes + " valid recovery routes in " +
						nf2.format((System.nanoTime() - timeStart) / 1.0e6) + " ms (" + discardedRoutes +
						" discarded, " + (genAttempts - discardedRoutes - validRoutes) + " failed). " +
						"Using the shortest one (" + nf2.format((routeLen(recStartBest) + routeLen(rec))) + " m).");
			}

			if (!genOnly) {
				addRoute(car, rec, false, false, -2, 1000);
				RControlStationCommLibrary.rcsc_setAutopilotActive(car, true, 2000);
				waitPolling(car, 500);
				waitUntilRouteAlmostEnded(car, carRoute);
				RControlStationCommLibrary.rcsc_setAutopilotActive(car, false, 2000);
			}
		}

		return res;
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

	/**
	 * Simple way to shorten route while keeping the start and end segments intact.
	 * 
	 * @param r
	 * The route to shorten.
	 * 
	 * @param ri
	 * RouteInfo to make sure the shorter route is valid.
	 */
	public static void shortenRoute(List<RpPoint> r, RouteInfo ri) {
		boolean debug = false;
		long startTime = System.nanoTime();

		boolean shorter = true;
		List<RpPoint> current = new ArrayList<RpPoint>();
		current.addAll(r);
		int iterations = 0;

		while (shorter) {
			shorter = false;

			for (int start = 2;start < current.size() - 1;start++) {
				for (int end = current.size() - 2;end > start;end--) {
					List<RpPoint> tmp = new ArrayList<RpPoint>();
					tmp.addAll(current.subList(0, start));
					tmp.addAll(current.subList(end, current.size()));

					if (ri.isRouteOk(tmp)) {
						shorter = true;
						current.clear();
						current.addAll(tmp);
						iterations++;
						break;
					}
				}

				if (shorter) {
					break;
				}
			}
		}

		if (debug) {
			double lenStart = routeLen(r);
			double lenEnd = routeLen(current);
			int pointsStart = r.size();
			int pointsEnd = current.size();

			NumberFormat nf2 = NumberFormat.getNumberInstance(Locale.UK);
			nf2.setMaximumFractionDigits(2);
			nf2.setMinimumFractionDigits(2);
			nf2.setGroupingUsed(false);
			System.out.println("Route shortened with " + nf2.format(lenStart - lenEnd) + " m in " +
					nf2.format((double)(System.nanoTime() - startTime) / 1.0e6) + " ms" + 
					" (iterations: " + iterations + ", removed: " + (pointsStart - pointsEnd) + ")");
		}

		r.clear();
		r.addAll(current);
	}

	/**
	 * Shorten route while keeping the start and end segments intact.
	 * 
	 * @param r
	 * The route to shorten.
	 * 
	 * @param ri
	 * RouteInfo to make sure the shorter route is valid.
	 */
	public static void shortenRouteMore(List<RpPoint> r, RouteInfo ri) {
		boolean debug = true;
		long startTime = System.nanoTime();
		int shorteningPasses = 0;
		int okChecks = 0;
		int pointsStart = r.size();
		double lenStart = routeLen(r);

		shortenRoute(r, ri);

		double interpol = 1.5;
		double endDiff = 0.02;

		interpolateRoute(r, interpol);

		boolean shorter = true;
		while (shorter) {
			shorter = false;

			for (int i = 1;i < (r.size() - 1);i++) {				
				List<List<RpPoint>> added = generateArcs(r.get(i - 1), r.get(i), ri);

				for (List<RpPoint> pList: added) {
					double rLenOld = routeLen(r);
					for (int subList = 1;subList <= pList.size();subList++) {
						for (int j = r.size() - 2;j > i - 1;j--) {
							List<RpPoint> tmp = new ArrayList<RpPoint>();
							tmp.addAll(r.subList(0, i + 1));
							int ind1 = tmp.size() - 1;
							tmp.addAll(pList.subList(0, subList));
							int ind2 = tmp.size() + 2;
							tmp.addAll(r.subList(j, r.size()));

							okChecks++;

							if ((routeLen(tmp) + endDiff) < rLenOld && ri.isRouteOk(tmp.subList(ind1, ind2))) {
								r.clear();
								r.addAll(tmp);
								interpolateRoute(r, interpol);
								shorter = true;
								shorteningPasses++;
								break;
							}
						}

						if (shorter) {
							break;
						}
					}
				}
			}
		}

		shortenRoute(r, ri);

		if (debug) {
			int pointsEnd = r.size();
			double lenEnd = routeLen(r);

			NumberFormat nf2 = NumberFormat.getNumberInstance(Locale.UK);
			nf2.setMaximumFractionDigits(2);
			nf2.setMinimumFractionDigits(2);
			nf2.setGroupingUsed(false);
			System.out.println("Route shortened with " + nf2.format(lenStart - lenEnd) + " m in " +
					nf2.format((double)(System.nanoTime() - startTime) / 1.0e6) + " ms" + 
					" (passes: " + shorteningPasses + ", okChecks: " + okChecks + ", removed: " +
					(pointsStart - pointsEnd) + ")");
		}
	}

	/**
	 * Try to connect two route using arcs and a line segment.
	 * 
	 * @param r1
	 * The first route to connect.
	 * 
	 * @param r2
	 * The route to connect the first route to. This route will be extended
	 * with the connection.
	 * 
	 * @param ri
	 * RouteInfo to determine if the connected route is valid.
	 * 
	 * @return
	 * True if the connection was successful, false otherwise.
	 */
	public static boolean tryConnect(List<RpPoint> r1, List<RpPoint> r2, RouteInfo ri) {
		List<List<RpPoint>> added1 = generateArcs(r1.get(r1.size() - 2), r1.get(r1.size() - 1), ri);
		List<List<RpPoint>> added2 = generateArcs(r2.get(1), r2.get(0), ri);

		for (List<RpPoint> pList2: added2) {
			for (int subList2 = 0;subList2 <= pList2.size();subList2++) {
				List<RpPoint> p2Now = new ArrayList<RpPoint>();
				p2Now.addAll(pList2.subList(0, subList2));
				Collections.reverse(p2Now);

				for (List<RpPoint> pList1: added1) {
					for (int subList1 = 0;subList1 <= pList1.size();subList1++) {
						List<RpPoint> tmp = new ArrayList<RpPoint>();
						tmp.addAll(r1);
						int ind1 = tmp.size() - 2;
						tmp.addAll(pList1.subList(0, subList1));
						tmp.addAll(p2Now);
						tmp.addAll(r2.subList(0, 2));

						if (ri.isRouteOk(tmp.subList(ind1, tmp.size()))) {
							r1.clear();
							r1.addAll(tmp);
							return true;
						}
					}
				}
			}
		}

		return false;
	}

	/**
	 * Generate two arcs, one in each direction, connected to a segment.
	 * 
	 * @param p1
	 * First point of segment.
	 * 
	 * @param p2
	 * Second point of segment, where arc will be connected.
	 * 
	 * @param ri
	 * RouteInfo to determine if arc is valid (arc will be cut shorter if it has collisions)
	 * 
	 * @return
	 * List with two lists of points defining the left and right arcs.
	 */
	public static List<List<RpPoint>> generateArcs(RpPoint p1, RpPoint p2, RouteInfo ri) {
		double minDist = 0.7;
		double maxAng = PI / 6.1;

		List<List<RpPoint>> res = new ArrayList<List<RpPoint>>();
		List<RpPoint> added1Left = new ArrayList<RpPoint>();
		List<RpPoint> added1Right = new ArrayList<RpPoint>();
		added1Left.add(p1);
		added1Left.add(p2);
		added1Right.addAll(added1Left);
		for (int j = 0;j < 11;j++) {
			RpPoint n = new RpPoint(p1);
			int ind1 = added1Left.size() - 1;
			double a1 = atan2(added1Left.get(ind1).py() - added1Left.get(ind1 - 1).py(),
					added1Left.get(ind1).px() - added1Left.get(ind1 - 1).px());
			n.px(added1Left.get(ind1).px() + minDist * cos(a1 - maxAng));
			n.py(added1Left.get(ind1).py() + minDist * sin(a1 - maxAng));
			if (ri.isPointWithinRoutePolygon(n)) {
				added1Left.add(n);
			}

			n = new RpPoint(p1);
			ind1 = added1Right.size() - 1;
			a1 = atan2(added1Right.get(ind1).py() - added1Right.get(ind1 - 1).py(),
					added1Right.get(ind1).px() - added1Right.get(ind1 - 1).px());
			n.px(added1Right.get(ind1).px() + minDist * cos(a1 + maxAng));
			n.py(added1Right.get(ind1).py() + minDist * sin(a1 + maxAng));
			if (ri.isPointWithinRoutePolygon(n)) {
				added1Right.add(n);
			}
		}
		res.add(added1Left.subList(2, added1Left.size()));
		res.add(added1Right.subList(2, added1Right.size()));

		return res;
	}

	/**
	 * Add points between points on a route where the distance between two points
	 * is greater than maxSegmentLength.
	 * 
	 * @param r
	 * The route to interpolate with points.
	 * 
	 * @param maxSegmentLen
	 * The maximum distance between consecutive points before points are added.
	 */
	public static void interpolateRoute(List<RpPoint> r, double maxSegmentLen) {
		boolean added = true;
		while (added) {
			added = false;
			for (int i = 1;i < r.size();i++) {
				RpPoint prev = r.get(i - 1);
				RpPoint now  = r.get(i);

				if (pointDistance(prev, now) > maxSegmentLen) {
					RpPoint mid = new RpPoint(prev);
					mid.px((prev.px() + now.px()) / 2.0);
					mid.py((prev.py() + now.py()) / 2.0);
					mid.speed((prev.speed() + now.speed()) / 2.0);
					mid.time((prev.time() + now.time()) / 2);
					r.add(i, mid);
					added = true;
					break;
				}
			}
		}
	}

	public static double pointDistance(RpPoint p1, RpPoint p2) {
		return sqrt(pow(p2.px() - p1.px(), 2) + pow(p2.py() - p1.py(), 2));
	}

	public static double pointDistance(double p1x, double p1y, double p2x, double p2y) {
		return sqrt(pow(p2x - p1x, 2) + pow(p2y - p1y, 2));
	}

	public static double angleBetweenLines(
			double px1, double py1, double px2, double py2,
			double qx1, double qy1, double qx2, double qy2) {
		double a1 = atan2(py2 - py1, px2 - px1);
		double a2 = atan2(qy2 - qy1, qx2 - qx1);
		double diff = a2 - a1;

		if (diff > PI) {
			diff -= 2.0 * PI;
		} else if (diff < -PI) {
			diff += 2.0 * PI;
		}

		return diff;
	}
}
