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
import java.util.SplittableRandom;
import rcontrolstationcomm.Utils.RpPoint;

import static java.lang.Math.*;
import static java.lang.System.out;

public class RouteInfo {
	private double mXMin;
	private double mXMax;
	private double mYMin;
	private double mYMax;
	private double mLength;
	private List<RpPoint> mRoute;
	private List<List<RpPoint>> mCutouts;
	private SplittableRandom mRandom;
	private int mLastOuterAttempts;
	private int mLastGeneratedPoints;
	private boolean mDebugEn;

	public RouteInfo() {
		mXMin = 0.0;
		mXMax = 0.0;
		mYMin = 0.0;
		mYMax = 0.0;
		mLength = 0.0;
		mRoute = null;
		mCutouts = null;
		mRandom = new SplittableRandom();
		mLastOuterAttempts = 0;
		mLastGeneratedPoints = 0;
		mDebugEn = true;
	}
	
	public void setDebug(boolean enabled) {
		mDebugEn = enabled;
	}
	
	public boolean debugEnabled() {
		return mDebugEn;
	}

	public RouteInfo(List<RpPoint> route) {
		setRoute(route);
	}

	public void setRoute(List<RpPoint> route) {
		if (route == null || route.size() == 0) {
			return;
		}

		mRoute = route;
		mCutouts = null;
		mXMin = route.get(0).px();
		mXMax = route.get(0).px();
		mYMin = route.get(0).py();
		mYMax = route.get(0).py();
		mLength = 0.0;
		mRandom = new SplittableRandom();
		mLastOuterAttempts = 0;
		mLastGeneratedPoints = 0;
		mDebugEn = true;

		for (int i = 0;i < route.size();i++) {
			RpPoint p = route.get(i);
			RpPoint pp = i > 0 ? route.get(i - 1) : p;
			mLength += Utils.pointDistance(p, pp);

			if (p.px() < mXMin) {
				mXMin = p.px();
			}

			if (p.px() > mXMax) {
				mXMax = p.px();
			}

			if (p.py() < mYMin) {
				mYMin = p.py();
			}

			if (p.py() > mYMax) {
				mYMax = p.py();
			}
		}
	}
	
	public void addCutout(List<RpPoint> route) {
		if (mCutouts == null) {
			mCutouts = new ArrayList<List<RpPoint>>();
		}
		
		mCutouts.add(route);
	}

	public boolean hasRoute() {
		return mRoute != null;
	}

	public double xMin() {
		return mXMin;
	}

	public double xMax() {
		return mXMax;
	}

	public double yMin() {
		return mYMin;
	}

	public double yMax() {
		return mYMax;
	}

	public double length() {
		return mLength;
	}

	public boolean isPointWithinRoutePolygon(double px, double py) {
		if (mRoute == null || mRoute.size() < 3) {
			return false;
		}

		int nVert = mRoute.size();
		int i, j;
		boolean c = false;

		for (i = 0, j = nVert - 1;i < nVert;j = i++) {
			double vxi = mRoute.get(i).px();
			double vyi = mRoute.get(i).py();
			double vxj = mRoute.get(j).px();
			double vyj = mRoute.get(j).py();

			if (((vyi > py) != (vyj > py)) && 
					(px < (vxj-vxi) * (py-vyi) / (vyj-vyi) + vxi)) {
				c = !c;
			}
		}
		
		// Should not be within any of the cutouts
		if (c && mCutouts != null) {
			for (List<RpPoint> r: mCutouts) {
				nVert = r.size();
				c = false;

				for (i = 0, j = nVert - 1;i < nVert;j = i++) {
					double vxi = r.get(i).px();
					double vyi = r.get(i).py();
					double vxj = r.get(j).px();
					double vyj = r.get(j).py();

					if (((vyi > py) != (vyj > py)) && 
							(px < (vxj-vxi) * (py-vyi) / (vyj-vyi) + vxi)) {
						c = !c;
					}
				}
				
				if (c) {
					c = false;
					break;
				} else {
					c = true;
				}
			}
		}

		return c;
	}
	
	public boolean isPointWithinRoutePolygon(RpPoint p) {
		return isPointWithinRoutePolygon(p.px(), p.py());
	}

	public boolean isSegmentWithinRoutePolygon(RpPoint p1, RpPoint p2) {
		boolean res = true;

		for (int j = 1;j < mRoute.size();j++) {
			RpPoint q1 = mRoute.get(j - 1);
			RpPoint q2 = mRoute.get(j);

			if (lineIntersect(p1, p2, q1, q2)) {
				res = false;
				break;
			}
		}

		if (mCutouts != null) {			
			for (List<RpPoint> r: mCutouts) {
				for (int j = 1;j < r.size();j++) {
					RpPoint q1 = r.get(j - 1);
					RpPoint q2 = r.get(j);

					if (lineIntersect(p1, p2, q1, q2)) {
						res = false;
						break;
					}
				}
			}
		}

		if (res) {
			res = isPointWithinRoutePolygon(p1.px(), p1.py());
		}

		return res;
	}
	
	public boolean closestLineIntersection(double p0_x, double p0_y,
			double p1_x, double p1_y, RpPoint coll) {
		if (mRoute == null || mRoute.size() < 2) {
			return false;
		}
		
		boolean res = false;
		RpPoint collLast = new RpPoint();
		double lastDist = 0.0;
		RpPoint pStart = new RpPoint(p0_x, p0_y);
		
		for (int i = 1;i < mRoute.size();i++) {
			if (getLineIntersection(p0_x, p0_y, p1_x, p1_y,
					mRoute.get(i - 1).px(), mRoute.get(i - 1).py(),
					mRoute.get(i).px(), mRoute.get(i).py(), collLast)) {
				
				if (res) {
					double dist = lastDist = Utils.pointDistance(pStart, collLast);
					if (dist < lastDist) {
						coll.setTo(collLast);
						lastDist = dist;
					}
				} else {
					coll.setTo(collLast);
					lastDist = Utils.pointDistance(pStart, collLast);
				}
				
				res = true;
			}
		}

		if (mCutouts != null) {			
			for (List<RpPoint> r: mCutouts) {
				for (int i = 1;i < r.size();i++) {
					if (getLineIntersection(p0_x, p0_y, p1_x, p1_y,
							r.get(i - 1).px(), r.get(i - 1).py(),
							r.get(i).px(), r.get(i).py(), collLast)) {

						if (res) {
							double dist = lastDist = Utils.pointDistance(pStart, collLast);
							if (dist < lastDist) {
								coll.setTo(collLast);
								lastDist = dist;
							}
						} else {
							coll.setTo(collLast);
							lastDist = Utils.pointDistance(pStart, collLast);
						}

						res = true;
					}
				}
			}
		}

		return res;
	}

	public List<RpPoint> generateRouteWithin(int length,
			List<RpPoint> previous, double speed) {
		
		long timeStart = System.nanoTime();

		List<RpPoint> r = new ArrayList<RpPoint>();
		List<RpPoint> rLargest = new ArrayList<RpPoint>();

		if (mRoute == null || mRoute.size() < 3) {
			return r;
		}
		
		// Generation settings
		int maxAttemptsOuter = 5000;
		int maxAttemptsInner = 50;
		double minDist = 0.6;
		double maxDist = 2.0;
		double maxAng = PI / 6;
		
		int attemptOuter = 0;
		int genPoints = 0;
		int start = 0;

		while (attemptOuter < maxAttemptsOuter) {
			r.clear();
			if (previous != null) {
				r.addAll(previous);
				start = previous.size();
			}
			
//			int lastStepBack = length + start;
			
			for (int pNow = start;pNow < length + start;pNow++) {
				int attemptInner = 0;
				boolean ok = false;
				double px = 0.0;
				double py = 0.0;
				
				double xMin = mXMin;
				double xMax = mXMax;
				double yMin = mYMin;
				double yMax = mYMax;
				
				if (pNow == 1) {
					double xLast = r.get(pNow - 1).px();
					double yLast = r.get(pNow - 1).py();
					
					xMax = xLast + maxDist;
					xMin = xLast - maxDist;
					yMax = yLast + maxDist;
					yMin = yLast - maxDist;
				} else if (pNow > 1) {
					double xLast1 = r.get(pNow - 1).px();
					double yLast1 = r.get(pNow - 1).py();
					double xLast2 = r.get(pNow - 2).px();
					double yLast2 = r.get(pNow - 2).py();
					
					double a1 = atan2(yLast1 - yLast2, xLast1 - xLast2);
					
					double p1x = xLast1 + maxDist * cos(a1 - maxAng);
					double p1y = yLast1 + maxDist * sin(a1 - maxAng);
					double p2x = xLast1 + maxDist * cos(a1);
					double p2y = yLast1 + maxDist * sin(a1);
					double p3x = xLast1 + maxDist * cos(a1 + maxAng);
					double p3y = yLast1 + maxDist * sin(a1 + maxAng);
					
					RpPoint coll = new RpPoint();
					
					if (closestLineIntersection(xLast1, yLast1, p1x, p1y, coll)) {
						p1x = coll.px();
						p1y = coll.py();
					}
					
					if (closestLineIntersection(xLast1, yLast1, p2x, p2y, coll)) {
						p2x = coll.px();
						p2y = coll.py();
					}
					
					if (closestLineIntersection(xLast1, yLast1, p3x, p3y, coll)) {
						p3x = coll.px();
						p3y = coll.py();
					}
					
					xMax = maxFrom4(xLast1, p1x, p2x, p3x);
					xMin = minFrom4(xLast1, p1x, p2x, p3x);
					yMax = maxFrom4(yLast1, p1y, p2y, p3y);
					yMin = minFrom4(yLast1, p1y, p2y, p3y);
					
					if (Utils.pointDistance(xMax, yMax, xMin, yMin) < minDist) {
						break;
					}
				}
				
				if (xMax > mXMax) {
					xMax = mXMax;
				}
				if (xMin < mXMin) {
					xMin = mXMin;
				}
				if (yMax > mYMax) {
					yMax = mYMax;
				}
				if (yMin < mYMin) {
					yMin = mYMin;
				}

				while (attemptInner < maxAttemptsInner) {
					px = randInRange(xMin, xMax);
					py = randInRange(yMin, yMax);
					
					attemptInner++;
					genPoints++;
					
					ok = true;

					if (pNow == 0) {
						if (!isPointWithinRoutePolygon(px, py)) {
							ok = false;
							continue;
						}
					} else {
						RpPoint p1 = r.get(pNow - 1);
						RpPoint p2 = new RpPoint();
						p2.px(px);
						p2.py(py);

						if (!isSegmentWithinRoutePolygon(p1, p2)) {
							ok = false;
							continue;
						}
						
						if (Utils.pointDistance(p1, p2) < minDist) {
							ok = false;
							continue;
						}
						
						if (pNow > 1) {
							double px1 = r.get(pNow - 2).px();
							double py1 = r.get(pNow - 2).py();
							double px2 = r.get(pNow - 1).px();
							double py2 = r.get(pNow - 1).py();
							double qx1 = r.get(pNow - 1).px();
							double qy1 = r.get(pNow - 1).py();
							double qx2 = px;
							double qy2 = py;
							
							if (abs(Utils.angleBetweenLines(px1, py1, px2, py2,
									qx1, qy1, qx2, qy2)) > maxAng) {
								ok = false;
								continue;
							}
						}
					}
					
					if (ok) {
						break;
					}
				}

				if (!ok) {
					break;
//					if (pNow >= start && lastStepBack > start) {
//						do {
//							pNow--;
//							r.remove(r.size() - 1);
//						} while (pNow >= lastStepBack);
//						lastStepBack = pNow;
//						pNow--;
//						continue;
//					} else {
//						break;
//					}
				}

				RpPoint p = new RpPoint();
				p.px(px);
				p.py(py);
				p.speed(speed);
				p.time(0);
				r.add(p);
			}
			
			attemptOuter++;
			
			if (r.size() > rLargest.size()) {
				rLargest.clear();
				rLargest.addAll(r);
			}
			
			if (rLargest.size() == (length + start)) {
				break;
			}
		}
		
		mLastOuterAttempts = attemptOuter;
		mLastGeneratedPoints = genPoints;
		
		if (mDebugEn) {
			out.println("Generated points: " + genPoints +
					", Outer loops: " + attemptOuter + 
					", Time: " + (System.nanoTime() - timeStart) / 1000 + " uS");
		}

		return rLargest;
	}

	public List<RpPoint> generateRouteWithin(int length,
			List<RpPoint> previous, double speed, int aheadMargin) {
		
		List<RpPoint> res = generateRouteWithin(length + aheadMargin, previous, speed);
		
		int start = 0;
		if (previous != null) {
			start = previous.size();
		}
		
		int end = length + start;
		if (end > res.size()) {
			end = res.size();
		}

		return res.subList(0, end);
	}
	
	boolean isRouteOk(List<RpPoint> r) {
		boolean res = true;
		
		double maxAng = PI / 6;
				
		if (r.size() == 1) {
			res = isPointWithinRoutePolygon(r.get(0).px(), r.get(0).py());
		} else if (r.size() > 1) {
			for (int i = 1;i < r.size();i++) {
				RpPoint p1 = r.get(i - 1);
				RpPoint p2 = r.get(i);

				if (!isSegmentWithinRoutePolygon(p1, p2)) {
					res = false;
				}

				if (i > 1) {
					double px1 = r.get(i - 2).px();
					double py1 = r.get(i - 2).py();
					double px2 = r.get(i - 1).px();
					double py2 = r.get(i - 1).py();
					double qx1 = r.get(i - 1).px();
					double qy1 = r.get(i - 1).py();
					double qx2 = p2.px();
					double qy2 = p2.py();

					if (abs(Utils.angleBetweenLines(px1, py1, px2, py2,
							qx1, qy1, qx2, qy2)) > maxAng) {
						res = false;
						break;
					}
				}
			}
		}
				
		return res;
	}
	
	public void setRandomSeed(long seed) {
		mRandom = new SplittableRandom(seed);
	}
	
	public int getLastOuterAttempts() {
		return mLastOuterAttempts;
	}
	
	public int getLastGeneratedPoints() {
		return mLastGeneratedPoints;
	}
	
	public double randInRange(double min, double max) {
	    return mRandom.nextDouble() * (max - min) + min;
	}
	
	private static double minFrom4(double a, double b, double c, double d) {
		double res = a;
		if (b < res) {
			res = b;
		}
		if (c < res) {
			res = c;
		}
		if (d < res) {
			res = d;
		}
		return res;
	}
	
	private static double maxFrom4(double a, double b, double c, double d) {
		double res = a;
		if (b > res) {
			res = b;
		}
		if (c > res) {
			res = c;
		}
		if (d > res) {
			res = d;
		}
		return res;
	}

	private static boolean getLineIntersection(double p0_x, double p0_y, double p1_x, double p1_y, 
			double p2_x, double p2_y, double p3_x, double p3_y, RpPoint coll)
	{
		double s1_x, s1_y, s2_x, s2_y;
		s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
		s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

		double s, t;
		s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
		t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

		if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
			// Collision detected
			coll.px(p0_x + (t * s1_x));
			coll.py(p0_y + (t * s1_y));
			return true;
		}

		return false; // No collision
	}

	private static boolean ccw(RpPoint A, RpPoint B, RpPoint C) {
		return (C.py() - A.py()) * (B.px() - A.px()) > (B.py() - A.py()) * (C.px() - A.px());
	}

	private static boolean lineIntersect(RpPoint A, RpPoint B, RpPoint C, RpPoint D) {
		return ccw(A,C,D) != ccw(B,C,D) && ccw(A,B,C) != ccw(A,B,D);
	}
}
