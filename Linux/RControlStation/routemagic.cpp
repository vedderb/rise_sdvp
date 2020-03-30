/*
    Copyright 2019 Benjamin Vedder benjamin@vedder.se &
                   Bo Joel Svensson svenssonjoel@yahoo.se

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
#include "routemagic.h"
#include <locpoint.h>
#include <attributes_masks.h>

#include <chrono>
using namespace std::chrono;
#include <QDebug>

#include <QFileDialog>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include <QMessageBox>
#include <mapwidget.h>
#include <QtMath>
#include <cassert>

RouteMagic::RouteMagic(QObject *parent) : QObject(parent)
{

}

double RouteMagic::maxFrom4(double a, double b, double c, double d)
{
    double res = a;
    if (b > res) res = b;
    if (c > res) res = c;
    if (d > res) res = d;
    return res;
}

double RouteMagic::minFrom4(double a, double b, double c, double d)
{
    double res = a;
    if (b < res) res = b;
    if (c < res) res = c;
    if (d < res) res = d;
    return res;
}

double RouteMagic::randInRange(double min, double max)
{
    double range = max - min;
    double k = RAND_MAX / range;
    return min + (static_cast<double>(rand()) / k);
}

bool RouteMagic::isPointWithin(double px, double py, QList<LocPoint> route)
{
    if (route.size() < 3) {
        return false;
    }
    int nVert = route.size();
    int i,j;
    bool c = false;

    for (i = 0, j = nVert -  1; i < nVert; j = i ++) {
        double vxi = route.at(i).getX();
        double vyi = route.at(i).getY();
        double vxj = route.at(j).getX();
        double vyj = route.at(j).getY();

        if (((vyi > py) != (vyj > py)) &&
            (px < (vxj-vxi) * (py-vyi) / (vyj-vyi) + vxi)) {
            c = !c;
        }
    }
    return c;
}

bool RouteMagic::isPointWithin(LocPoint p, QList<LocPoint> route)
{
    return isPointWithin(p.getX(),p.getY(),route);
}

bool RouteMagic::isPointOutside(double px, double py, QList<LocPoint> route)
{
    if (route.size() < 3) {
        return true;
    }
    int nVert = route.size();
    int i,j;
    bool c = true;

    for (i = 0, j = nVert-1; i < nVert; j = i++) {
        double vxi = route.at(i).getX();
        double vyi = route.at(i).getY();
        double vxj = route.at(j).getX();
        double vyj = route.at(j).getY();
        if (((vyi > py) != (vyj > py)) &&
                ( px < (vxj-vxi) * (py-vyi) / (vyj-vyi) + vxi)) {
            c = !c;
        }
    }
    return c;
}

bool RouteMagic::isPointOutside(LocPoint p, QList<LocPoint> route)
{
    return isPointOutside(p.getX(),p.getY(), route);
}

bool RouteMagic::isPointOutside(double px, double py, QList<QList<LocPoint> > cutouts)
{
    for (auto c : cutouts)  {
        if (!isPointOutside(px,py,c)) return false;
    }
    return true;
}

bool RouteMagic::isPointOutside(LocPoint p, QList<QList<LocPoint> > cutouts)
{
   return isPointOutside(p.getX(),p.getY(),cutouts);
}


bool RouteMagic::isSegmentWithin(LocPoint p1, LocPoint p2, QList<LocPoint> route)
{
    int i,j;
    int nVert = route.size();

    if (!isPointWithin(p2,route)) return false;

    for (i = 0, j = nVert - 1; i < nVert; j = i ++) {
        LocPoint q1 = route.at(j);
        LocPoint q2 = route.at(i);
        if (lineIntersect(p1, p2, q1, q2)) {
            return false;
        }
    }
    return true;
}

bool RouteMagic::isSegmentOutside(LocPoint p1, LocPoint p2, QList<LocPoint> route)
{
    if ( ! (isPointOutside(p1,route) && isPointOutside(p2, route))) return false;

    int i,j;
    int nVert = route.size();

    for (i = 0, j = nVert - 1; i < nVert; j = i ++) {
        LocPoint q1 = route.at(j);
        LocPoint q2 = route.at(i);
        if (lineIntersect(p1, p2, q1, q2)) {
            return false;
        }
    }
    return true;
}

bool RouteMagic::isSegmentOutside(LocPoint p1, LocPoint p2, QList<QList<LocPoint>> cutouts){
    for (auto c : cutouts) {
        if (!isSegmentOutside(p1,p2,c)) {
            return false;
        }
    }
    return true;
}

bool RouteMagic::ccw(LocPoint a, LocPoint b, LocPoint c)
{
    return (c.getY() - a.getY()) * (b.getX() - a.getX())
            > (b.getY() - a.getY()) * (c.getX() - a.getX());
}

bool RouteMagic::lineIntersect(LocPoint a, LocPoint b, LocPoint c, LocPoint d)
{
   return ccw(a,c,d) != ccw(b,c,d) && ccw(a,b,c) != ccw(a,b,d);
}

bool RouteMagic::boxesIntersect(QList<LocPoint> b0, QList<LocPoint> b1)
{
    if (b0.size() != 4 || b1.size() != 4) return false; // malformed boxes.

    int i0,j0;
    int i1,j1;
    for (i0 = 0, j0 = 3;i0 < 4; j0 = i0++) {
        for (i1 = 0, j1 = 3; i1 < 4; j1 = i1++) {
            if (RouteMagic::lineIntersect(b0.at(j0),b0.at(i0),
                              b1.at(j1),b1.at(i1))) {
                    return true;
            }
        }
    }
    return false;
}

bool RouteMagic::intersectionExists(QList<LocPoint> points0, QList<LocPoint> points1)
{
    if (points0.size() < 2 || points1.size() < 2)
        return false;

    for (int i = 1; i < points0.size(); i++)
        for (int j = 1; j < points1.size(); j++)
            if (lineIntersect(points0.at(i-1), points0.at(i), points1.at(j-1), points1.at(j)))
                return true;

    return false;
}

QList<LocPoint> RouteMagic::getAllIntersections(QList<LocPoint> points0, QList<LocPoint> points1)
{
    QList<LocPoint> intersections;
    if (points0.size() < 2 || points1.size() < 2)
        return intersections;

    for (int i = 1; i < points0.size(); i++)
        for (int j = 1; j < points1.size(); j++)
            if (lineIntersect(points0.at(i-1), points0.at(i), points1.at(j-1), points1.at(j))) {
                intersections.append(getLineIntersection(QPair<LocPoint, LocPoint>(points0.at(i-1), points0.at(i)),
                                                         QPair<LocPoint, LocPoint>(points1.at(j-1), points1.at(j))));
            }

    return intersections;
}

LocPoint RouteMagic::getLineIntersection(QPair<LocPoint, LocPoint> line0, QPair<LocPoint, LocPoint> line1) // intersection can lie outside of segments!
{
    double x1=line0.first.getX(), y1=line0.first.getY(), x2=line0.second.getX(), y2=line0.second.getY(),
            x3=line1.first.getX(), y3=line1.first.getY(), x4=line1.second.getX(), y4=line1.second.getY();

    // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
    double xIntersect = ( ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4)) / ((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)) );
    double yIntersect = ( ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4)) / ((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)) );

    // qDebug() << "(" << x1 << "," << y1 << ")---(" << x2 << "," << y2 << ")  /  (" << x3 << "," << y3 << ")---(" << x4 << "," << y4 << ") => (" << intersection.getX() << "," << intersection.getY() << ")\n";

    return LocPoint(xIntersect, yIntersect);
}

QList<LocPoint> RouteMagic::getAllIntersections(QList<LocPoint> route)
{
    QList<LocPoint> intersections;
    if (route.size() < 2)
        return intersections;

    for (int i = 1; i < route.size(); i++)
        for (int j = i+2; j < route.size(); j++)
            if (lineIntersect(route.at(i-1), route.at(i), route.at(j-1), route.at(j)))
                intersections.append(getLineIntersection(QPair<LocPoint, LocPoint>(route.at(i-1), route.at(i)),
                                                         QPair<LocPoint, LocPoint>(route.at(j-1), route.at(j))));

    return intersections;
}

bool RouteMagic::circlesOverlap(LocPoint p0, double r0, LocPoint p1, double r1)
{
    double dist = p0.getDistanceTo(p1);
    if ((r0 + r1) >= dist) return true;
    return false;
}

bool RouteMagic::lineIntersectsRouteTime(LocPoint p0, LocPoint p1, QList<LocPoint> route)
{
    // Assumes relative-time trajectories.
    // Assumes p1 is in the future relative to p0.

    if (route.size() < 2) return false;

    int i = 1;

    int32_t p0t = p0.getTime();
    int32_t p1t = p1.getTime();

    while (i < route.size()){
        // TEST: Rule out collision with teleporting object
        if (route.at(i-1).getTime() == route.at(i).getTime()) { i ++; continue;}

        if ((p1t >= route.at(i-1).getTime() && (p0t <= route.at(i).getTime()))){
            LocPoint p_last = LocPoint(route.at(i-1).getX(),route.at(i-1).getY());
            LocPoint p_now = LocPoint(route.at(i).getX(),route.at(i).getY());
            if (lineIntersect(p0,p1,p_last,p_now)) return true;
        }
        i++;
    }
    return false;
}

bool RouteMagic::routeIntersectsRouteInTime(QList<LocPoint> t0, QList<LocPoint> t1)
{
    if (t0.size() < 2 || t1.size() < 2) {
        return false;
    }

    LocPoint p0 = t0.at(0);

    for (int i = 1; i < t0.size(); i++) {
        LocPoint p1 = t0.at(i);

        if (lineIntersectsRouteTime(p0,p1, t1)) {
            return true;
        }
        p0 = p1;
    }
    return false;
}

double RouteMagic::angleBetweenLines(double px1, double py1, double px2, double py2,
                                     double qx1, double qy1, double qx2, double qy2)
{
    double a1 = atan2(py2-py1,px2-px1);
    double a2 = atan2(qy2-qy1,qx2-qx1);
    double diff = a2 - a1;
    if (diff > M_PI) {
        diff -= 2.0*M_PI;
    } else if (diff < -M_PI) {
        diff += 2.0*M_PI;
    }
    return diff;
}

double RouteMagic::angleBetweenLines(LocPoint p0a, LocPoint p0b, LocPoint p1a, LocPoint p1b) {
    return angleBetweenLines(p0a.getX(),p0a.getY(),p0b.getX(),p0b.getY(),
                             p1a.getX(),p1a.getY(),p1b.getX(),p1b.getY());
}


bool RouteMagic::getLineIntersection(double p0_x, double p0_y, double p1_x, double p1_y, double p2_x, double p2_y, double p3_x, double p3_y, LocPoint *coll)
{
    double s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    double s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
        // Collision detected
        coll->setX(p0_x + (t * s1_x));
        coll->setY(p0_y + (t * s1_y));
        return true;
    }

    return false; // No collision
}

bool RouteMagic::getLineIntersection(LocPoint p0, LocPoint p1, LocPoint p2, LocPoint p3, LocPoint *coll) {
    return getLineIntersection(p0.getX(),p0.getY(),
                               p1.getX(),p1.getY(),
                               p2.getX(),p2.getY(),
                               p3.getX(),p3.getY(), coll);
}

bool RouteMagic::closestLineIntersection(double p0_x, double p0_y, double p1_x, double p1_y, QList<QList<LocPoint>>routes, LocPoint *coll)
{
    bool res = false;
    LocPoint collLast;
    double lastDist = 100000000.0;
    LocPoint pStart = LocPoint(p0_x, p0_y);

    if (routes.size() > 0) {
        for (QList<LocPoint> r: routes) {
            for (int i = 1;i < r.size();i++) {
                if (getLineIntersection(p0_x, p0_y, p1_x, p1_y,
                                        r.at(i - 1).getX(), r.at(i - 1).getY(),
                                        r.at(i).getX(), r.at(i).getY(), &collLast)) {

                    if (res) {
                        double dist = pStart.getDistanceTo(collLast);
                        if (dist < lastDist) {
                            coll->setX(collLast.getX());
                            coll->setY(collLast.getY());
                            lastDist = dist;
                        }
                    } else {
                        coll->setX(collLast.getX());
                        coll->setY(collLast.getY());
                        lastDist = pStart.getDistanceTo(collLast);
                    }

                    res = true;
                }
            }
        }
    }

    return res;
    //return false;
}

double RouteMagic::distanceToLine(LocPoint p, LocPoint l0, LocPoint l1)
{
    double l = l0.getDistanceTo(l1);
    if (l < 0.01) {
        return p.getDistanceTo(l0);
    } // one cm

    double x = p.getX();
    double y = p.getY();
    double x1 = l0.getX();
    double y1 = l0.getY();
    double x2 = l1.getX();
    double y2 = l1.getY();
    double a = x - x1;
    double b = y - y1;
    double c = x2 - x1;
    double d = y2 - y1;

    double dot = a * c + b * d;
    double len_sq = c * c + d * d;

    double param = dot / len_sq;

    double xx, yy;

    if (param < 0) {
        xx = x1;
        yy = y1;
    }
    else if (param > 1) {
        xx = x2;
        yy = y2;
    }
    else {
        xx = x1 + param * c;
        yy = y1 + param * d;
    }

    double dx = x - xx;
    double dy = y - yy;
    return sqrt(dx * dx + dy * dy);
}

QPair<LocPoint,LocPoint> RouteMagic::getBaselineDeterminingMinHeightOfConvexPolygon(QList<LocPoint> convexPolygon)
{
    // 1. Determine point with max distance for each line
    QList<QPair<QPair<LocPoint,LocPoint>, double>> maxDistances;
    for (int i = 0; i < convexPolygon.size() - 1 /* skip last */; i++) {
        LocPoint l0 = convexPolygon.at(i);
        LocPoint l1 = convexPolygon.at(i+1);

        double maxDistance = -1;
        for (int j = 0; j < convexPolygon.size(); j++) {
            if (j == i || j == i+1)
                continue;

            double currDistance = distanceToLine(convexPolygon.at(j), l0, l1);
            maxDistance = (currDistance > maxDistance)? currDistance : maxDistance;
        }
        maxDistances.append(QPair<QPair<LocPoint,LocPoint>, double>(QPair<LocPoint,LocPoint>(l0, l1), maxDistance));
    }
    assert(maxDistances.size() == convexPolygon.size() - 1);

    // 2. Determine line with minimum distance determined in 1.
    return std::min_element(maxDistances.begin(), maxDistances.end(),
                            [](QPair<QPair<LocPoint,LocPoint>, double> const& a, QPair<QPair<LocPoint,LocPoint>, double> const& b) -> bool
    {return a.second < b.second;})->first;
}

int RouteMagic::getClosestPointInRoute(LocPoint referencePoint, QList<LocPoint> route)
{
    double minDistance = std::numeric_limits<double>::max();
    int minIdx = -1;
    for (int i = 0; i < route.size(); i++) {
        double distance = sqrt(pow((route.at(i).getX()-referencePoint.getX()), 2) + pow((route.at(i).getY()-referencePoint.getY()), 2));
        if (distance < minDistance) {
            minDistance = distance;
            minIdx = i;
        }
    }
    assert(minIdx != -1);

    return minIdx;
}


bool RouteMagic::isRouteDrivable(QList<LocPoint> r,
                                 QList<LocPoint> outerFence,
                                 QList<QList<LocPoint>> cutouts,
                                 double maxAng) {
    bool res = true;

    //double maxAng = M_PI / 6;

    if (r.size() == 1) {
        res = isPointWithin(r.at(0), outerFence);
    } else if (r.size() > 1) {
        for (int i = 1;i < r.size();i++) {
            LocPoint p1 = r.at(i - 1);
            LocPoint p2 = r.at(i);

            if (!isSegmentWithin(p1,p2,outerFence)) {
                return false;
            }
            for (auto c : cutouts) {
                if (!isSegmentOutside(p1,p2,c)) {
                    return false;
                }
            }

            if (i >= 2) {
                double px1 = r.at(i - 2).getX();
                double py1 = r.at(i - 2).getY();
                double px = p1.getX();
                double py = p1.getY();
                double qx = p2.getX();
                double qy = p2.getY();

                if (fabs(angleBetweenLines(px1, py1, px, py,
                                           px, py, qx, qy)) > maxAng) {
                    return false;
                }
            }
        }
    }

    return res;
}

QList<LocPoint> RouteMagic::generateRouteWithin(int length,
                                                QList<LocPoint> prev_traj,
                                                double speed,
                                                int aheadmargin,
                                                QList<LocPoint> outerFence,
                                                QList<QList<LocPoint>> cutouts)
{
    QList<LocPoint> res = generateRouteWithin(length+aheadmargin, prev_traj,speed,outerFence,cutouts);
    int start = prev_traj.size();

    int end = length+start;
    if (end > res.size()) {
        end = res.size();
    }

    res = res.mid(0,end);
    return(res);
}

// Attempt to port return route generation
QList<LocPoint> RouteMagic::generateRecoveryRoute(QList<LocPoint> endSegment, // last points of objects last traj
                                                 QList<LocPoint> recoverTo,  // first points of objects first traj
                                                 int aheadMargin,
                                                 int genAttempts, bool tryShorten,
                                                  QList<LocPoint> outerFence,
                                                  QList<QList<LocPoint>> cutouts) {
    //int64_t timestart= duration_cast<microseconds>(time_point_cast<microseconds>(high_resolution_clock::now()).time_since_epoch()).count();
    // Settings
    bool debugPrint = false;
    int maxParts = 120;
    int genPoints = 5;

    bool res = false;
    QList<LocPoint> rec = recoverTo;
    //List<RpPoint> rec = getRoute(car, recoveryRoute, 1000);
    //CAR_STATE st = getCarState(car, 1000);
    //double ang = atan2(endSegment.at(endSegment.size()-2).getY() - endSegment.last().getY(),
    //                   endSegment.at(endSegment.size()-2).getX() - endSegment.last().getX());
    //double ang = -st.yaw() * Math.PI / 180.0;
    double speed = rec.at(0).getSpeed();
    int validRoutes = 0;
    int discardedRoutes = 0;
    bool direct = true;

    QList<LocPoint> recStart;
    QList<LocPoint> recStartNow;
    QList<LocPoint> recStartNew;
    QList<LocPoint> recStartBest;
    recStart = endSegment;


    if (isRouteDrivable(recStart,outerFence, cutouts, M_PI/6)) {
        int recIndexNow = 0;
        int recIndexBest = 0;
        double recLenLeftNow = 0.0;
        double recLenLeftBest = 0.0;

        for (int i = 0;i < genAttempts;i++) {
            recStartNow.clear();
            recStartNow.append(recStart);
            bool ok = false;
            //bool discarded = false;

            for (int j = 0;j < maxParts;j++) {

                //qDebug() << "Attempt: " << i << " PART: " << j << "Current Length: " << recStartNow.size();

                recStartNew.clear();
                recStartNew.append(recStartNow);
                ok = tryConnect(&recStartNew, &rec, outerFence,cutouts); // See if recStartNew is updated!
                if (ok) {
                    recStartNew.removeLast();
                    recStartNew.removeLast();
                    recIndexNow = 0;
                    recLenLeftNow = routeLen(rec);
                    validRoutes++;
                    break;
                }

                double minLenLeft = routeLen(rec) +
                        recStartNew.at(recStartNew.size() - 1).getDistanceTo(rec.at(0));

                if (!recStartBest.isEmpty() &&
                        (routeLen(recStartNew) + minLenLeft) > (routeLen(recStartBest) + recLenLeftBest)) {
                    // Already longer than the best attempt so far, discard...
                    discardedRoutes++;
                    //discarded = true;
                    break;
                }
                int sizeOld = recStartNow.size();
                recStartNow = generateRouteWithin(genPoints, recStartNow, speed, aheadMargin, outerFence, cutouts);
                direct = false;
                if (sizeOld == recStartNow.size()) {
                    //qDebug() << "NO POINTS ADDED";
                    // No new points added, most likely stuck. Start over...
                    break;
                }
            }

            if (ok) {
                //					shortenRouteMore(recStartNew, ri);
                if (recStartBest.isEmpty()) {
                    recStartBest.append(recStartNew);
                    recIndexBest = recIndexNow;
                    recLenLeftBest = recLenLeftNow;
                } else {
                    if ((routeLen(recStartNew) + recLenLeftNow) < (routeLen(recStartBest) + recLenLeftBest)) {
                        recStartBest.clear();
                        recStartBest.append(recStartNew);
                        recIndexBest = recIndexNow;
                        recLenLeftBest = recLenLeftNow;
                    }
                }
            }

            if (debugPrint) {
                qDebug() << "Attempt " << i << "/" << genAttempts << " OK: " << (ok ? "true" : "false");
            }

            if (ok && direct) {
                // Direct path found
                break;
            }
        }

        if (!recStartBest.isEmpty()) {
            for (int i = 0;i < recIndexBest;i++) {
                rec.removeFirst();
            }

            recStartBest.append(rec.at(0));
            rec.removeFirst();

            //int64_t timestop= duration_cast<microseconds>(time_point_cast<microseconds>(high_resolution_clock::now()).time_since_epoch()).count();
            //int64_t t_diff = timestop - timestart;
            //qDebug() << "recovery generation time: " << t_diff / 1000000 << " sec";

            if (tryShorten) {
                //qDebug() << "trying to shorten route!";
                //int64_t shortentimestart= duration_cast<microseconds>(time_point_cast<microseconds>(high_resolution_clock::now()).time_since_epoch()).count();
                recStartBest = shortenRouteMore(recStartBest, outerFence, cutouts);
                //int64_t shortentimestop = duration_cast<microseconds>(time_point_cast<microseconds>(high_resolution_clock::now()).time_since_epoch()).count();
                //int64_t shortent_diff = shortentimestop - shortentimestart;

                //qDebug() << "shortenroute time: " << shortent_diff/ 1000000 << " sec";

            }

            //addRoute(car, recStartBest, false, false, carRoute, 1000);

            res = true;
        } else {
            qDebug() << "[ERROR] Unable to generate valid recovery route start";
        }
    } else {
        qDebug() << "[ERROR] Car seems to be outside of valid polygon";
    }

    if (res) {
        if (direct) {
            qDebug() << "Found direct valid recovery route";
        } else {
            qDebug() <<"Found " << validRoutes << " valid recovery routes(" << discardedRoutes <<
                        " discarded, " << (genAttempts - discardedRoutes - validRoutes) << " failed). " <<
                        "Using the shortest one (" << (routeLen(recStartBest) + routeLen(rec)) << " m).";
        }
    }
    return recStartBest;

}


double RouteMagic::routeLen(QList<LocPoint> r) {
    double res = 0.0;

    for (int i = 1;i < r.size();i++) {
        double x0 = r.at(i - 1).getX();
        double y0 = r.at(i - 1).getY();
        double x1 = r.at(i).getX();
        double y1 = r.at(i).getY();
        res += sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
    }

    return res;
}

QList<LocPoint> RouteMagic::shortenRoute(QList<LocPoint> r, QList<LocPoint> outerFence, QList<QList<LocPoint>>cutouts) {
    bool debug = false;
    //long startTime = System.nanoTime();

    bool shorter = true;
    QList<LocPoint> current = r;
    int iterations = 0;

    while (shorter) {
        shorter = false;

        for (int start = 2;start < current.size() - 1;start++) {
            for (int end = current.size() - 2;end > start;end--) {
                QList<LocPoint> tmp;
                tmp.append(current.mid(0, start));
                tmp.append(current.mid(end, current.size()));

                if (isRouteDrivable(tmp,outerFence,cutouts, M_PI/6)) {
                    shorter = true;
                    current = tmp;
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

        qDebug() << "Route shortened with " << (lenStart - lenEnd) << " m " <<
                    " (iterations: "  << iterations << ", removed: " << (pointsStart - pointsEnd) << ")";
    }
    return current;
}

QList<LocPoint> RouteMagic::shortenRouteMore(QList<LocPoint> route,QList<LocPoint> outerFence, QList<QList<LocPoint>> cutouts) {
    bool debug = true;
    int shorteningPasses = 0;
    int okChecks = 0;
    int pointsStart = route.size();
    double lenStart = routeLen(route);
    //int64_t time0 = 0;
    //int64_t time1 = 0;

    QList<LocPoint> r = shortenRoute(route, outerFence, cutouts);

    double interpol = 1.5;
    double endDiff = 0.02;

    //time0 = duration_cast<microseconds>(time_point_cast<microseconds>(high_resolution_clock::now()).time_since_epoch()).count();
    r = interpolateRoute(r, interpol);
    //time1 = duration_cast<microseconds>(time_point_cast<microseconds>(high_resolution_clock::now()).time_since_epoch()).count();

    //qDebug() << "interpolation 0 time: " << (time1-time0) / 1000000 << " sec";

    bool shorter = true;
    while (shorter) {
        shorter = false;

        for (int i = 1;i < (r.size() - 1);i++) {
            QList<QList<LocPoint>> added = generateArcs(r.at(i - 1), r.at(i),outerFence,cutouts);

            for (QList<LocPoint> pList: added) {
                double rLenOld = routeLen(r);
                //time0 = duration_cast<microseconds>(time_point_cast<microseconds>(high_resolution_clock::now()).time_since_epoch()).count();
                for (int subList = 1;subList <= pList.size();subList++) {

                    for (int j = r.size() - 2;j > i - 1;j--) {
                        QList<LocPoint> tmp;
                        tmp.append(r.mid(0, i + 1));
                        int ind1 = tmp.size() - 1;
                        tmp.append(pList.mid(0, subList));
                        int ind2 = tmp.size() + 2;
                        tmp.append(r.mid(j, r.size()));

                        okChecks++;

                        if ((routeLen(tmp) + endDiff) < rLenOld && isRouteDrivable(tmp.mid(ind1, ind2),outerFence,cutouts,M_PI/6)) {
                            r = tmp;
                            r = interpolateRoute(r, interpol);
                            shorter = true;
                            shorteningPasses++;
                            break;
                        }
                    }

                    if (shorter) {
                        break;
                    }
                }
                //time1 = duration_cast<microseconds>(time_point_cast<microseconds>(high_resolution_clock::now()).time_since_epoch()).count();
                //qDebug() << "middle loop time: " << (time1-time0) / 1000000 << " sec";

            }
        }
    }

    r = shortenRoute(r, outerFence, cutouts);

    if (debug) {
        int pointsEnd = r.size();
        double lenEnd = routeLen(r);

        qDebug() << "Route shortened with " << (lenStart - lenEnd) << " m " <<
                           " (passes: " << shorteningPasses <<
                           ", okChecks: " << okChecks <<
                           ", removed: " << (pointsStart - pointsEnd) << ")";
    }
    return r;
}

// DONE: Maybe should be passed as pointers to QList as the function updates r1 in-place??
//  * So this was the problem.
bool RouteMagic::tryConnect(QList<LocPoint> *r1, QList<LocPoint> *r2, QList<LocPoint>outerFence, QList<QList<LocPoint>> cutouts) {

    if (r1->size() < 2 || r2->size() < 2) {
        qDebug() << "tryConnect: route too short";
        qDebug() << "r1->size: " << r1->size();
        qDebug() << "r2->size: " << r2->size();
        return false;
    }

    QList<QList<LocPoint>> added1 = generateArcs(r1->at(r1->size() - 2), r1->at(r1->size() - 1),outerFence,cutouts);
    QList<QList<LocPoint>> added2 = generateArcs(r2->at(1), r2->at(0),outerFence,cutouts);
    QList<LocPoint> p2Now;

    int max_added1 = qMax(added1.at(0).size(),
                         added1.at(1).size());
    int max_added2 = qMax(added2.at(0).size(),
                         added2.at(1).size());

    QList<LocPoint> end_left = *r2;
    QList<LocPoint> end_right = *r2;

    for (int i_2 = -1; i_2 < max_added2; i_2 ++) {

        if (i_2 >= 0 && added2.at(0).size() > i_2) {
            end_left.prepend(added2.at(0).at(i_2));
        }
        if (i_2 >= 0 && added2.at(1).size() > i_2) {
            end_right.prepend(added2.at(1).at(i_2));
        }

        QList<LocPoint> start_left = *r1;
        QList<LocPoint> start_right = *r1;
        int size_left = start_left.size() - 2;
        int size_right = start_left.size() - 2;

        for (int i_1 = -1; i_1 < max_added1; i_1 ++) {

            if (i_1 >= 0 && added1.at(0).size() > i_1) {
                start_left.append(added1.at(0).at(i_1));
            }
            if (i_1 >= 0 && added1.at(1).size() > i_1) {
                start_right.append(added1.at(1).at(i_1));
            }

            QList<LocPoint> tmp_left;
            tmp_left.append(start_left);
            tmp_left.append(end_left);

            if (isRouteDrivable(tmp_left.mid(size_left, -1), outerFence, cutouts, M_PI/6)) {
                r1->clear();
                r1->append(tmp_left);
                return true;
            }

            QList<LocPoint> tmp_right;
            tmp_right.append(start_right);
            tmp_right.append(end_right);

            if (isRouteDrivable(tmp_right.mid(size_right, -1), outerFence, cutouts, M_PI/6)) {
                r1->clear();
                r1->append(tmp_right);
                return true;
            }
            QList<LocPoint> tmp_left1;
            tmp_left1.append(start_left);
            tmp_left1.append(end_right);

            if (isRouteDrivable(tmp_left1.mid(size_left,-1), outerFence, cutouts, M_PI/6)) {
                r1->clear();
                r1->append(tmp_left1);
                return true;
            }

            QList<LocPoint> tmp_right1;
            tmp_right1.append(start_right);
            tmp_right1.append(end_left);

            if (isRouteDrivable(tmp_right1.mid(size_right,-1), outerFence, cutouts, M_PI/6)) {
                r1->clear();
                r1->append(tmp_right1);
                return true;
            }
        }
    }

    return false;
    /*
    for (QList<LocPoint> pList2: added2) {
        for (int subList2 = 0;subList2 <= pList2.size();subList2++) {
            p2Now.clear();
            //p2Now.append(pList2.mid(0, subList2));

            // reverse sublist into p2Now.
            for (LocPoint p : pList2.mid(0, subList2)) {
                p2Now.prepend(p);
            }

            //qDebug() << "p2Now size = " << p2Now.size();

            for (QList<LocPoint> pList1: added1) {
                for (int subList1 = 0;subList1 <= pList1.size();subList1++) {
                    QList<LocPoint> tmp;
                    tmp.append(*r1);
                    int ind1 = tmp.size() - 2;
                    tmp.append(pList1.mid(0, subList1));
                    tmp.append(p2Now);
                    tmp.append(*r2);

                    if (isRouteDrivable(tmp.mid(ind1, tmp.size()), outerFence, cutouts, M_PI/6)) {
                        r1->clear();
                        r1->append(tmp);
                        return true;
                    }
                }
            }
        }
    }

    return false;
    */
}

QList<LocPoint> RouteMagic::reverseRoute(QList<LocPoint> t)
{
    QList<LocPoint> rev;
    for (auto p : t)
        rev.prepend(p);
    return rev;
}

QList<QList<LocPoint>> RouteMagic::generateArcs(LocPoint p1, LocPoint p2,QList<LocPoint>outerFence, QList<QList<LocPoint>> cutouts) {
    double minDist = 0.7;
    double maxAng = M_PI / 6.1;

    QList<QList<LocPoint>> res;
    QList<LocPoint> added1Left;
    QList<LocPoint> added1Right;
    added1Left.append(p1);
    added1Left.append(p2);
    added1Right.append(added1Left);
    for (int j = 0;j < 11;j++) {
        LocPoint n = p1;
        int ind1 = added1Left.size() - 1;
        double a1 = atan2(added1Left.at(ind1).getY() - added1Left.at(ind1 - 1).getY(),
                          added1Left.at(ind1).getX() - added1Left.at(ind1 - 1).getX());
        n.setX(added1Left.at(ind1).getX() + minDist * cos(a1 - maxAng));
        n.setY(added1Left.at(ind1).getY() + minDist * sin(a1 - maxAng));
        if (isPointWithin(n,outerFence) &&
                isPointOutside(n, cutouts)) {
            added1Left.append(n);
        }

        n = p1;
        ind1 = added1Right.size() - 1;
        a1 = atan2(added1Right.at(ind1).getY() - added1Right.at(ind1 - 1).getY(),
                   added1Right.at(ind1).getX() - added1Right.at(ind1 - 1).getX());
        n.setX(added1Right.at(ind1).getX() + minDist * cos(a1 + maxAng));
        n.setY(added1Right.at(ind1).getY() + minDist * sin(a1 + maxAng));
        if (isPointWithin(n,outerFence) &&
                isPointOutside(n,cutouts)) {
            added1Right.append(n);
        }
    }
    res.append(added1Left.mid(2, added1Left.size()));
    res.append(added1Right.mid(2, added1Right.size()));

    return res;
}

QList<LocPoint> RouteMagic::interpolateRoute(QList<LocPoint> route, double maxSegmentLen) {
    bool added = true;
    QList<LocPoint> r = route;
    while (added) {
        added = false;
        for (int i = 1;i < r.size();i++) {
            LocPoint prev = r.at(i - 1);
            LocPoint now  = r.at(i);

            if (prev.getDistanceTo(now) > maxSegmentLen) {
                LocPoint mid = prev;
                mid.setX((prev.getX() + now.getX()) / 2.0);
                mid.setY((prev.getY() + now.getY()) / 2.0);
                mid.setSpeed((prev.getSpeed() + now.getSpeed()) / 2.0);
                mid.setTime((prev.getTime() + now.getTime()) / 2);
                r.insert(i,mid);
                added = true;
                break;
            }
        }
    }
    return r;
}


QList<LocPoint> RouteMagic::generateRouteWithin(int length,
                                                QList<LocPoint> prev_traj,
                                                double speed,
                                                QList<LocPoint> outerFence,
                                                QList<QList<LocPoint>> cutouts)
{
    int64_t timeStart = duration_cast<microseconds>(time_point_cast<microseconds>(high_resolution_clock::now()).time_since_epoch()).count();
    srand(static_cast<unsigned int>(timeStart));
    QList<LocPoint> r;
    QList<LocPoint> rLargest;
    QList<LocPoint> prev = prev_traj;

    if (outerFence.size() < 3) {
        return prev_traj; // hack
    }

    // Generation settings
    int maxAttemptsOuter = 5000;
    int maxAttemptsInner = 50;
    double minDist = 0.6;
    double maxDist = 2.0;
    double maxAng = M_PI / 6;

    int attemptOuter = 0;
    int genPoints = 0;
    int start = 0;

    while (attemptOuter < maxAttemptsOuter) {
        r.clear();
        if (prev.size() > 0) {
            r = prev;   // I'm guessing this is a copying assignement.
            start = prev.size();
        }

        //			int lastStepBack = length + start;

        for (int pNow = start;pNow < length + start;pNow++) {
            int attemptInner = 0;
            bool ok = false;
            double px = 0.0;
            double py = 0.0;

            double xMin; // = mXMin;
            double xMax; // = mXMax;
            double yMin; // = mYMin;
            double yMax; // = mYMax;

            xMin = xMax = yMin = yMax = 0; // Only if pNow == 0 will this be the values of min,max..

            if (pNow == 1) {
                double xLast = r.at(pNow - 1).getX();
                double yLast = r.at(pNow - 1).getY();

                xMax = xLast + maxDist;
                xMin = xLast - maxDist;
                yMax = yLast + maxDist;
                yMin = yLast - maxDist;
            } else if (pNow > 1) {
                double xLast1 = r.at(pNow - 1).getX();
                double yLast1 = r.at(pNow - 1).getY();
                double xLast2 = r.at(pNow - 2).getX();
                double yLast2 = r.at(pNow - 2).getY();

                double a1 = atan2(yLast1 - yLast2, xLast1 - xLast2);

                double p1x = xLast1 + maxDist * cos(a1 - maxAng);
                double p1y = yLast1 + maxDist * sin(a1 - maxAng);
                double p2x = xLast1 + maxDist * cos(a1);
                double p2y = yLast1 + maxDist * sin(a1);
                double p3x = xLast1 + maxDist * cos(a1 + maxAng);
                double p3y = yLast1 + maxDist * sin(a1 + maxAng);

                LocPoint coll;

                QList<QList<LocPoint>> allBounds = cutouts;
                allBounds.append(outerFence);

                if (closestLineIntersection(xLast1, yLast1, p1x, p1y, allBounds, &coll)) {
                    p1x = coll.getX();
                    p1y = coll.getY();
                }

                if (closestLineIntersection(xLast1, yLast1, p2x, p2y, allBounds, &coll)) {
                    p2x = coll.getX();
                    p2y = coll.getY();
                }

                if (closestLineIntersection(xLast1, yLast1, p3x, p3y, allBounds, &coll)) {
                    p3x = coll.getX();
                    p3y = coll.getY();
                }

                xMax = maxFrom4(xLast1, p1x, p2x, p3x);
                xMin = minFrom4(xLast1, p1x, p2x, p3x);
                yMax = maxFrom4(yLast1, p1y, p2y, p3y);
                yMin = minFrom4(yLast1, p1y, p2y, p3y);


                // hack
                LocPoint max_p = LocPoint(xMax,yMax);
                LocPoint min_p = LocPoint(xMin,yMin);

                if (max_p.getDistanceTo(min_p) < minDist) break;
            }
/*
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
*/
            while (attemptInner < maxAttemptsInner) {

                px = randInRange(xMin, xMax);
                py = randInRange(yMin, yMax);

                attemptInner++;
                genPoints++;

                ok = true;

                if (pNow == 0) {
                    if (!isPointWithin(px, py,outerFence) ||
                            !isPointOutside(px, py,cutouts)) {
                        ok = false;
                        continue;
                    }
                } else {
                    LocPoint p1;
                    p1.setXY(r.at(pNow - 1).getX(),r.at(pNow - 1).getY());
                    LocPoint p2;
                    p2.setX(px);
                    p2.setY(py);

                    if (!isSegmentWithin(p1, p2, outerFence) ||
                            !isSegmentOutside(p1, p2, cutouts)) {
                        ok = false;
                        continue;
                    }

                    if (p1.getDistanceTo(p2) < minDist) {
                        ok = false;
                        continue;
                    }
                    //if (Utils.pointDistance(p1, p2) < minDist) {
                    //    ok = false;
                    //    continue;
                    //}

                    if (pNow > 1) {
                        double px1 = r.at(pNow - 2).getX();
                        double py1 = r.at(pNow - 2).getY();
                        double px2 = r.at(pNow - 1).getX();
                        double py2 = r.at(pNow - 1).getY();
                        double qx1 = r.at(pNow - 1).getX();
                        double qy1 = r.at(pNow - 1).getY();
                        double qx2 = px;
                        double qy2 = py;

                        if (fabs(angleBetweenLines(px1, py1, px2, py2,
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

            LocPoint p;
            p.setX(px);
            p.setY(py);
            p.setSpeed(speed);
            p.setTime(0);

            r.append(p);
        }

        attemptOuter++;

        if (r.size() > rLargest.size()) {
            rLargest.clear();
            rLargest = r; // again assuming copying assignment
        }

        if (rLargest.size() == (length + start)) {
            break;
        }
    }

    //mLastOuterAttempts = attemptOuter;
    //mLastGeneratedPoints = genPoints;

    if (false) {
        //auto timestamp = time_point_cast<std::chrono::microseconds>(system_clock::now());
        //auto ts_ms = duration_cast<microseconds>(timestamp.time_since_epoch());
        //uint64_t ts_u = ts_ms.count();
        int64_t timestamp = duration_cast<microseconds>(time_point_cast<microseconds>(high_resolution_clock::now()).time_since_epoch()).count();
        int64_t t_diff = timestamp - timeStart;

        qDebug() << "Generated points: " << genPoints <<
                    ", Outer loops: " << attemptOuter <<
                    ", Time: " << (t_diff) << " uS";
    }

    //res = rLargest;
    //prev_traj.traj_pts.append(QVector<chronos_traj_pt>::fromList(rLargest));
            //.append(rLargest);
    return rLargest; //res; //prev_traj;
}

QList<LocPoint> RouteMagic::fillBoundsWithTrajectory(QList<LocPoint> bounds, QList<LocPoint> entry, QList<LocPoint> exit, double spacing, double angle, bool reduce)
{
    double xMin = 200000;
    double xMax = -200000;
    double yMin = 200000;
    double yMax = -200000;

    for (auto p : bounds) {
        if (p.getX() > xMax) xMax = p.getX();
        if (p.getX() < xMin) xMin = p.getX();
        if (p.getY() > yMax) yMax = p.getY();
        if (p.getY() < yMin) yMin = p.getY();
    }

    double distMax = 0;
    for (auto p1 : bounds) {
        for (auto p2 : bounds) {
            double d = p1.getDistanceTo(p2);
            if (d > distMax) distMax = d;
        }
    }

    double xCenter = xMin + (xMax - xMin) / 2;
    double yCenter = yMin + (yMax - yMin) / 2;

    double xNOrigoTranslate = xCenter;
    double yNOrigoTranslate = yCenter;

    double xCurr = spacing;
    double yCurr = 0;

    QList<QList<LocPoint>> grid;

    double dHalf = distMax / 2;

    for (xCurr = -dHalf; xCurr < dHalf; xCurr += spacing) {
        QList<LocPoint> row;
        for (yCurr = -dHalf; yCurr < dHalf; yCurr += spacing) {
            row.append(LocPoint(xCurr,yCurr));
        }
        grid.append(row);
    }

    // Rotate and translate points into the boundary
    for (auto &row : grid) {
        for (auto &p : row) {
            double x = p.getX();
            double y = p.getY();

            double x2 = (x)*cos(angle)-(y)*sin(angle);
            double y2 = (x)*sin(angle)+(y)*cos(angle);

            p.setX(x2 += xNOrigoTranslate);
            p.setY(y2 += yNOrigoTranslate);
        }
    }

    // clip away unwanted points
    for (auto &row : grid ) {
        QMutableListIterator<LocPoint> it(row);
        while(it.hasNext()) {
            LocPoint p = it.next();
            if (!isPointWithin(p,bounds)) {
                it.remove();
                continue;
            }
            LocPoint b = bounds.last();
            for (auto b0 : bounds) {
                double d = distanceToLine(p,b,b0);
                if ( d < spacing) {
                    it.remove();
                    break;
                }
                b = b0;
            }
        }
    }

    // Remove entirely empty row-trajectories
    QMutableListIterator<QList<LocPoint>> gridIt(grid);
    while (gridIt.hasNext()) {
        QList<LocPoint> row = gridIt.next();
        if (row.size() <= 1)
            gridIt.remove();
    }

    // Reduce trajectory density
    if (reduce) {

        for (auto &row : grid ) {
            if (row.size() >= 2) {
                QList<LocPoint> tmp = row;
                row.clear();
                row.append(tmp.first());
                row.append(tmp.last());
            }
        }
    }

    QList<LocPoint> route;

    route = grid.first();
    if (grid.size() <= 1) return route;

    bool rev = true;
    for (int i = 1; i < grid.size(); i ++) {
        QList<LocPoint> row = rev ? reverseRoute(grid.at(i)) : grid.at(i);

        if (!tryConnect(&route, &row,bounds,QList<QList<LocPoint>>())) {
            qDebug() << "route->size: " << route.size();
            qDebug() << "row->size:   " << row.size();
            qDebug() << "Failed to connect trajectory";
            route.append(row);
            return route;
        }
        //route.append(row);
        rev = !rev;
    }

    return route;
}

QList<LocPoint> RouteMagic::fillConvexPolygonWithZigZag(QList<LocPoint> bounds, double spacing, double speed, double speedInTurns, int turnIntermediateSteps, uint32_t setAttributesOnStraights, double pointWithAttributesDistanceToTurn)
{
    QList<LocPoint> route;

    // 1. resize bounds to ensure spacing
    bounds = getShrinkedConvexPolygon(bounds, spacing);

    // 2. get bound that determines optimal zigzag direction
    QPair<LocPoint, LocPoint> baseline = getBaselineDeterminingMinHeightOfConvexPolygon(bounds);
    double angle = atan2(baseline.second.getY() - baseline.first.getY(), baseline.second.getX() - baseline.first.getX());

    // 3. draw parallels to baseline that cover whole polygon
    int polygonDirectionSign = getConvexPolygonOrientation(bounds);

    LocPoint newLineBegin(baseline.first.getX() - 100000*cos(angle) + 0.01*polygonDirectionSign*cos(angle + PI/2),
                          baseline.first.getY() - 100000*sin(angle) + 0.01*polygonDirectionSign*sin(angle + PI/2));
    LocPoint newLineEnd(baseline.second.getX() + 100000*cos(angle) + 0.01*polygonDirectionSign*cos(angle + PI/2),
                        baseline.second.getY() + 100000*sin(angle) + 0.01*polygonDirectionSign*sin(angle + PI/2));

    while (intersectionExists(QList<LocPoint>({newLineBegin, newLineEnd}), bounds)) {
        route.append(newLineBegin);
        route.append(newLineEnd);

        newLineBegin = LocPoint(newLineBegin.getX() + polygonDirectionSign*spacing*cos(angle + PI/2), newLineBegin.getY() + polygonDirectionSign*spacing*sin(angle + PI/2));
        newLineEnd =   LocPoint(newLineEnd.getX() + polygonDirectionSign*spacing*cos(angle + PI/2), newLineEnd.getY() + polygonDirectionSign*spacing*sin(angle + PI/2));
    }

    // 4. cut parallels down to polygon ((i) ensure "short" connections not in bounds, (ii) sort bounds to get reproducible results, (iii) get intersections with bounds)
    for (int i=1; i<route.size(); i+=4)
        std::swap(route[i-1], route[i]);

    int baselineStartInBounds = -1;
    for (int i = 0; i < bounds.size() && baselineStartInBounds == -1; i++)
        if (bounds.at(i).getX() == baseline.first.getX() && bounds.at(i).getY() == baseline.first.getY())
            baselineStartInBounds = i;
    assert(baselineStartInBounds > -1);

    QList<LocPoint> boundsStartingWithBaseline;
    for (int i = baselineStartInBounds; i < bounds.size(); i++)
        boundsStartingWithBaseline.append(bounds.at(i));
    for (int i = 0; i < baselineStartInBounds; i++)
        boundsStartingWithBaseline.append(bounds.at(i));
    bounds = boundsStartingWithBaseline;
    bounds.append(bounds.first());

    route = getAllIntersections(route, bounds);

    // 5. adjust orientations, set speed, smoothen turns
    for (int i = 1; i < route.size(); i+=4)
        std::swap(route[i-1], route[i]);
    for (auto& pt : route)
        pt.setSpeed(speed);

    if (turnIntermediateSteps > 0) {
        for (int i = 1; i < route.size()-1; i+=(2+turnIntermediateSteps)) {
            int turnDirectionSign = (i/(2+turnIntermediateSteps))%2 == 0 ? 1 : -1;

            // Find turnCenter that guarantees to stay within bounds
            QPair<LocPoint, LocPoint> turnBound(LocPoint(route.at(i).getX(), route.at(i).getY()), LocPoint(route.at(i+1).getX(), route.at(i+1).getY()));
            double turnBoundAngle = atan2(turnBound.second.getY() - turnBound.first.getY(), turnBound.second.getX() - turnBound.first.getX());

            QPair<LocPoint, LocPoint> turnBoundShifted(LocPoint(route.at(i).getX() - (spacing/2)*turnDirectionSign*cos(turnBoundAngle + PI/2),
                                                                route.at(i).getY() - (spacing/2)*turnDirectionSign*sin(turnBoundAngle + PI/2)),
                                                       LocPoint(route.at(i+1).getX() - (spacing/2)*turnDirectionSign*cos(turnBoundAngle + PI/2),
                                                                route.at(i+1).getY() - (spacing/2)*turnDirectionSign*sin(turnBoundAngle + PI/2)));

            QPair<LocPoint, LocPoint> turnCenterLine(LocPoint(route.at(i).getX() + (spacing/2)*polygonDirectionSign*cos(angle + PI/2),
                                                              route.at(i).getY() + (spacing/2)*polygonDirectionSign*sin(angle + PI/2)),
                                                     LocPoint(route.at(i-1).getX() + (spacing/2)*polygonDirectionSign*cos(angle + PI/2),
                                                              route.at(i-1).getY() + (spacing/2)*polygonDirectionSign*sin(angle + PI/2)));

            LocPoint turnCenter = getLineIntersection(turnBoundShifted, turnCenterLine);

            // Draw half circle
            const double piStep = PI/(turnIntermediateSteps+1) * turnDirectionSign * polygonDirectionSign;
            for (int j = 0; j <= turnIntermediateSteps+1; j++) {
                LocPoint turnStep(turnCenter.getX() + (spacing/2)*cos(j*piStep + angle - PI/2 * polygonDirectionSign),
                                  turnCenter.getY() + (spacing/2)*sin(j*piStep + angle - PI/2 * polygonDirectionSign));

                if (j == 0 || j == turnIntermediateSteps+1) {
                    turnStep.setSpeed(speed);
                    route.replace(i+j,turnStep);
                } else {
                    turnStep.setSpeed(speedInTurns);
                    route.insert(i+j,turnStep);
                }
            }
        }
    }

    // 6. introduce points for predictable processing of attributes, set attributes
    if (setAttributesOnStraights) {
        for (int i = 1; i < route.size(); i+=(2+turnIntermediateSteps+2)) {
            int turnDirectionSign = (i/(2+turnIntermediateSteps+2))%2 == 0 ? 1 : -1;

            LocPoint attrStart(route.at(i-1).getX() + pointWithAttributesDistanceToTurn*turnDirectionSign*cos(angle),
                               route.at(i-1).getY() + pointWithAttributesDistanceToTurn*turnDirectionSign*sin(angle));
            LocPoint attrEnd(route.at(i).getX() - pointWithAttributesDistanceToTurn*turnDirectionSign*cos(angle),
                             route.at(i).getY() - pointWithAttributesDistanceToTurn*turnDirectionSign*sin(angle));

            uint32_t tmpAttr = attrStart.getAttributes();
            tmpAttr |= setAttributesOnStraights;
            attrStart.setAttributes(tmpAttr);

            tmpAttr = attrEnd.getAttributes();
            tmpAttr |= setAttributesOnStraights;
            attrEnd.setAttributes(tmpAttr);

            attrStart.setSpeed(speed);
            attrEnd.setSpeed(speed);

            route.insert(i,attrEnd);
            route.insert(i,attrStart);
        }
    }


    return route;
}

QList<LocPoint> RouteMagic::fillConvexPolygonWithFramedZigZag(QList<LocPoint> bounds, double spacing, double speed, double speedInTurns, int turnIntermediateSteps, uint32_t setAttributesOnStraights, double pointWithAttributesDistanceToTurn)
{
    QList<LocPoint> frame = getShrinkedConvexPolygon(bounds, spacing);

    QList<LocPoint> zigzag = fillConvexPolygonWithZigZag(frame, spacing, speed, speedInTurns, turnIntermediateSteps, setAttributesOnStraights, pointWithAttributesDistanceToTurn);

    int pointIdx = getClosestPointInRoute(zigzag.first(), frame);

    QList<LocPoint> route;
    for (int i = pointIdx; i < frame.size(); i++)
        route.append(frame.at(i));
    for (int i = 0; i < pointIdx; i++)
        route.append(frame.at(i));
    route.append(frame.at(pointIdx));

    for (auto& pt : route)
        pt.setSpeed(speed);
    route.append(zigzag);

    return route;
}

QList<LocPoint> RouteMagic::getShrinkedConvexPolygon(QList<LocPoint> bounds, double spacing)
{
    bounds.append(bounds.at(0)); // make sure bounds are closed
    int directionSign = getConvexPolygonOrientation(bounds);
    QList<LocPoint> shrinkedBounds;
    for (int i=1; i<bounds.size(); i++) {
        double angle = atan2(bounds.at(i).getY() - bounds.at(i-1).getY(), bounds.at(i).getX() - bounds.at(i-1).getX());
        LocPoint lineStart = LocPoint(bounds.at(i-1).getX() + directionSign*spacing*cos(angle + PI/2), bounds.at(i-1).getY() + directionSign*spacing*sin(angle + PI/2));
        LocPoint lineEnd = LocPoint(bounds.at(i).getX() + directionSign*spacing*cos(angle + PI/2), bounds.at(i).getY() + directionSign*spacing*sin(angle + PI/2));
        shrinkedBounds.append(lineStart);
        shrinkedBounds.append(lineEnd);
    }

    shrinkedBounds = getAllIntersections(shrinkedBounds);
    if (shrinkedBounds.size() > 1)
       std::swap(shrinkedBounds[0], shrinkedBounds[1]);
    shrinkedBounds.append(shrinkedBounds[0]);

    return shrinkedBounds;
}

int RouteMagic::getConvexPolygonOrientation(QList<LocPoint> bounds)
{
    if (bounds.size() < 3)
        return 0;

    return ((bounds[1].getX()*bounds[2].getY() + bounds[0].getX()*bounds[1].getY() + bounds[0].getY()*bounds[2].getX())
            -(bounds[0].getY()*bounds[1].getX() + bounds[1].getY()*bounds[2].getX() + bounds[0].getX()*bounds[2].getY())) > 0? 1 : -1;

}

void RouteMagic::saveRoutes(bool withId, QList<QList<LocPoint> > routes)
{
    QString filename = QFileDialog::getSaveFileName(nullptr,
                                                    tr("Save Routes"), "",
                                                    tr("Xml files (*.xml)"));

    // Cancel pressed
    if (filename.isEmpty()) {
        return;
    }

    if (!filename.toLower().endsWith(".xml")) {
        filename.append(".xml");
    }

    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        QMessageBox::critical(nullptr, "Save Routes",
                              "Could not open\n" + filename + "\nfor writing");
        qDebug() << "Could not save routes";
        return;
    }


    QXmlStreamWriter stream(&file);
    stream.setCodec("UTF-8");
    stream.setAutoFormatting(true);
    stream.writeStartDocument();

    stream.writeStartElement("routes");

    for (int i = 0;i < routes.size();i++) {
        if (!routes.at(i).isEmpty()) {
            stream.writeStartElement("route");

            if (withId) {
                stream.writeTextElement("id", QString::number(i));
            }

            for (const LocPoint &p: routes.at(i)) {
                stream.writeStartElement("point");
                stream.writeTextElement("x", QString::number(p.getX()));
                stream.writeTextElement("y", QString::number(p.getY()));
                stream.writeTextElement("speed", QString::number(p.getSpeed()));
                stream.writeTextElement("time", QString::number(p.getTime()));
                //stream.writeTextElement("attributes", QString::number(p.getAttributes()));
                stream.writeEndElement();
            }
            stream.writeEndElement();
        }
    }

    stream.writeEndDocument();
    file.close();
}

int RouteMagic::loadRoutes(QString filename, MapWidget *map) {
    int res = 0;

    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) {
        res = -1;
        return res;
    }

    QXmlStreamReader stream(&file);

    // Look for routes tag
    bool routes_found = false;
    while (stream.readNextStartElement()) {
        if (stream.name() == "routes") {
            routes_found = true;
            break;
        }
    }

    if (routes_found) {
        QList<QPair<int, QList<LocPoint> > > routes;
        QList<LocPoint> anchors;

        while (stream.readNextStartElement()) {
            QString name = stream.name().toString();

            if (name == "route") {
                int id = -1;
                QList<LocPoint> route;

                while (stream.readNextStartElement()) {
                    QString name2 = stream.name().toString();

                    if (name2 == "id") {
                        id = stream.readElementText().toInt();
                    } else if (name2 == "point") {
                        LocPoint p;

                        while (stream.readNextStartElement()) {
                            QString name3 = stream.name().toString();

                            if (name3 == "x") {
                                p.setX(stream.readElementText().toDouble());
                            } else if (name3 == "y") {
                                p.setY(stream.readElementText().toDouble());
                            } else if (name3 == "speed") {
                                p.setSpeed(stream.readElementText().toDouble());
                            } else if (name3 == "time") {
                                p.setTime(stream.readElementText().toInt());
                            } else if (name3 == "attributes") {
                                p.setAttributes(stream.readElementText().toInt());
                            } else {
                                qWarning() << ": Unknown XML element :" << name2;
                                stream.skipCurrentElement();
                            }
                        }

                        route.append(p);
                    } else {
                        qWarning() << ": Unknown XML element :" << name2;
                        stream.skipCurrentElement();
                    }

                    if (stream.hasError()) {
                        qWarning() << " : XML ERROR :" << stream.errorString();
                    }
                }

                routes.append(QPair<int, QList<LocPoint> >(id, route));
            } else if (name == "anchors") {
                while (stream.readNextStartElement()) {
                    QString name2 = stream.name().toString();

                    if (name2 == "anchor") {
                        LocPoint p;

                        while (stream.readNextStartElement()) {
                            QString name3 = stream.name().toString();

                            if (name3 == "x") {
                                p.setX(stream.readElementText().toDouble());
                            } else if (name3 == "y") {
                                p.setY(stream.readElementText().toDouble());
                            } else if (name3 == "height") {
                                p.setHeight(stream.readElementText().toDouble());
                            } else if (name3 == "id") {
                                p.setId(stream.readElementText().toInt());
                            } else {
                                qWarning() << ": Unknown XML element :" << name2;
                                stream.skipCurrentElement();
                            }
                        }

                        anchors.append(p);
                    } else {
                        qWarning() << ": Unknown XML element :" << name2;
                        stream.skipCurrentElement();
                    }

                    if (stream.hasError()) {
                        qWarning() << " : XML ERROR :" << stream.errorString();
                    }
                }
            }

            if (stream.hasError()) {
                qWarning() << "XML ERROR :" << stream.errorString();
                qWarning() << stream.lineNumber() << stream.columnNumber();
            }
        }

        for (QPair<int, QList<LocPoint> > r: routes) {
            if (r.first >= 0) {
                int routeLast = map->getRouteNow();
                map->setRouteNow(r.first);
                map->setRoute(r.second);
                map->setRouteNow(routeLast);
            } else {
                map->addRoute(r.second);
            }
        }

        for (LocPoint p: anchors) {
            map->addAnchor(p);
        }

        file.close();
    } else {
        res = -2;
    }

    return res;
}
