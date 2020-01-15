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

#include <chrono>
using namespace std::chrono;
#include <QDebug>

#include <QFileDialog>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include <QMessageBox>
#include <mapwidget.h>
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
    QList<QList<LocPoint>> added1 = generateArcs(r1->at(r1->size() - 2), r1->at(r1->size() - 1),outerFence,cutouts);
    QList<QList<LocPoint>> added2 = generateArcs(r2->at(1), r2->at(0),outerFence,cutouts);
    QList<LocPoint> p2Now;

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
                    tmp.append(r2->mid(0, 2));

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
