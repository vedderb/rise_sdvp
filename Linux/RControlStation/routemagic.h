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
#ifndef ROUTEMAGIC_H
#define ROUTEMAGIC_H

#include <QObject>

#include "locpoint.h"
#include "mapwidget.h" // not sure i want this dependency here..

class RouteMagic : public QObject
{
    Q_OBJECT
public:
    explicit RouteMagic(QObject *parent = nullptr);

    static constexpr double PI = 3.14159265;

    static double maxFrom4(double a, double b, double c, double d);
    static double minFrom4(double a, double b, double c, double d);
    static double randInRange(double min, double max);

    static bool isPointWithin(double px, double py, QList<LocPoint> route);
    static bool isPointWithin(LocPoint p,QList<LocPoint> route);
    static bool isPointOutside(double px, double py, QList <LocPoint> route);
    static bool isPointOutside(LocPoint p, QList<LocPoint> route);
    static bool isPointOutside(double px, double py, QList<QList<LocPoint>> cutouts);
    static bool isPointOutside(LocPoint p, QList<QList<LocPoint> > cutouts);
    static bool isSegmentWithin(LocPoint p1, LocPoint p2, QList<LocPoint> route);
    static bool isSegmentOutside(LocPoint p1, LocPoint p2, QList<LocPoint> route);
    static bool isSegmentOutside(LocPoint p1, LocPoint p2, QList<QList<LocPoint> > cutouts);

    static bool ccw(LocPoint a, LocPoint b, LocPoint c);
    static bool lineIntersect(LocPoint a, LocPoint b, LocPoint c, LocPoint d);
    static bool boxesIntersect(QList<LocPoint> b0, QList<LocPoint> b1);
    static bool intersectionExists(QList<LocPoint> points0, QList<LocPoint> points1);
    static QList<LocPoint> getAllIntersections(QList<LocPoint> points0, QList<LocPoint> points1);    
    static QList<LocPoint> getAllIntersections(QList<LocPoint> route);
    static bool circlesOverlap(LocPoint p0, double r0, LocPoint p1, double r1);
    static bool lineIntersectsRouteTime(LocPoint p0, LocPoint p1, QList<LocPoint> route);
    static bool routeIntersectsRouteInTime(QList<LocPoint> t0, QList<LocPoint> t1);
    static double angleBetweenLines(double px1, double py1, double px2, double py2, double qx1, double qy1, double qx2, double qy2);
    static double angleBetweenLines(LocPoint p0a, LocPoint p0b, LocPoint p1a, LocPoint p1b);
    static bool getLineIntersection(double p0_x, double p0_y, double p1_x, double p1_y, double p2_x, double p2_y, double p3_x, double p3_y, LocPoint *coll);
    static bool getLineIntersection(LocPoint p0, LocPoint p1, LocPoint p2, LocPoint p3, LocPoint *coll);
    static LocPoint getLineIntersection(QPair<LocPoint, LocPoint> line0, QPair<LocPoint, LocPoint> line1); // needs cleanup (all LineIntersection functions), partially dublicates
    static bool closestLineIntersection(double p0_x, double p0_y, double p1_x, double p1_y, QList<QList<LocPoint> > routes, LocPoint *coll);
    static double distanceToLine(LocPoint p, LocPoint l0, LocPoint l1);
    static QPair<LocPoint, LocPoint> getBaselineDeterminingMinHeightOfConvexPolygon(QList<LocPoint> convexPolygon);
    static int getClosestPointInRoute(LocPoint referencePoint, QList<LocPoint> route);
    static bool isRouteDrivable(QList<LocPoint> r, QList<LocPoint> outerFence, QList<QList<LocPoint> > cutouts, double maxAng);
    static QList<LocPoint> generateRecoveryRoute(QList<LocPoint> endSegment, QList<LocPoint> recoverTo, int aheadMargin, int genAttempts, bool tryShorten, QList<LocPoint> outerFence, QList<QList<LocPoint> > cutouts);
    static bool tryConnect(QList<LocPoint> *r1, QList<LocPoint> *r2, QList<LocPoint> outerFence, QList<QList<LocPoint> > cutouts);
    static QList<LocPoint> reverseRoute(QList<LocPoint> t);
    static double routeLen(QList<LocPoint> r);
    static QList<QList<LocPoint> > generateArcs(LocPoint p1, LocPoint p2, QList<LocPoint> outerFence, QList<QList<LocPoint> > cutouts);
    static QList<LocPoint> shortenRouteMore(QList<LocPoint> route, QList<LocPoint> outerFence, QList<QList<LocPoint> > cutouts);
    static QList<LocPoint> shortenRoute(QList<LocPoint> r, QList<LocPoint> outerFence, QList<QList<LocPoint> > cutouts);
    static QList<LocPoint> interpolateRoute(QList<LocPoint> route, double maxSegmentLen);
    static QList<LocPoint> generateRouteWithin(int length, QList<LocPoint> prev_traj, double speed, int aheadmargin, QList<LocPoint> outerFence, QList<QList<LocPoint> > cutouts);
    static QList<LocPoint> generateRouteWithin(int length, QList<LocPoint> prev_traj, double speed, QList<LocPoint> outerFence, QList<QList<LocPoint> > cutouts);

    static QList<LocPoint> fillBoundsWithTrajectory(QList<LocPoint> bounds, QList<LocPoint> entry, QList<LocPoint> exit, double spacing, double angle, bool reduce);
    static QList<LocPoint> fillConvexPolygonWithZigZag(QList<LocPoint> bounds, double spacing, bool keepTurnsInBounds = false, double speed = 3.0/3.6, double speedInTurns = 2.0/3.6, int turnIntermediateSteps = 0, int visitEveryX = 0,
                                                       uint32_t setAttributesOnStraights = 0, uint32_t setAttributesInTurns = 0, double attributeDistanceAfterTurn = 1.0, double attributeDistanceBeforeTurn = 1.0);
    static QList<LocPoint> fillConvexPolygonWithFramedZigZag(QList<LocPoint> bounds, double spacing, bool keepTurnsInBounds = false, double speed = 3.0/3.6, double speedInTurns = 2.0/3.6, int turnIntermediateSteps = 0, int visitEveryX = 0,
                                                             uint32_t setAttributesOnStraights = 0, uint32_t setAttributesInTurns = 0, double attributeDistanceAfterTurn = 1.0, double attributeDistanceBeforeTurn = 1.0);
    static QList<LocPoint> getShrinkedConvexPolygon(QList<LocPoint> bounds, double spacing);
    static int getConvexPolygonOrientation(QList<LocPoint> bounds);

    static void saveRoutes(bool, QList<QList<LocPoint> > routes);
    static int  loadRoutes(QString filename, MapWidget *map);

signals:

public slots:
};

#endif // ROUTEMAGIC_H
