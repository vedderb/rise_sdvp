/*
    Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#include <QDebug>
#include <math.h>
#include <qmath.h>

#include "mapwidget.h"

namespace
{
static void normalizeAngleRad(double &angle)
{
    angle = fmod(angle, 2.0 * M_PI);

    if (angle < 0.0) {
        angle += 2.0 * M_PI;
    }
}
}

MapWidget::MapWidget(QWidget *parent) :
    MapWidgetType(parent)
{
    mScaleFactor = 0.3;
    mRotation = 0;
    mXOffset = 0;
    mYOffset = 0;
    mMouseLastX = 1000000;
    mMouseLastY = 1000000;
    mFollowCar = -1;
    mTraceCar = -1;
    mSelectedCar = -1;
    xRealPos = 0;
    yRealPos = 0;
    mCarInfo.clear();
    mCarTrace.clear();
    mCarTraceGps.clear();
    mPaintTimer = new QTimer(this);
    mPaintTimer->setSingleShot(true);
    mRoutePointSpeed = 1.0;
    mAntialias = false;

    mHasRouteClosest = false;
    mRouteCurrentPoint = 0;

    connect(mPaintTimer, SIGNAL(timeout()), this, SLOT(paintTimerSlot()));

    setMouseTracking(true);
}

CarInfo *MapWidget::getCarInfo(int car)
{
    for (int i = 0;i < mCarInfo.size();i++) {
        if (mCarInfo[i].getId() == car) {
            return &mCarInfo[i];
        }
    }

    return 0;
}

void MapWidget::addCar(CarInfo car)
{
    mCarInfo.append(car);
    repaintAfterEvents();
}

bool MapWidget::removeCar(int carId)
{
    QMutableListIterator<CarInfo> i(mCarInfo);
    while (i.hasNext()) {
        if (i.next().getId() == carId) {
            i.remove();
            return true;
        }
    }

    return false;
}

void MapWidget::setScaleFactor(double scale)
{
    double scaleDiff = scale / mScaleFactor;
    mScaleFactor = scale;
    mXOffset *= scaleDiff;
    mYOffset *= scaleDiff;
    repaintAfterEvents();
}

void MapWidget::setRotation(double rotation)
{
    mRotation = rotation;
    repaintAfterEvents();
}

void MapWidget::setXOffset(double offset)
{
    mXOffset = offset;
    repaintAfterEvents();
}

void MapWidget::setYOffset(double offset)
{
    mYOffset = offset;
    repaintAfterEvents();
}

void MapWidget::clearTrace()
{
    mCarTrace.clear();
    mCarTraceGps.clear();
    repaintAfterEvents();
}

void MapWidget::addRoutePoint(double px, double py, double speed)
{
    LocPoint pos;
    pos.setXY(px, py);
    pos.setSpeed(speed);
    mRoute.append(pos);
    update();
}

QList<LocPoint> MapWidget::getRoute()
{
    return mRoute;
}

void MapWidget::setRoute(QList<LocPoint> route)
{
    mRoute = route;
    update();
}

void MapWidget::clearRoute()
{
    mRoute.clear();
    repaintAfterEvents();
}

void MapWidget::setRoutePointSpeed(double speed)
{
    mRoutePointSpeed = speed;
}

void MapWidget::addPerspectivePixmap(PerspectivePixmap map)
{
    mPerspectivePixmaps.append(map);
}

void MapWidget::clearPerspectivePixmaps()
{
    mPerspectivePixmaps.clear();
    repaintAfterEvents();
}

QPoint MapWidget::getMousePosRelative()
{
    QPoint p = this->mapFromGlobal(QCursor::pos());
    p.setX((p.x() - mXOffset - width() / 2) / mScaleFactor);
    p.setY((-p.y() - mYOffset + height() / 2) / mScaleFactor);
    return p;
}

void MapWidget::repaintAfterEvents()
{
    if (!mPaintTimer->isActive()) {
        mPaintTimer->start();
    }
}

void MapWidget::setAntialiasing(bool antialias)
{
    mAntialias = antialias;
    repaintAfterEvents();
}

void MapWidget::paintTimerSlot()
{
    update();
}

void MapWidget::setFollowCar(int car)
{
    int oldCar = mFollowCar;
    mFollowCar = car;

    if (oldCar != mFollowCar) {
        repaintAfterEvents();
    }
}

void MapWidget::setTraceCar(int car)
{
    mTraceCar = car;
}

void MapWidget::setSelectedCar(int car)
{
    mSelectedCar = car;
}

void MapWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);

    if (mAntialias) {
        painter.setRenderHint(QPainter::Antialiasing);
    }

    const double scaleMax = 20;
    const double scaleMin = 0.00001;

    // Make sure scale and offsetappend is reasonable
    if (mScaleFactor < scaleMin) {
        double scaleDiff = scaleMin / mScaleFactor;
        mScaleFactor = scaleMin;
        mXOffset *= scaleDiff;
        mYOffset *= scaleDiff;
    } else if (mScaleFactor > scaleMax) {
        double scaleDiff = scaleMax / mScaleFactor;
        mScaleFactor = scaleMax;
        mXOffset *= scaleDiff;
        mYOffset *= scaleDiff;
    }

    // Optionally follow a car
    if (mFollowCar >= 0) {
        for (int i = 0;i < mCarInfo.size();i++) {
            CarInfo &carInfo = mCarInfo[i];
            if (carInfo.getId() == mFollowCar) {
                LocPoint followLoc = carInfo.getLocation();
                mXOffset = -followLoc.getX() * 1000.0 * mScaleFactor;
                mYOffset = -followLoc.getY() * 1000.0 * mScaleFactor;
            }

        }
    }

    // Limit the offset to avoid overflow at 2^31 mm
    double lim = 2000000000.0 * mScaleFactor;
    if (mXOffset > lim) {
        mXOffset = lim;
    } else if (mXOffset < -lim) {
        mXOffset = -lim;
    }

    if (mYOffset > lim) {
        mYOffset = lim;
    } else if (mYOffset < -lim) {
        mYOffset = -lim;
    }

    // Paint begins here
    painter.fillRect(event->rect(), QBrush(Qt::transparent));

    const double car_w = 800;
    const double car_h = 335;
    const double car_corner = 20;
    double angle, x, y;
    QString txt;
    QPointF pt_txt;
    QRectF rect_txt;
    QPen pen;
    QFont font = this->font();

    // Map coordinate transforms
    QTransform drawTrans;
    drawTrans.translate(event->rect().width() / 2 + mXOffset, event->rect().height() / 2 - mYOffset);
    drawTrans.scale(mScaleFactor, -mScaleFactor);
    drawTrans.rotate(mRotation);

    // Text coordinates
    QTransform txtTrans;
    txtTrans.translate(0, 0);
    txtTrans.scale( 1, 1 );
    txtTrans.rotate(0);

    // Set font
    font.setPointSize(10);
    painter.setFont(font);

    // Axis parameters
    const double scaleMult = 0.1 * ceil((1.0 / ((mScaleFactor * 10.0) / 50.0)));
    const double step = 100 * scaleMult;
    const double zeroAxisWidth = 3;
    const QColor zeroAxisColor = Qt::red;
    const QColor firstAxisColor = Qt::gray;
    const QColor secondAxisColor = Qt::blue;
    const QColor textColor = QPalette::Foreground;
    const double xStart = -ceil(width() / step / mScaleFactor) * step - ceil(mXOffset / step / mScaleFactor) * step;
    const double xEnd = ceil(width() / step / mScaleFactor) * step - floor(mXOffset / step / mScaleFactor) * step;
    const double yStart = -ceil(height() / step / mScaleFactor) * step - ceil(mYOffset / step / mScaleFactor) * step;
    const double yEnd = ceil(height() / step / mScaleFactor) * step - floor(mYOffset / step / mScaleFactor) * step;

    // Draw perspective pixmaps first
    painter.setTransform(drawTrans);
    for(int i = 0;i < mPerspectivePixmaps.size();i++) {
        mPerspectivePixmaps[i].drawUsingPainter(painter);
    }

    painter.setTransform(txtTrans);

    // Draw Y-axis segments
    for (double i = xStart;i < xEnd;i += step) {
        if (fabs(i) < 1e-3) {
            i = 0.0;
        }

        if ((int)(i / step) % 2) {
            pen.setWidth(0);
            pen.setColor(firstAxisColor);
            painter.setPen(pen);
        } else {
            txt.sprintf("%.3f", i / 1000.0);
            pt_txt.setX(i);
            pt_txt.setY(0);
            pt_txt = drawTrans.map(pt_txt);
            pt_txt.setX(pt_txt.x() + 5);
            pt_txt.setY(height() - 10);
            painter.setPen(QPen(textColor));
            painter.drawText(pt_txt, txt);

            if (fabs(i) < 1e-3) {
                pen.setWidthF(zeroAxisWidth);
                pen.setColor(zeroAxisColor);
            } else {
                pen.setWidth(0);
                pen.setColor(secondAxisColor);
            }
            painter.setPen(pen);
        }

        QPointF pt_start(i, yStart);
        QPointF pt_end(i, yEnd);
        pt_start = drawTrans.map(pt_start);
        pt_end = drawTrans.map(pt_end);
        painter.drawLine(pt_start, pt_end);
    }

    // Draw X-axis segments
    for (double i = yStart;i < yEnd;i += step) {
        if (fabs(i) < 1e-3) {
            i = 0.0;
        }

        if ((int)(i / step) % 2) {
            pen.setWidth(0);
            pen.setColor(firstAxisColor);
            painter.setPen(pen);
        } else {
            txt.sprintf("%.3f", i / 1000.0);
            pt_txt.setY(i);

            pt_txt = drawTrans.map(pt_txt);
            pt_txt.setX(10);
            pt_txt.setY(pt_txt.y() - 5);
            painter.setPen(QPen(textColor));
            painter.drawText(pt_txt, txt);

            if (fabs(i) < 1e-3) {
                pen.setWidthF(zeroAxisWidth);
                pen.setColor(zeroAxisColor);
            } else {
                pen.setWidth(0);
                pen.setColor(secondAxisColor);
            }
            painter.setPen(pen);
        }

        QPointF pt_start(xStart, i);
        QPointF pt_end(xEnd, i);
        pt_start = drawTrans.map(pt_start);
        pt_end = drawTrans.map(pt_end);
        painter.drawLine(pt_start, pt_end);
    }

    // Store trace for the selected car
    if (mTraceCar >= 0) {
        for (int i = 0;i < mCarInfo.size();i++) {
            CarInfo &carInfo = mCarInfo[i];
            if (carInfo.getId() == mTraceCar) {
                if (mCarTrace.isEmpty()) {
                    mCarTrace.append(carInfo.getLocation());
                }

                if (mCarTrace.last().getDistanceTo(carInfo.getLocation()) > 0.05) {
                    mCarTrace.append(carInfo.getLocation());
                }
            }
        }

        // GPS Trace
        for (int i = 0;i < mCarInfo.size();i++) {
            CarInfo &carInfo = mCarInfo[i];
            if (carInfo.getId() == mTraceCar) {
                if (mCarTraceGps.isEmpty()) {
                    mCarTraceGps.append(carInfo.getLocationGps());
                }

                if (mCarTraceGps.last().getDistanceTo(carInfo.getLocationGps()) > 0.01) {
                    mCarTraceGps.append(carInfo.getLocationGps());
                }
            }
        }
    }

    // Draw trace for the selected car
    pen.setWidthF(5.0 / mScaleFactor);
    pen.setColor(Qt::red);
    painter.setPen(pen);
    painter.setTransform(drawTrans);
    for (int i = 1;i < mCarTrace.size();i++) {
        painter.drawLine(mCarTrace[i - 1].getX() * 1000.0, mCarTrace[i - 1].getY() * 1000.0,
                mCarTrace[i].getX() * 1000.0, mCarTrace[i].getY() * 1000.0);
    }

    // Draw GPS trace for the selected car
    pen.setWidthF(2.5 / mScaleFactor);
    pen.setColor(Qt::magenta);
    painter.setPen(pen);
    painter.setTransform(drawTrans);
    for (int i = 1;i < mCarTraceGps.size();i++) {
        painter.drawLine(mCarTraceGps[i - 1].getX() * 1000.0, mCarTraceGps[i - 1].getY() * 1000.0,
                mCarTraceGps[i].getX() * 1000.0, mCarTraceGps[i].getY() * 1000.0);
    }

    // Draw route
    pen.setWidthF(5.0 / mScaleFactor);
    pen.setColor(Qt::darkYellow);
    painter.setPen(pen);
    painter.setBrush(Qt::yellow);
    painter.setTransform(drawTrans);
    for (int i = 1;i < mRoute.size();i++) {
        painter.drawLine(mRoute[i - 1].getX() * 1000.0, mRoute[i - 1].getY() * 1000.0,
                mRoute[i].getX() * 1000.0, mRoute[i].getY() * 1000.0);
    }
    for (int i = 0;i < mRoute.size();i++) {
        QPointF p = mRoute[i].getPointMm();
        painter.setTransform(drawTrans);
        pen.setColor(Qt::darkYellow);
        painter.setPen(pen);
        painter.drawEllipse(p, 10 / mScaleFactor, 10 / mScaleFactor);

        txt.sprintf("%.1f km/h", mRoute[i].getSpeed() * 3.6);
        pt_txt.setX(p.x() + 10 / mScaleFactor);
        pt_txt.setY(p.y());
        painter.setTransform(txtTrans);
        pt_txt = drawTrans.map(pt_txt);
        pen.setColor(Qt::black);
        painter.setPen(pen);
        rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 20,
                           pt_txt.x() + 150, pt_txt.y() + 25);
        painter.drawText(rect_txt, txt);
    }

    // For debugging...
    if (mHasRouteClosest) {
        QPointF p = mRouteClosest.getPointMm();
        painter.setTransform(drawTrans);
        pen.setColor(Qt::darkGreen);
        painter.setPen(pen);
        painter.drawEllipse(p, 10 / mScaleFactor, 10 / mScaleFactor);

        QPointF pm = getMousePosRelative();
        painter.setBrush(Qt::transparent);
        painter.drawEllipse(pm, 1000.0, 1000.0);
    }

    // Draw cars
    painter.setPen(QPen(textColor));
    for(int i = 0;i < mCarInfo.size();i++) {
        CarInfo &carInfo = mCarInfo[i];
        LocPoint pos = carInfo.getLocation();
        LocPoint pos_gps = carInfo.getLocationGps();
        x = pos.getX() * 1000.0;
        y = pos.getY() * 1000.0;
        double x_gps = pos_gps.getX() * 1000.0;
        double y_gps = pos_gps.getY() * 1000.0;
        angle = pos.getAlpha() * 180.0 / M_PI;
        painter.setTransform(drawTrans);

        // Draw standard deviation
        QColor col = Qt::red;
        col.setAlphaF(0.2);
        painter.setBrush(QBrush(col));
        painter.drawEllipse(pos.getPointMm(), pos.getSigma() * 1000.0, pos.getSigma() * 1000.0);

        // Draw car
        painter.setBrush(QBrush(Qt::black));
        painter.save();
        painter.translate(x, y);
        painter.rotate(-angle);
        // Wheels
        painter.drawRoundedRect(-car_w / 12.0,-(car_h / 2), car_w / 6.0, car_h, car_corner / 3, car_corner / 3);
        painter.drawRoundedRect(car_w - car_w / 2.5,-(car_h / 2), car_w / 6.0, car_h, car_corner / 3, car_corner / 3);
        // Front bumper
        painter.setBrush(QBrush(Qt::green));
        painter.drawRoundedRect(-car_w / 6.0, -((car_h - car_w / 20.0) / 2.0), car_w, car_h - car_w / 20.0, car_corner, car_corner);
        // Hull
        painter.setBrush(QBrush(carInfo.getColor()));
        painter.drawRoundedRect(-car_w / 6.0, -((car_h - car_w / 20.0) / 2.0), car_w - (car_w / 20.0), car_h - car_w / 20.0, car_corner, car_corner);
        painter.restore();
        // Center
        painter.setBrush(QBrush(Qt::blue));
        painter.drawEllipse(QPointF(x, y), car_h / 15.0, car_h / 15.0);

        // GPS Location
        painter.setBrush(QBrush(Qt::magenta));
        painter.drawEllipse(QPointF(x_gps, y_gps), car_h / 15.0, car_h / 15.0);

        // Autopilot state
        LocPoint ap_goal = carInfo.getApGoal();
        if (ap_goal.getRadius() > 0.0) {
            QPointF p = ap_goal.getPointMm();
            pen.setColor(carInfo.getColor());
            painter.setPen(pen);
            painter.drawEllipse(p, 10 / mScaleFactor, 10 / mScaleFactor);

            QPointF pm = pos.getPointMm();
            painter.setBrush(Qt::transparent);
            painter.drawEllipse(pm, ap_goal.getRadius() * 1000.0, ap_goal.getRadius() * 1000.0);
        }

        painter.setPen(QPen(textColor));

        // Print data
        txt.sprintf("%s\n(%.3f, %.3f, %.0f)", carInfo.getName().toLocal8Bit().data(),
                    pos.getX(), pos.getY(), angle);
        pt_txt.setX(x + 120 + (car_w - 190) * ((cos(pos.getAlpha()) + 1) / 2));
        pt_txt.setY(y);
        painter.setTransform(txtTrans);
        pt_txt = drawTrans.map(pt_txt);
        rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 20,
                           pt_txt.x() + 150, pt_txt.y() + 25);
        painter.drawText(rect_txt, txt);
    }

    // Draw units (m)
    painter.setTransform(txtTrans);
    font.setPointSize(16);
    painter.setFont(font);
    txt = "(m)";
    painter.drawText(width() - 50, 35, txt);

    painter.end();
}

namespace {
typedef struct {
    float px;
    float py;
    float speed;
} ROUTE_POINT;

int circleLineIntersection(float cx, float cy, float rad,
                                const ROUTE_POINT *point1, const ROUTE_POINT *point2,
                                ROUTE_POINT *intersection1, ROUTE_POINT *intersection2)
{
    float dx, dy, a, b, c, det, t, x, y;

    const float p1x = point1->px;
    const float p1y = point1->py;
    const float p2x = point2->px;
    const float p2y = point2->py;

    float maxx = p1x;
    float minx = p2x;
    float maxy = p1y;
    float miny = p2y;

    if (maxx < minx) {
        maxx = p2x;
        minx = p1x;
    }

    if (maxy < miny) {
        maxy = p2y;
        miny = p1y;
    }

    dx = p2x - p1x;
    dy = p2y - p1y;

    a = dx * dx + dy * dy;
    b = 2 * (dx * (p1x - cx) + dy * (p1y - cy));
    c = (p1x - cx) * (p1x - cx) + (p1y - cy) * (p1y - cy) - rad * rad;

    det = b * b - 4 * a * c;

    int ints = 0;
    if ((a <= 1e-6) || (det < 0.0)) {
        // No real solutions.
    } else if (det == 0) {
        // One solution.
        t = -b / (2 * a);
        x = p1x + t * dx;
        y = p1y + t * dy;

        if (x >= minx && x <= maxx &&
                y >= miny && y <= maxy) {
            intersection1->px = x;
            intersection1->py = y;
            ints++;
        }
    } else {
        // Two solutions.
        t = (-b + sqrt(det)) / (2 * a);
        x = p1x + t * dx;
        y = p1y + t * dy;

        if (x >= minx && x <= maxx &&
                y >= miny && y <= maxy) {
            intersection1->px = x;
            intersection1->py = y;
            ints++;
        }

        t = (-b - sqrt(det)) / (2 * a);
        x = p1x + t * dx;
        y = p1y + t * dy;

        if (x >= minx && x <= maxx &&
                y >= miny && y <= maxy) {
            if (ints) {
                intersection2->px = x;
                intersection2->py = y;
            } else {
                intersection1->px = x;
                intersection1->py = y;
            }

            ints++;
        }
    }

    return ints;
}

void closestPointLineSegment(const ROUTE_POINT *point1, const ROUTE_POINT *point2, float px, float py, ROUTE_POINT *closest) {
    const float p1x = point1->px;
    const float p1y = point1->py;
    const float p2x = point2->px;
    const float p2y = point2->py;

    const float d1x = px - p1x;
    const float d1y = py - p1y;
    const float dx = p2x - p1x;
    const float dy = p2y - p1y;

    const float ab2 = dx * dx + dy * dy;
    const float ap_ab = d1x * dx + d1y * dy;
    float t = ap_ab / ab2;

    if (t < 0.0) {
        t = 0.0;
    } else if (t > 1.0) {
        t = 1.0;
    }

    closest->px = p1x + dx * t;
    closest->py = p1y + dy * t;
}

float pointDistance(const ROUTE_POINT *p1, const ROUTE_POINT *p2) {
    float dx = p2->px - p1->px;
    float dy = p2->py - p1->py;
    return sqrtf(dx * dx + dy * dy);
}
}

void MapWidget::mouseMoveEvent(QMouseEvent *e)
{
    if (e->buttons() & Qt::LeftButton && !(e->modifiers() & (Qt::ControlModifier | Qt::ShiftModifier)))
    {
        int x = e->pos().x();
        int y = e->pos().y();

        if (mMouseLastX < 100000)
        {
            int diffx = x - mMouseLastX;
            mXOffset += diffx;
            repaintAfterEvents();
        }

        if (mMouseLastY < 100000)
        {
            int diffy = y - mMouseLastY;
            mYOffset -= diffy;

            emit offsetChanged(mXOffset, mYOffset);
            repaintAfterEvents();
        }

        mMouseLastX = x;
        mMouseLastY = y;
    }

    if (false) {
        bool update_now = false;
        if (mRoute.size() >= 2) {
            QPoint p = getMousePosRelative();

            int add = 5;
            if (add > mRoute.size()) {
                add = mRoute.size();
            }

            int start = mRouteCurrentPoint;
            int end = mRouteCurrentPoint + add;

            float cx = p.x() / 1000.0;
            float cy = p.y() / 1000.0;

            ROUTE_POINT last;
            bool has = false;
            int current = 0;
            for (int i = start;i < end;i++) {
                int ind = i;
                int indn = i + 1;

                if (ind >= mRoute.size()) {
                    ind -= mRoute.size();
                }

                if (indn >= mRoute.size()) {
                    indn -= mRoute.size();
                }

                ROUTE_POINT int1, int2;
                ROUTE_POINT p1, p2;
                p1.px = mRoute.at(ind).getX();
                p1.py = mRoute.at(ind).getY();
                p2.px = mRoute.at(indn).getX();
                p2.py = mRoute.at(indn).getY();

                int res = circleLineIntersection(cx, cy, 1.0, &p1, &p2, &int1, &int2);

                if (!has && res > 0) {
                    has = true;
                    last = int1;
                    current = i;
                }

                if (res == 1) {
                    if (pointDistance(&p2, &int1) <
                            pointDistance(&last, &p2)) {
                        last = int1;
                        current = i;
                    }
                }

                if (res == 2) {
                    if (pointDistance(&p2, &int2) <
                            pointDistance(&last, &p2)) {
                        last = int2;
                        current = i;
                    }
                }
            }

            if (!has) {
                ROUTE_POINT mouse;
                mouse.px = cx;
                mouse.py = cy;
                bool lastSet = false;
                for (int i = start;i < end;i++) {
                    int ind = i;
                    int indn = i + 1;

                    if (ind >= mRoute.size()) {
                        ind -= mRoute.size();
                    }

                    if (indn >= mRoute.size()) {
                        indn -= mRoute.size();
                    }

                    ROUTE_POINT tmp;
                    ROUTE_POINT p1, p2;
                    p1.px = mRoute.at(ind).getX();
                    p1.py = mRoute.at(ind).getY();
                    p2.px = mRoute.at(indn).getX();
                    p2.py = mRoute.at(indn).getY();
                    closestPointLineSegment(&p1, &p2, cx, cy, &tmp);

                    if (!lastSet || pointDistance(&tmp, &mouse) < pointDistance(&last, &mouse)) {
                        last = tmp;
                        lastSet = true;
                        current = i;
                    }
                }
            }

            if (current > (mRouteCurrentPoint + 1) && current > 0) {
                mRouteCurrentPoint = current -  1;

                if (mRouteCurrentPoint >= mRoute.size()) {
                    mRouteCurrentPoint -= mRoute.size();
                }

                qDebug() << mRouteCurrentPoint;
            }

            mRouteClosest.setXY(last.px, last.py);
            mHasRouteClosest = true;
            update_now = true;
        } else {
            if (mHasRouteClosest) {
                update_now = true;
            }

            mHasRouteClosest = false;
        }

        if (mHasRouteClosest) {
            update_now = true;
        }

        if (update_now) {
            update();
        }
    }
}

void MapWidget::mousePressEvent(QMouseEvent *e)
{
    mRouteCurrentPoint = 0;

    if (mSelectedCar >= 0 && (e->buttons() & Qt::LeftButton) && (e->modifiers() & Qt::ControlModifier)) {
        for (int i = 0;i < mCarInfo.size();i++) {
            CarInfo &carInfo = mCarInfo[i];
            if (carInfo.getId() == mSelectedCar) {
                LocPoint pos = carInfo.getLocation();
                QPoint p = getMousePosRelative();
                pos.setXY(p.x() / 1000.0, p.y() / 1000.0);
                carInfo.setLocation(pos);
                emit posSet(mSelectedCar, pos);
                repaintAfterEvents();
            }
        }
    } else if (e->modifiers() & Qt::ShiftModifier) {
        if (e->buttons() & Qt::LeftButton) {
            LocPoint pos;
            QPoint p = getMousePosRelative();
            pos.setXY(p.x() / 1000.0, p.y() / 1000.0);
            pos.setSpeed(mRoutePointSpeed);
            mRoute.append(pos);
            emit routePointAdded(pos);
        } else if (e->buttons() & Qt::RightButton) {
            LocPoint pos;
            if (mRoute.size() > 0) {
                pos = mRoute.last();
                mRoute.removeLast();
            }
            emit lastRoutePointRemoved(pos);
        }
        repaintAfterEvents();
    }
}

void MapWidget::mouseReleaseEvent(QMouseEvent *e)
{
    if (!(e->buttons() & Qt::LeftButton))
    {
        mMouseLastX = 1000000;
        mMouseLastY = 1000000;
    }
}

void MapWidget::wheelEvent(QWheelEvent *e)
{
    if (e->modifiers() & Qt::ControlModifier && mSelectedCar >= 0) {
        for (int i = 0;i < mCarInfo.size();i++) {
            CarInfo &carInfo = mCarInfo[i];
            if (carInfo.getId() == mSelectedCar) {
                LocPoint pos = carInfo.getLocation();
                double angle = pos.getAlpha() + (double)e->delta() * 0.0005;
                normalizeAngleRad(angle);
                pos.setAlpha(angle);
                carInfo.setLocation(pos);
                emit posSet(mSelectedCar, pos);
                repaintAfterEvents();
            }
        }
    } else {
        int x = e->pos().x();
        int y = e->pos().y();
        double scaleDiff = ((double)e->delta() / 600.0);
        if (scaleDiff > 0.8)
        {
            scaleDiff = 0.8;
        }

        if (scaleDiff < -0.8)
        {
            scaleDiff = -0.8;
        }

        x -= width() / 2;
        y -= height() / 2;

        mScaleFactor += mScaleFactor * scaleDiff;
        mXOffset += mXOffset * scaleDiff;
        mYOffset += mYOffset * scaleDiff;

        emit scaleChanged(mScaleFactor);
        emit offsetChanged(mXOffset, mYOffset);
        repaintAfterEvents();
    }
}
