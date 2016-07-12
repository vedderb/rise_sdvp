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
#include "utility.h"

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
    mScaleFactor = 0.1;
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
    mAntialiasDrawings = false;
    mAntialiasOsm = true;
    mInfoTraceTextZoom = 0.5;

    mOsm = new OsmClient(this);
    mDrawOpenStreetmap = true;
    mOsmZoomLevel = 15;
    mOsmRes = 1.0;

    // Set this to the SP base station position for now
    mRefLat = 57.71495867;
    mRefLon = 12.89134921;
    mRefHeight = 219.0;

    // Hardcoded for now
    mOsm->setCacheDir("osm_tiles");
//    mOsm->setTileServerUrl("http://tile.openstreetmap.org");
//    mOsm->setTileServerUrl("http://home.vedder.se/osm_tiles");
    mOsm->setTileServerUrl("https://c.osm.rrze.fau.de/osmhd");

    connect(mOsm, SIGNAL(tileReady(OsmTile)),
            this, SLOT(tileReady(OsmTile)));
    connect(mOsm, SIGNAL(errorGetTile(QString)),
            this, SLOT(errorGetTile(QString)));

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
    update();
}

void MapWidget::setRoutePointSpeed(double speed)
{
    mRoutePointSpeed = speed;
}

void MapWidget::addInfoPoint(LocPoint &info)
{
    mInfoTrace.append(info);
    update();
}

void MapWidget::clearInfoTrace()
{
    mInfoTrace.clear();
    update();
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

void MapWidget::setAntialiasDrawings(bool antialias)
{
    mAntialiasDrawings = antialias;
    update();
}

void MapWidget::setAntialiasOsm(bool antialias)
{
    mAntialiasOsm = antialias;
    update();
}

void MapWidget::paintTimerSlot()
{
    update();
}

void MapWidget::tileReady(OsmTile tile)
{
    (void)tile;
    update();
}

void MapWidget::errorGetTile(QString reason)
{
    qWarning() << "OSM tile error:" << reason;
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

    painter.setRenderHint(QPainter::Antialiasing, mAntialiasDrawings);
    painter.setRenderHint(QPainter::TextAntialiasing, mAntialiasDrawings);

    const double scaleMax = 20;
    const double scaleMin = 0.000001;

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

    // Grid parameters
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

    const double cx = -mXOffset / mScaleFactor / 1000.0;
    const double cy = -mYOffset / mScaleFactor / 1000.0;
    const double view_w = width() / mScaleFactor / 1000.0;
    const double view_h = height() / mScaleFactor / 1000.0;

    // Draw perspective pixmaps first
    painter.setTransform(drawTrans);
    for(int i = 0;i < mPerspectivePixmaps.size();i++) {
        mPerspectivePixmaps[i].drawUsingPainter(painter);
    }

    // Draw openstreetmap tiles
    if (mDrawOpenStreetmap) {
        double i_llh[3];
        i_llh[0] = mRefLat;
        i_llh[1] = mRefLon;
        i_llh[2] = mRefHeight;

        mOsmZoomLevel = (int)round(log2(mScaleFactor * mOsmRes * 130000000.0 *
                                        cos(i_llh[0] * M_PI / 180.0)));
        if (mOsmZoomLevel > 19) {
            mOsmZoomLevel = 19;
        } else if (mOsmZoomLevel < 0) {
            mOsmZoomLevel = 0;
        }

        int xt = OsmTile::long2tilex(i_llh[1], mOsmZoomLevel);
        int yt = OsmTile::lat2tiley(i_llh[0], mOsmZoomLevel);

        double llh_t[3];
        llh_t[0] = OsmTile::tiley2lat(yt, mOsmZoomLevel);
        llh_t[1] = OsmTile::tilex2long(xt, mOsmZoomLevel);
        llh_t[2] = 0.0;

        double xyz[3];
        utility::llhToEnu(i_llh, llh_t, xyz);

        // Calculate scale at ENU origin
        double w = OsmTile::lat2width(i_llh[0], mOsmZoomLevel);

        int t_ofs_x = (int)ceil(-(cx - view_w / 2.0) / w);
        int t_ofs_y = (int)ceil((cy + view_h / 2.0) / w);

        painter.setRenderHint(QPainter::SmoothPixmapTransform, mAntialiasOsm);

        for (int j = 0;j < 40;j++) {
            for (int i = 0;i < 40;i++) {
                int xt_i = xt + i - t_ofs_x;
                int yt_i = yt + j - t_ofs_y;
                double ts_x = xyz[0] + w * i - (double)t_ofs_x * w;
                double ts_y = -xyz[1] + w * j - (double)t_ofs_y * w;

                // We are outside the view
                if (ts_x > (cx + view_w / 2.0)) {
                    break;
                } else if ((ts_y - w) > (-cy + view_h / 2.0)) {
                    break;
                }

                // This tile is not part of the map
                if (xt_i < 0 || yt_i < 0 ||
                        xt_i >= (1 << mOsmZoomLevel) ||
                        yt_i >= (1 << mOsmZoomLevel)) {
                    continue;
                }

                int res;
                OsmTile t = mOsm->getTile(mOsmZoomLevel, xt_i, yt_i, res);

                if (res > 0) {
                    if (w < 0.0) {
                        w = t.getWidthTop();
                    }

                    QTransform transOld = painter.transform();
                    QTransform trans = painter.transform();
                    trans.scale(1, -1);
                    painter.setTransform(trans);
                    painter.drawPixmap(ts_x * 1000.0, ts_y * 1000.0,
                                       w * 1000.0, w * 1000.0, t.pixmap());
                    painter.setTransform(transOld);
                } else {
                    if (!mOsm->downloadQueueFull()) {
                        mOsm->downloadTile(mOsmZoomLevel, xt_i, yt_i);
                    }
                }
            }
        }

        // Restore antialiasing
        painter.setRenderHint(QPainter::SmoothPixmapTransform, mAntialiasDrawings);
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

    // Draw info trace
    pen.setWidthF(3.0 / mScaleFactor);
    pen.setColor(Qt::darkGreen);
    painter.setPen(pen);
    painter.setBrush(Qt::green);
    painter.setTransform(drawTrans);
    for (int i = 1;i < mInfoTrace.size();i++) {
        painter.drawLine(mInfoTrace[i - 1].getX() * 1000.0, mInfoTrace[i - 1].getY() * 1000.0,
                mInfoTrace[i].getX() * 1000.0, mInfoTrace[i].getY() * 1000.0);
    }

    // Draw green points first
    for (int i = 0;i < mInfoTrace.size();i++) {
        const LocPoint &ip = mInfoTrace[i];
        QPointF p = ip.getPointMm();

        if ((ip.getColor() == Qt::green || ip.getColor() == Qt::darkGreen) &&
                p.x() > xStart && p.x() < xEnd && p.y() > yStart && p.y() < yEnd) {
            painter.setTransform(drawTrans);
            pen.setColor(ip.getColor());
            painter.setBrush(ip.getColor());
            painter.setPen(pen);
            painter.drawEllipse(p, 5 / mScaleFactor, 5 / mScaleFactor);

            if (mScaleFactor > mInfoTraceTextZoom) {
                pt_txt.setX(p.x() + 5 / mScaleFactor);
                pt_txt.setY(p.y());
                painter.setTransform(txtTrans);
                pt_txt = drawTrans.map(pt_txt);
                pen.setColor(Qt::black);
                painter.setPen(pen);
                painter.setFont(QFont("monospace"));
                rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 20,
                                   pt_txt.x() + 500, pt_txt.y() + 500);
                painter.drawText(rect_txt, Qt::AlignTop | Qt::AlignLeft, ip.getInfo());
            }
        }
    }

    // Draw other points after the green ones so that they come on top
    for (int i = 0;i < mInfoTrace.size();i++) {
        const LocPoint &ip = mInfoTrace[i];
        QPointF p = ip.getPointMm();

        if (ip.getColor() != Qt::green && ip.getColor() != Qt::darkGreen &&
                p.x() > xStart && p.x() < xEnd && p.y() > yStart && p.y() < yEnd) {
            painter.setTransform(drawTrans);
            pen.setColor(ip.getColor());
            painter.setBrush(ip.getColor());
            painter.setPen(pen);
            painter.drawEllipse(p, 5 / mScaleFactor, 5 / mScaleFactor);

            if (mScaleFactor > mInfoTraceTextZoom) {
                pt_txt.setX(p.x() + 5 / mScaleFactor);
                pt_txt.setY(p.y());
                painter.setTransform(txtTrans);
                pt_txt = drawTrans.map(pt_txt);
                pen.setColor(Qt::black);
                painter.setPen(pen);
                painter.setFont(QFont("monospace"));
                rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 20,
                                   pt_txt.x() + 500, pt_txt.y() + 500);
                painter.drawText(rect_txt, Qt::AlignTop | Qt::AlignLeft, ip.getInfo());
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
    font.setPointSize(12);
    painter.setFont(font);
    txt = "Grid unit: m";
    painter.drawText(width() - 140, 30, txt);

    // Draw zoom level
    painter.setTransform(txtTrans);
    font.setPointSize(12);
    painter.setFont(font);
    txt.sprintf("Zoom: %.7f", mScaleFactor);
    painter.drawText(width() - 140, 55, txt);

    // Draw OSM zoom level
    if (mDrawOpenStreetmap) {
        painter.setTransform(txtTrans);
        font.setPointSize(12);
        painter.setFont(font);
        txt.sprintf("OSM zoom: %d", mOsmZoomLevel);
        painter.drawText(width() - 140, 80, txt);
    }

    painter.end();
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
}

void MapWidget::mousePressEvent(QMouseEvent *e)
{
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

double MapWidget::getInfoTraceTextZoom() const
{
    return mInfoTraceTextZoom;
}

void MapWidget::setInfoTraceTextZoom(double infoTraceTextZoom)
{
    mInfoTraceTextZoom = infoTraceTextZoom;
    update();
}

OsmClient *MapWidget::osmClient()
{
    return mOsm;
}

double MapWidget::getOsmRes() const
{
    return mOsmRes;
}

void MapWidget::setOsmRes(double osmRes)
{
    mOsmRes = osmRes;
    update();
}

bool MapWidget::getDrawOpenStreetmap() const
{
    return mDrawOpenStreetmap;
}

void MapWidget::setDrawOpenStreetmap(bool drawOpenStreetmap)
{
    mDrawOpenStreetmap = drawOpenStreetmap;
}

void MapWidget::setEnuRef(double lat, double lon, double height)
{
    mRefLat = lat;
    mRefLon = lon;
    mRefHeight = height;
    update();
}

void MapWidget::getEnuRef(double *llh)
{
    llh[0] = mRefLat;
    llh[1] = mRefLon;
    llh[2] = mRefHeight;
}
