/*
    Copyright 2012 - 2016 Benjamin Vedder	benjamin@vedder.se

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

#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QGLWidget>
#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QBrush>
#include <QFont>
#include <QPen>
#include <QPalette>
#include <QList>
#include <QInputDialog>
#include <QTimer>

#include "locpoint.h"
#include "carinfo.h"
#include "perspectivepixmap.h"
#include "osmclient.h"

// QWidget or QGLWidget
#define MapWidgetType   QGLWidget

class MapWidget : public MapWidgetType
{
    Q_OBJECT

public:
    explicit MapWidget(QWidget *parent = 0);
    CarInfo* getCarInfo(int car);
    void setFollowCar(int car);
    void setTraceCar(int car);
    void setSelectedCar(int car);
    void addCar(CarInfo car);
    bool removeCar(int carId);
    void setScaleFactor(double scale);
    void setRotation(double rotation);
    void setXOffset(double offset);
    void setYOffset(double offset);
    void clearTrace();
    void addRoutePoint(double px, double py, double speed);
    QList<LocPoint> getRoute();
    void setRoute(QList<LocPoint> route);
    void clearRoute();
    void setRoutePointSpeed(double speed);
    void addInfoPoint(LocPoint &info);
    void clearInfoTrace();
    void addPerspectivePixmap(PerspectivePixmap map);
    void clearPerspectivePixmaps();
    QPoint getMousePosRelative();
    void repaintAfterEvents();
    void setAntialiasing(bool antialias);
    bool getDrawOpenStreetmap() const;
    void setDrawOpenStreetmap(bool drawOpenStreetmap);
    void setEnuRef(double lat, double lon, double height);
    void getEnuRef(double *llh);

signals:
    void scaleChanged(double newScale);
    void offsetChanged(double newXOffset, double newYOffset);
    void posSet(quint8 id, LocPoint pos);
    void routePointAdded(LocPoint pos);
    void lastRoutePointRemoved(LocPoint pos);

public slots:
    void paintTimerSlot();

private slots:
    void tileReady(OsmTile tile);
    void errorGetTile(QString reason);

protected:
    void paintEvent(QPaintEvent *event);
    void mouseMoveEvent (QMouseEvent * e);
    void mousePressEvent(QMouseEvent * e);
    void mouseReleaseEvent(QMouseEvent * e);
    void wheelEvent(QWheelEvent * e);

private:
    QList<CarInfo> mCarInfo;
    QList<LocPoint> mCarTrace;
    QList<LocPoint> mCarTraceGps;
    QList<LocPoint> mRoute;
    QList<LocPoint> mInfoTrace;
    QList<PerspectivePixmap> mPerspectivePixmaps;
    double mRoutePointSpeed;
    double mScaleFactor;
    double mRotation;
    double mXOffset;
    double mYOffset;
    int mMouseLastX;
    int mMouseLastY;
    int mFollowCar;
    int mTraceCar;
    int mSelectedCar;
    double xRealPos;
    double yRealPos;
    QTimer *mPaintTimer;
    bool mAntialias;

    OsmClient *mOsm;
    int mOsmZoomLevel;
    bool mDrawOpenStreetmap;
    double mRefLat;
    double mRefLon;
    double mRefHeight;

};

#endif // MAPWIDGET_H
