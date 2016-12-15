/*
    Copyright 2012 Benjamin Vedder	benjamin@vedder.se

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

#ifndef LOCPOINT_H
#define LOCPOINT_H

#include <QPointF>
#include <QString>

class LocPoint
{
public:
    LocPoint(double x = 0, double y = 0, double alpha = 0,
             double speed = 0.5, double radius = 5.0, double sigma = 0.0,
             quint32 color = Qt::darkGreen, qint32 time = 0);
    LocPoint(const LocPoint &point);

    double getX() const;
    double getY() const;
    double getAlpha() const;
    double getSpeed() const;
    QPointF getPoint() const;
    QPointF getPointMm() const;
    double getRadius() const;
    double getSigma() const;
    QString getInfo() const;
    quint32 getColor() const;
    qint32 getTime() const;

    void setX(double x);
    void setY(double y);
    void setXY(double x, double y);
    void scaleXY(double scalefactor);
    void setAlpha(double alpha);
    void setSpeed(double speed);
    void setRadius(double radius);
    void setSigma(double sigma);
    double getDistanceTo(const LocPoint &point) const;
    void setInfo(const QString &info);
    void setColor(const quint32 &color);
    void setTime(const qint32 &time);

    // Operators
    LocPoint& operator=(const LocPoint& point);
    bool operator==(const LocPoint& point);
    bool operator!=(const LocPoint& point);

private:
    double mX;
    double mY;
    double mAlpha;
    double mSpeed;
    double mRadius;
    double mSigma;
    QString mInfo;
    quint32 mColor;
    qint32 mTime;

};

#endif // LOCPOINT_H
