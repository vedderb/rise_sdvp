/*
    Copyright 2013 - 2015 Benjamin Vedder	benjamin@vedder.se

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

#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QtOpenGL/QGLFunctions>

class OrientationWidget : public QGLWidget, protected QGLFunctions
{
    Q_OBJECT

public:
    OrientationWidget(QWidget *parent = 0);
    ~OrientationWidget();

    QSize minimumSizeHint() const;
    QSize sizeHint() const;
    void setRollPitchYaw(float roll, float pitch, float yaw);

    typedef struct {
        QVector<float> vertexArray;
        QVector<float> normalArray;
        QVector<float> uvArray;
        int numTriangles;
        int numUvCoords;
        float faceColor[4];
    } MESHDATA_t;

public slots:
    void setXRotation(float angle);
    void setYRotation(float angle);
    void setZRotation(float angle);

signals:
    void xRotationChanged(float angle);
    void yRotationChanged(float angle);
    void zRotationChanged(float angle);

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

private:
    float xRot;
    float yRot;
    float zRot;
    float xRotOfs;
    float yRotOfs;
    float zRotOfs;
    float scale;
    QPoint lastPos;
    QColor bgColor;
    QVector<MESHDATA_t> quadMeshes;

};

#endif
