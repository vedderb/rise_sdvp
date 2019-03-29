/*
    Copyright 2013 - 2017 Benjamin Vedder	benjamin@vedder.se

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

#ifndef ORIENTATIONWIDGET_H
#define ORIENTATIONWIDGET_H

#include <QOpenGLWidget>
#include <QTimer>

class OrientationWidget : public QOpenGLWidget
{
    Q_OBJECT

public:
    typedef struct {
        QVector<float> vertexArray;
        QVector<float> normalArray;
        QVector<float> uvArray;
        int numTriangles;
        int numUvCoords;
        float faceColor[4];
    } MESHDATA_t;

    typedef enum {
        MODEL_CAR = 0,
        MODEL_QUADROTOR
    } MODEL_TYPE;

    OrientationWidget(QWidget *parent = 0, MODEL_TYPE model = MODEL_CAR);
    ~OrientationWidget();

    QSize minimumSizeHint() const;
    QSize sizeHint() const;
    void setRollPitchYaw(float roll, float pitch, float yaw);
    void setYawOffset(float offset);
    void setQuanternions(float q0, float q1, float q2, float q3);
    void saveModel(QString path);
    void loadModel(QString path);

public slots:
    void setXRotation(float angle);
    void setYRotation(float angle);
    void setZRotation(float angle);
    void updateTimerSlot();

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
    float mXRot;
    float mYRot;
    float mZRot;
    float mXRotOfs;
    float mYRotOfs;
    float mZRotOfs;
    float mZRotOfsCar;
    float mScale;
    float mQ0, mQ1, mQ2, mQ3;
    bool mUseQuaternions;
    QPoint mLastPos;
    QColor mBgColor;
    QVector<MESHDATA_t> mMeshes;
    QTimer *mUpdateTimer;

    void updateUsingTimer();

};

#endif
