/*
    Copyright 2019 Benjamin Vedder	benjamin@vedder.se

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

/*
 * Install:
 * sudo apt install qtmultimedia5-dev libqt5multimedia5-plugins
 */

#ifndef CAMERA_H
#define CAMERA_H

#include <QObject>
#include <QCamera>
#include <QAbstractVideoSurface>

class MyVideoSurface : public QAbstractVideoSurface
{
    Q_OBJECT
public:
    MyVideoSurface(QObject* parent = nullptr);
    QList<QVideoFrame::PixelFormat>
    supportedPixelFormats(QAbstractVideoBuffer::HandleType h) const;
    bool present(const QVideoFrame &frame);

signals:
    void imageCaptured(QImage img);

};

class Camera : public QObject
{
    Q_OBJECT
public:
    explicit Camera(QObject *parent = nullptr);
    MyVideoSurface* video();
    bool openCamera(int cameraIndex = 0);
    void closeCamera();
    bool startCameraStream(int width = 640, int height = 480, int fps = -1);
    QString cameraInfo();
    bool isLoaded();

signals:

public slots:

private:
    QCamera *mCamera;
    MyVideoSurface *mVid;

};

#endif // CAMERA_H
