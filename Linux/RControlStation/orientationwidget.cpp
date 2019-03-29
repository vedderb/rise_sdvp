/*
    Copyright 2013 - 2016 Benjamin Vedder	benjamin@vedder.se

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

#define GL_GLEXT_PROTOTYPES

#include <QtGui>
#include <QtOpenGL>

#include <math.h>

#ifdef HAS_ASSIMP
#include <assimp/Importer.hpp>
#include <assimp/mesh.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#endif

#include "orientationwidget.h"

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

namespace {
static void qNormalizeAngle(float &angle)
{
    angle = fmodf(angle, 360.0);

    if (angle < 0.0) {
        angle += 360.0;
    }
}

#ifdef HAS_ASSIMP
static void qSetColor(float colorVec[], QColor c)
{
    colorVec[0] = c.redF();
    colorVec[1] = c.greenF();
    colorVec[2] = c.blueF();
    colorVec[3] = c.alphaF();
}

static void get_bounding_box_for_node (const aiNode* nd,
                                       aiVector3D* min,
                                       aiVector3D* max,
                                       aiMesh **meshes)
{
    for (unsigned int n = 0; n < nd->mNumMeshes; ++n) {
        const aiMesh* mesh = meshes[nd->mMeshes[n]];

        for (unsigned int t = 0; t < mesh->mNumVertices; ++t) {
            aiVector3D tmp = mesh->mVertices[t];

            min->x = min->x < tmp.x ? min->x : tmp.x;
            min->y = min->y < tmp.y ? min->y : tmp.y;
            min->z = min->z < tmp.z ? min->z : tmp.z;

            max->x = max->x > tmp.x ? max->x : tmp.x;
            max->y = max->y > tmp.y ? max->y : tmp.y;
            max->z = max->z > tmp.z ? max->z : tmp.z;
        }
    }

    for (unsigned int n = 0; n < nd->mNumChildren; ++n) {
        get_bounding_box_for_node(nd->mChildren[n], min, max, meshes);
    }
}

static void get_bounding_box (aiVector3D* min, aiVector3D* max, const aiScene *scene)
{
    min->x = min->y = min->z =  1e10f;
    max->x = max->y = max->z = -1e10f;
    get_bounding_box_for_node(scene->mRootNode, min, max, scene->mMeshes);
}
#endif
}

OrientationWidget::OrientationWidget(QWidget *parent, MODEL_TYPE model)
    : QOpenGLWidget(parent)
{
    mXRot = 0;
    mYRot = 0;
    mZRot = 0;

    mQ0 = 1.0;
    mQ1 = 0.0;
    mQ2 = 0.0;
    mQ3 = 0.0;
    mUseQuaternions = false;

    // Set these to get the rotation of this particular model right
    mXRotOfs = 90.0;
    mYRotOfs = 180.0;
    mZRotOfs = 180.0;
    mZRotOfsCar = 0.0;

    mBgColor = palette().color(QPalette::Window);

    mUpdateTimer = new QTimer(this);
    mUpdateTimer->setSingleShot(true);
    connect(mUpdateTimer, SIGNAL(timeout()), this, SLOT(updateTimerSlot()));

#ifdef HAS_ASSIMP
    Assimp::Importer importer;
    const aiScene *scene = 0;

    if (model == MODEL_CAR) {
        QFile file(":/models/Models/car.3ds");
        if (file.open(QIODevice::ReadOnly)) {
            QByteArray data = file.readAll();
            scene = importer.ReadFileFromMemory(data.data(), data.size(),
                                                aiProcessPreset_TargetRealtime_Quality,
                                                "3ds");
        }
    } else {
        QFile file(":/models/Models/quadrotor.blend");
        if (file.open(QIODevice::ReadOnly)) {
            QByteArray data = file.readAll();
            scene = importer.ReadFileFromMemory(data.data(), data.size(),
                                                aiProcessPreset_TargetRealtime_Quality,
                                                "blend");
        }
    }

    if (!scene) {
        qWarning() << "Could not load scene";
        return;
    }

    aiVector3D scene_min, scene_max;
    get_bounding_box(&scene_min, &scene_max, scene);
    float tmp;
    tmp = scene_max.x - scene_min.x;
    tmp = scene_max.y - scene_min.y > tmp ? scene_max.y - scene_min.y : tmp;
    tmp = scene_max.z - scene_min.z > tmp ? scene_max.z - scene_min.z : tmp;
    mScale = 0.95 / tmp;

    mMeshes.resize(scene->mNumMeshes);

    for (unsigned int m = 0;m < scene->mNumMeshes;m++) {
        MESHDATA_t *mData = &mMeshes[m];
        aiMesh *mesh = scene->mMeshes[m];

        mData->numTriangles = mesh->mNumFaces * 3;
        mData->numUvCoords = mesh->GetNumUVChannels();

        mData->vertexArray.resize(mesh->mNumFaces*3*3);
        mData->normalArray.resize(mesh->mNumFaces*3*3);

        if (mesh->mTextureCoords[0] != NULL) {
            mData->uvArray.resize(mesh->mNumFaces * 3 * 2 * 5);
        }

        int vertexIndex = 0;
        int normalIndex = 0;
        int uvIndex = 0;

        for(unsigned int i=0;i < mesh->mNumFaces;i++) {
            const aiFace &face = mesh->mFaces[i];

            for(int j = 0;j < 3;j++) {
                if (mesh->mTextureCoords[0] != NULL) {
                    aiVector3D uv = mesh->mTextureCoords[0][face.mIndices[j]];
                    memcpy(mData->uvArray.data() + uvIndex, &uv,sizeof(float)*2);
                    uvIndex += 2;
                }

                aiVector3D normal = mesh->mNormals[face.mIndices[j]];
                memcpy(mData->normalArray.data() + normalIndex, &normal, sizeof(float)*3);
                normalIndex += 3;

                aiVector3D pos = mesh->mVertices[face.mIndices[j]];
                memcpy(mData->vertexArray.data() + vertexIndex, &pos, sizeof(float)*3);
                vertexIndex += 3;
            }
        }

        switch (m) {
        case 0:
            qSetColor(mData->faceColor, Qt::red);
            break;

        case 1:
            qSetColor(mData->faceColor, Qt::black);
            break;

        default:
            qSetColor(mData->faceColor, QColor(20, 20, 25));
            break;
        }
    }

//    saveModel("quadrotor.model");
#else
    if (model == MODEL_CAR) {
        loadModel(":/models/Models/car.model");
    } else {
        loadModel(":/models/Models/quadrotor.model");
    }
#endif
}

OrientationWidget::~OrientationWidget()
{
}

QSize OrientationWidget::minimumSizeHint() const
{
    return QSize(0, 0);
}

QSize OrientationWidget::sizeHint() const
{
    return QSize(0, 0);
}

void OrientationWidget::setRollPitchYaw(float roll, float pitch, float yaw)
{
    mXRot = roll;
    mYRot = -pitch;
    mZRot = -yaw;
    mUseQuaternions = false;
    updateUsingTimer();
}

void OrientationWidget::setYawOffset(float offset)
{
    mZRotOfsCar = -offset;
    updateUsingTimer();
}

void OrientationWidget::setQuanternions(float q0, float q1, float q2, float q3)
{
    mQ0 = q0;
    mQ1 = q1;
    mQ2 = q2;
    mQ3 = q3;
    mUseQuaternions = true;
    updateUsingTimer();
}

void OrientationWidget::saveModel(QString path)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return;
    }

    QTextStream out(&file);

    out << mScale << "\n";
    out << mMeshes.size() << "\n";

    for (int m = 0;m < mMeshes.size();m++) {
        MESHDATA_t *mData = &mMeshes[m];

        out << mData->numTriangles << "\n";
        out << mData->numUvCoords << "\n";

        out << mData->faceColor[0] << "\n";
        out << mData->faceColor[1] << "\n";
        out << mData->faceColor[2] << "\n";
        out << mData->faceColor[3] << "\n";

        out << mData->normalArray.size() << "\n";
        for (int i = 0;i < mData->normalArray.size();i++) {
            out << mData->normalArray.at(i) << "\n";
        }

        out << mData->vertexArray.size() << "\n";
        for (int i = 0;i < mData->vertexArray.size();i++) {
            out << mData->vertexArray.at(i) << "\n";
        }

        out << mData->uvArray.size() << "\n";
        for (int i = 0;i < mData->uvArray.size();i++) {
            out << mData->uvArray.at(i) << "\n";
        }

    }

    file.close();
}

void OrientationWidget::loadModel(QString path)
{
    QFile file(path);
    if (file.exists()) {
        if (file.open(QIODevice::ReadOnly)) {
            QTextStream in(&file);

            mMeshes.clear();
            mScale = in.readLine().toDouble();
            int len = in.readLine().toInt();

            for (int m = 0;m < len;m++) {
                MESHDATA_t mData;

                mData.numTriangles = in.readLine().toInt();
                mData.numUvCoords = in.readLine().toInt();

                mData.faceColor[0] = in.readLine().toDouble();
                mData.faceColor[1] = in.readLine().toDouble();
                mData.faceColor[2] = in.readLine().toDouble();
                mData.faceColor[3] = in.readLine().toDouble();

                int normLen = in.readLine().toInt();
                for (int i = 0;i < normLen;i++) {
                    mData.normalArray.append(in.readLine().toDouble());
                }

                int vertLen = in.readLine().toInt();
                for (int i = 0;i < vertLen;i++) {
                    mData.vertexArray.append(in.readLine().toDouble());
                }

                int uvLen = in.readLine().toInt();
                for (int i = 0;i < uvLen;i++) {
                    mData.uvArray.append(in.readLine().toDouble());
                }

                mMeshes.append(mData);
            }

            file.close();

            return;
        }
    }

    qWarning() << "Could not load" << path;
    return;
}

void OrientationWidget::setXRotation(float angle)
{
    qNormalizeAngle(angle);
    if (angle != mXRot) {
        mXRot = angle;
        emit xRotationChanged(angle);
        updateUsingTimer();
    }
}

void OrientationWidget::setYRotation(float angle)
{
    qNormalizeAngle(angle);
    if (angle != mYRot) {
        mYRot = angle;
        emit yRotationChanged(angle);
        updateUsingTimer();
    }
}

void OrientationWidget::setZRotation(float angle)
{
    qNormalizeAngle(angle);
    if (angle != mZRot) {
        mZRot = angle;
        emit zRotationChanged(angle);
        updateUsingTimer();
    }
}

void OrientationWidget::updateTimerSlot()
{
    update();
}

void OrientationWidget::initializeGL()
{
    glClearColor(mBgColor.redF(), mBgColor.greenF(), mBgColor.blueF(), mBgColor.alphaF());
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_MULTISAMPLE);

    static GLfloat lightPosition[4] = { 0.5, 5.0, 7.0, 1.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
}

void OrientationWidget::paintGL()
{
    int width = size().width();
    int height = size().height();
    int side = qMin(width, height);
    glViewport((width - side) / 2, (height - side) / 2, side, side);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glTranslatef(0.0, 0.0, -10.0);
    glScalef(mScale, mScale, mScale);

    // Rotate world
    glRotatef(mZRotOfs, 0.0, 0.0, 1.0);
    glRotatef(mYRotOfs, 0.0, 1.0, 0.0);
    glRotatef(mXRotOfs, 1.0, 0.0, 0.0);

    // Rotate car
    if (mUseQuaternions) {
        float qw = mQ0;
        float qx = mQ1;
        float qy = mQ2;
        float qz = mQ3;

        GLfloat rot[16] = {1.0f - 2.0f*qy*qy - 2.0f*qz*qz, 2.0f*qx*qy - 2.0f*qz*qw, 2.0f*qx*qz + 2.0f*qy*qw, 0.0f,
                           2.0f*qx*qy + 2.0f*qz*qw, 1.0f - 2.0f*qx*qx - 2.0f*qz*qz, 2.0f*qy*qz - 2.0f*qx*qw, 0.0f,
                           2.0f*qx*qz - 2.0f*qy*qw, 2.0f*qy*qz + 2.0f*qx*qw, 1.0f - 2.0f*qx*qx - 2.0f*qy*qy, 0.0f,
                           0.0f, 0.0f, 0.0f, 1.0f};

        GLfloat rot_t[16] = {rot[0], rot[4], rot[8], rot[12],
                            rot[1], rot[5], rot[9], rot[13],
                            rot[2], rot[6], rot[10], rot[14],
                            rot[3], rot[7], rot[11], rot[15]};

        glRotatef(mZRotOfsCar, 0.0, 0.0, 1.0);
        glMultMatrixf(rot_t);
        glRotatef(180.0, 0.0, 0.0, 1.0);
    } else {
        glRotatef(mZRot + mZRotOfsCar, 0.0, 0.0, 1.0);
        glRotatef(mYRot, 0.0, 1.0, 0.0);
        glRotatef(mXRot, 1.0, 0.0, 0.0);
    }

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);

    for (int m = 0;m < mMeshes.size();m++) {
        MESHDATA_t *mData = &mMeshes[m];

        glVertexPointer(3,GL_FLOAT, 0, mData->vertexArray.data());
        glNormalPointer(GL_FLOAT, 0, mData->normalArray.data());

        if (mData->uvArray.size() > 0) {
            glEnableClientState(GL_TEXTURE_COORD_ARRAY);
#ifdef UNIX
            glClientActiveTexture(GL_TEXTURE0_ARB);
#endif
            glTexCoordPointer(2,GL_FLOAT, 0, mData->uvArray.data());
        } else {
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mData->faceColor);
        }

        glDrawArrays(GL_TRIANGLES, 0, mData->numTriangles);

        if (mData->uvArray.size() > 0) {
            glDisableClientState(GL_TEXTURE_COORD_ARRAY);
        }
    }

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
}

void OrientationWidget::resizeGL(int width, int height)
{
    (void)width;
    (void)height;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-0.5, +0.5, -0.5, +0.5, -10.0, 50.0);
    glMatrixMode(GL_MODELVIEW);
}

void OrientationWidget::mousePressEvent(QMouseEvent *event)
{
    mLastPos = event->pos();
}

void OrientationWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - mLastPos.x();
    int dy = event->y() - mLastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRotation(mXRot + 0.5 * (float)dx);
        setYRotation(mYRot + 0.5 * (float)-dy);
    } else if (event->buttons() & Qt::RightButton) {
        setYRotation(mYRot + 0.5 * (float)-dy);
        setZRotation(mZRot + 0.5 * (float)dx);
    }
    mLastPos = event->pos();
}

void OrientationWidget::updateUsingTimer()
{
    if (!mUpdateTimer->isActive()) {
        mUpdateTimer->start();
    }
}
