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

#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>

#include <QtGui>
#include <QtOpenGL>

#include <math.h>

#include <assimp/Importer.hpp>
#include <assimp/mesh.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include "orientationwidget.h"

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

namespace {
static void qSetColor(float colorVec[], QColor c)
{
    colorVec[0] = c.redF();
    colorVec[1] = c.greenF();
    colorVec[2] = c.blueF();
    colorVec[3] = c.alphaF();
}

static void qNormalizeAngle(float &angle)
{
    angle = fmodf(angle, 360.0);

    if (angle < 0.0) {
        angle += 360.0;
    }
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
}

OrientationWidget::OrientationWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
    xRot = 0;
    yRot = 0;
    zRot = 0;

    // Set these to get the rotation of this particular model right
    xRotOfs = 270.0;
    yRotOfs = 0.0;
    zRotOfs = 90.0;

    bgColor = palette().color(QPalette::Window);

    Assimp::Importer importer;
    QString path = "Models/quadrotor_base.blend";
    const aiScene *scene = importer.ReadFile(path.toLocal8Bit().data(),
                                             aiProcessPreset_TargetRealtime_Quality);

    if (!scene) {
        qDebug() << "Could not load" << path;
        return;
    }

    aiVector3D scene_min, scene_max;
    get_bounding_box(&scene_min, &scene_max, scene);
    float tmp;
    tmp = scene_max.x - scene_min.x;
    tmp = scene_max.y - scene_min.y > tmp ? scene_max.y - scene_min.y : tmp;
    tmp = scene_max.z - scene_min.z > tmp ? scene_max.z - scene_min.z : tmp;
    scale = 0.95 / tmp;

    quadMeshes.resize(scene->mNumMeshes);

    for (unsigned int m = 0;m < scene->mNumMeshes;m++) {
        MESHDATA_t *mData = &quadMeshes[m];
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

        case 2:
            qSetColor(mData->faceColor, Qt::darkCyan);
            break;

        default:
            if (m % 2 == 0) {
                qSetColor(mData->faceColor, Qt::darkBlue);
            } else {
                qSetColor(mData->faceColor, Qt::green);
            }
            break;
        }
    }
}

OrientationWidget::~OrientationWidget()
{
}

QSize OrientationWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize OrientationWidget::sizeHint() const
{
    return QSize(400, 400);
}

void OrientationWidget::setRollPitchYaw(float roll, float pitch, float yaw)
{
    // There is probably a better way to handle this...
    xRot = sin(-yaw * M_PI / 180.0) * roll + cos(-yaw * M_PI / 180.0) * pitch;
    yRot = cos(-yaw * M_PI / 180.0) * roll - sin(-yaw * M_PI / 180.0) * pitch;
    zRot = yaw;

    qNormalizeAngle(xRot);
    qNormalizeAngle(yRot);
    qNormalizeAngle(zRot);

    updateGL();
}

void OrientationWidget::setXRotation(float angle)
{
    qNormalizeAngle(angle);
    if (angle != xRot) {
        xRot = angle;
        emit xRotationChanged(angle);
        updateGL();
    }
}

void OrientationWidget::setYRotation(float angle)
{
    qNormalizeAngle(angle);
    if (angle != yRot) {
        yRot = angle;
        emit yRotationChanged(angle);
        updateGL();
    }
}

void OrientationWidget::setZRotation(float angle)
{
    qNormalizeAngle(angle);
    if (angle != zRot) {
        zRot = angle;
        emit zRotationChanged(angle);
        updateGL();
    }
}

void OrientationWidget::initializeGL()
{
    qglClearColor(bgColor);
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
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glTranslatef(0.0, 0.0, -10.0);
    glScalef(scale, scale, scale);
    glRotatef(xRot + xRotOfs, 1.0, 0.0, 0.0);
    glRotatef(yRot + yRotOfs, 0.0, 1.0, 0.0);
    glRotatef(zRot + zRotOfs, 0.0, 0.0, 1.0);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);

    for (int m = 0;m < quadMeshes.size();m++) {
        MESHDATA_t *mData = &quadMeshes[m];

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
    int side = qMin(width, height);
    glViewport((width - side) / 2, (height - side) / 2, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-0.5, +0.5, -0.5, +0.5, -10.0, 50.0);
    glMatrixMode(GL_MODELVIEW);
}

void OrientationWidget::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}

void OrientationWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRotation(xRot + 0.5 * (float)dy);
        setYRotation(yRot + 0.5 * (float)dx);
    } else if (event->buttons() & Qt::RightButton) {
        setXRotation(xRot + 0.5 * (float)dy);
        setZRotation(zRot + 0.5 * (float)dx);
    }
    lastPos = event->pos();
}
