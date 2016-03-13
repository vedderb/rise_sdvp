#include "perspectivepixmap.h"
#include <cmath>

PerspectivePixmap::PerspectivePixmap()
{

}

void PerspectivePixmap::setPixmap(QPixmap pixmap)
{
    mPixMap = pixmap;
}

void PerspectivePixmap::setXOffset(double offset)
{
    mXOffset = offset;
}

void PerspectivePixmap::setYOffset(double offset)
{
    mYOffset = offset;
}

void PerspectivePixmap::setScale(double scale)
{
    mScale = scale;
}

QPixmap PerspectivePixmap::getPixmap()
{
    return mPixMap;
}

double PerspectivePixmap::getXOffset()
{
    return mXOffset;
}

double PerspectivePixmap::getYOffset()
{
    return mYOffset;
}

double PerspectivePixmap::getScale()
{
    return mScale;
}

void PerspectivePixmap::drawUsingPainter(QPainter &painter)
{
    if (!mPixMap.isNull()) {
        qreal opOld = painter.opacity();
        QTransform transOld = painter.transform();
        QTransform trans = painter.transform();
        trans.scale(1, -1);
        painter.setTransform(trans);
        painter.setOpacity(0.8);
        painter.drawPixmap(-mXOffset / mScale, -mYOffset / mScale,
                           mPixMap.width() / mScale, mPixMap.height() / mScale, mPixMap);
        painter.setOpacity(opOld);
        painter.setTransform(transOld);
    }
}
