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

#include "imagewidget.h"
#include <QPainter>
#include <QPaintEvent>
#include <QDebug>

ImageWidget::ImageWidget(QWidget *parent) : QWidget(parent)
{

}

void ImageWidget::paintEvent(QPaintEvent *event)
{
    (void)event;

    if (!mPixmap.isNull()) {
        QPainter painter(this);
        painter.setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
        painter.fillRect(rect(), Qt::black);

        int pw = mPixmap.width();
        int ph = mPixmap.height();
        int w = width();
        int h = height();

        if (((double)pw / (double)ph) > ((double)w / (double)h)) {
            painter.drawPixmap(0, (h - (ph * w) / pw) / 2, w, (ph * w) / pw, mPixmap);
        } else {
            painter.drawPixmap((w - (pw * h) / ph) / 2, 0, (pw * h) / ph, h, mPixmap);
        }
    }
}

QPixmap ImageWidget::pixmap() const
{
    return mPixmap;
}

void ImageWidget::setPixmap(const QPixmap &pixmap)
{
    mPixmap = pixmap;
    update();
}
