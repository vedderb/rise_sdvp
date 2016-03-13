#ifndef PERSPECTIVEPIXMAP_H
#define PERSPECTIVEPIXMAP_H

#include <QPixmap>
#include <QPainter>

class PerspectivePixmap
{

public:
    explicit PerspectivePixmap();
    void setPixmap(QPixmap pixmap);
    void setXOffset(double offset);
    void setYOffset(double offset);
    void setScale(double scale);
    QPixmap getPixmap();
    double getXOffset();
    double getYOffset();
    double getScale();
    void drawUsingPainter(QPainter &painter);

signals:
    
public slots:

private:
    QPixmap mPixMap;
    double mXOffset;
    double mYOffset;
    double mScale;
    
};

#endif // PERSPECTIVEPIXMAP_H
