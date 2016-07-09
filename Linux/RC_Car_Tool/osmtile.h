#ifndef OSMTILE_H
#define OSMTILE_H

#include <QPixmap>

class OsmTile
{
public:
    OsmTile(QPixmap pxm, int zoom = 0, int x = 0, int y = 0);
    OsmTile(int zoom = 0, int x = 0, int y = 0);
    void setZXY(int zoom, int x, int y);

    int zoom() const;
    void setZoom(double zoom);

    int x() const;
    void setX(int x);

    int y() const;
    void setY(int y);

    QPixmap pixmap() const;
    void setPixmap(const QPixmap &pixmap);

    double getNorth();
    double getSouth();
    double getWest();
    double getEast();

    double getWidthTop();

    // Static functions
    static int long2tilex(double lon, int z);
    static int lat2tiley(double lat, int z);
    static double tilex2long(int x, int z);
    static double tiley2lat(int y, int z);
    static double lat2width(double lat, int zoom);

    // Operators
    bool operator==(const OsmTile& tile);
    bool operator!=(const OsmTile& tile);

private:
    QPixmap mPixmap;
    int mZoom;
    int mX;
    int mY;

};

#endif // OSMTILE_H
