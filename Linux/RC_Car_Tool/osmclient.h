#ifndef OSMCLIENT_H
#define OSMCLIENT_H

#include <QObject>
#include <QString>
#include <QPixmap>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QUrl>
#include <QByteArray>
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QList>

#include "osmtile.h"

/**
 * @brief The OsmClient class
 *
 * See:
 * http://tile.openstreetmap.org/17/70185/39633.png
 * http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
 * http://wiki.openstreetmap.org/wiki/Zoom_levels
 * https://en.wikipedia.org/wiki/Web_Mercator
 * https://switch2osm.org/serving-tiles/manually-building-a-tile-server-14-04/
 *
 */

class OsmClient : public QObject
{
    Q_OBJECT
public:
    explicit OsmClient(QObject *parent = 0);
    bool setCacheDir(QString path);
    bool setTileServerUrl(QString path);
    int getTile(int zoom, int x, int y);
    OsmTile getTileMemory(int zoom, int x, int y, bool &ok);

    int getMaxMemoryTiles() const;
    void setMaxMemoryTiles(int maxMemoryTiles);

signals:
    void tileReady(OsmTile tile);
    void errorGetTile(QString reason);

public slots:

private slots:
    void fileDownloaded(QNetworkReply *pReply);

private:
    QString mCacheDir;
    QString mTileServer;
    QNetworkAccessManager mWebCtrl;
    QList<OsmTile> mMemoryTiles;
    int mMaxMemoryTiles;

    void downloadTile(int zoom, int x, int y);
    void emitTile(OsmTile tile);

};

#endif // OSMCLIENT_H
