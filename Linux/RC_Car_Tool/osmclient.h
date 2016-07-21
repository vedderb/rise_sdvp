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
#include <QHash>
#include <QList>

#include "osmtile.h"

/**
 * @brief The OsmClient class
 *
 * See:
 *
 * Tile example
 * http://tile.openstreetmap.org/17/70185/39633.png
 *
 * Tile names and coordinates
 * http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
 *
 * Zoom level description
 * http://wiki.openstreetmap.org/wiki/Zoom_levels
 *
 * Markartor projection
 * https://en.wikipedia.org/wiki/Web_Mercator
 *
 * Ubuntu tile server
 * https://switch2osm.org/serving-tiles/manually-building-a-tile-server-14-04/
 * https://switch2osm.org/loading-osm-data/
 * http://stackoverflow.com/questions/16834983/i-am-trying-to-config-my-own-map-server-with-mapnik-mod-tile-and-apache-no-tiles
 *
 * Carto style
 * https://github.com/gravitystorm/openstreetmap-carto
 *
 * Zoom levels
 * http://wiki.openstreetmap.org/wiki/User:SomeoneElse/Tileserver_zoom_levels
 *
 * Another carto style
 * https://github.com/SomeoneElseOSM/openstreetmap-carto-AJT
 *
 * Map updates
 * http://wiki.openstreetmap.org/wiki/Minutely_Mapnik
 *
 * HD Tiles:
 * https://lists.openstreetmap.org/pipermail/tile-serving/2014-July/001144.html
 */

class OsmClient : public QObject
{
    Q_OBJECT
public:
    explicit OsmClient(QObject *parent = 0);
    bool setCacheDir(QString path);
    bool setTileServerUrl(QString path);
    OsmTile getTile(int zoom, int x, int y, int &res);
    int downloadTile(int zoom, int x, int y);
    bool downloadQueueFull();
    void clearCache();

    int getMaxMemoryTiles() const;
    void setMaxMemoryTiles(int maxMemoryTiles);

    int getMaxDownloadingTiles() const;
    void setMaxDownloadingTiles(int maxDownloadingTiles);

    int getHddTilesLoaded() const;
    int getTilesDownloaded() const;
    int getMemoryTilesNow() const;
    int getRamTilesLoaded() const;

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
    QHash<quint64, OsmTile> mMemoryTiles;
    QList<quint64> mMemoryTilesOrder;
    QHash<quint64, bool> mDownloadingTiles;
    QHash<quint64, bool> mDownloadErrorTiles;
    QList<QPixmap> mStatusPixmaps;

    int mMaxMemoryTiles;
    int mMaxDownloadingTiles;
    int mHddTilesLoaded;
    int mTilesDownloaded;
    int mRamTilesLoaded;

    void emitTile(OsmTile tile);
    quint64 calcKey(int zoom, int x, int y);
    void storeTileMemory(quint64 key, const OsmTile &tile);
    const QPixmap& getStatusPixmap(quint64 key);

};

#endif // OSMCLIENT_H
