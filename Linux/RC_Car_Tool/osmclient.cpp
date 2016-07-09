#include "osmclient.h"
#include <QDebug>

OsmClient::OsmClient(QObject *parent) : QObject(parent)
{
    mMaxMemoryTiles = 300;
    mMaxDownloadingTiles = 3;

    connect(&mWebCtrl, SIGNAL(finished(QNetworkReply*)),
            this, SLOT(fileDownloaded(QNetworkReply*)));
}

bool OsmClient::setCacheDir(QString path)
{
    QDir().mkpath(path);
    QFileInfo file;
    file.setFile(path);

    if (file.isDir()) {
        mCacheDir = path;
        return true;
    } else {
        qWarning() << "Invalid cache directory provided.";
        return false;
    }
}

bool OsmClient::setTileServerUrl(QString path)
{
    QUrl url(path);

    if (url.isValid()) {
        mTileServer = path;
        return true;
    } else {
        qWarning() << "Invalid tile server url provided:" << url.errorString();
        return false;
    }
}

/**
 * @brief OsmClient::getTile
 * Get a openstreetmap tile
 *
 * @param zoom
 * zoom level
 *
 * @param x
 * x index
 *
 * @param y
 * y index
 *
 * @param res
 * Reference to store the result in.
 *
 * Result greater than 0 means that a valid tile is returned. Negative results
 * are errors.
 *
 * -1: Unknown error.
 * 0: Tile not cached in memory or on disk.
 * 1: Tile read from memory.
 * 2: Tile read from disk.
 *
 * @return
 * The tile if res > 0, otherwise a tile with a null pixmap.
 */
OsmTile OsmClient::getTile(int zoom, int x, int y, int &res)
{
    res = -1;

    quint64 key = calcKey(zoom, x, y);
    OsmTile t = mMemoryTiles.value(key);

    if (!t.pixmap().isNull()) {
        res = 1;
    } else if (!mCacheDir.isEmpty()) {
        QString path = mCacheDir + "/" + QString::number(zoom) + "/" +
                QString::number(x) + "/" + QString::number(y) + ".png";
        QFile file;
        file.setFileName(path);
        if (file.exists()) {
            res = 2;
            t = OsmTile(QPixmap(path), zoom, x, y);
        }
    }

    return t;
}

/**
 * @brief OsmClient::downloadTile
 * Start a tile dowmload.
 *
 * @param zoom
 * zoom level
 *
 * @param x
 * x index
 *
 * @param y
 * y index
 *
 * @return
 * -3: Tile server not set.
 * -2: Too many tiles downloading.
 * -1: Unknown error.
 * 1: Tile download started.
 *
 */
int OsmClient::downloadTile(int zoom, int x, int y)
{
    int retval = -1;

    if (!mTileServer.isEmpty()) {
        if (mDownloadingTiles.size() < mMaxDownloadingTiles) {
            quint64 key = calcKey(zoom, x, y);
            if (!mDownloadingTiles.contains(key)) {
                // Only add if this tile is not already downloading
                QString path = mTileServer + "/" + QString::number(zoom) +
                        "/" + QString::number(x) + "/" + QString::number(y) + ".png";
                QNetworkRequest request(path);
                request.setRawHeader("User-Agent", "MyBrowser");
                mWebCtrl.get(request);
                mDownloadingTiles.insert(key, true);
            }
            retval = 1;
        } else {
            emit errorGetTile("Too many tiles downloading.");
            retval = -2;
        }
    } else {
        emit errorGetTile("Tile server not set.");
        retval = -3;
    }

    return retval;
}

bool OsmClient::downloadQueueFull()
{
    return mDownloadingTiles.size() >= mMaxDownloadingTiles;
}

void OsmClient::fileDownloaded(QNetworkReply *pReply)
{
    QString path = pReply->url().toString();
    path = path.left(path.length() - 4);
    int ind = path.lastIndexOf("/");
    int y = path.mid(ind + 1).toInt();
    path = path.left(ind);
    ind = path.lastIndexOf("/");
    int x = path.mid(ind + 1).toInt();
    path = path.left(ind);
    ind = path.lastIndexOf("/");
    int zoom = path.mid(ind + 1).toInt();

    mDownloadingTiles.remove(calcKey(zoom, x, y));

    if (pReply->error() == QNetworkReply::NoError) {
        QPixmap pm;
        QByteArray data = pReply->readAll();
        pm.loadFromData(data, "PNG");

        // Try to cache tile
        if (!mCacheDir.isEmpty()) {
            QString path = mCacheDir + "/" + QString::number(zoom) + "/" +
                    QString::number(x) + "/" + QString::number(y) + ".png";
            QFile file;
            file.setFileName(path);
            if (!file.exists()) {
                QDir().mkpath(mCacheDir + "/" + QString::number(zoom) + "/" + QString::number(x));
                if (file.open(QIODevice::ReadWrite)) {
                     file.write(data);
                     file.close();
                } else {
                    emit errorGetTile("Cache error: " + file.errorString());
                }
            }
        }

        emitTile(OsmTile(pm, zoom, x, y));
    } else {
        emit errorGetTile("Download error: " + pReply->errorString());
    }
}

int OsmClient::getMaxDownloadingTiles() const
{
    return mMaxDownloadingTiles;
}

void OsmClient::setMaxDownloadingTiles(int maxDownloadingTiles)
{
    mMaxDownloadingTiles = maxDownloadingTiles;
}

int OsmClient::getMaxMemoryTiles() const
{
    return mMaxMemoryTiles;
}

void OsmClient::setMaxMemoryTiles(int maxMemoryTiles)
{
    mMaxMemoryTiles = maxMemoryTiles;
}

void OsmClient::emitTile(OsmTile tile)
{
    quint64 key = calcKey(tile.zoom(), tile.x(), tile.y());
    if (!mMemoryTiles.contains(key)) {
        mMemoryTiles.insert(key, tile);
        mMemoryTilesOrder.append(key);

        // The list is used to keep track of when to delete the oldest tiles
        while (mMemoryTilesOrder.size() > mMaxMemoryTiles) {
            quint64 k = mMemoryTilesOrder.takeFirst();
            int res = mMemoryTiles.remove(k);

            if (res != 1) {
                qDebug() << res << mMemoryTiles.size() << k;
            }
        }
    }

    emit tileReady(tile);
}

quint64 OsmClient::calcKey(int zoom, int x, int y)
{
    return ((qint64)zoom << 40) | ((qint64)x << 20) | (qint64)y;
}
