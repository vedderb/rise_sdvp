#include "osmclient.h"

OsmClient::OsmClient(QObject *parent) : QObject(parent)
{
    mMaxMemoryTiles = 500;
    mMaxDownloadingTiles = 4;

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

int OsmClient::getTile(int zoom, int x, int y)
{
    int retval = -1;

    if (!mCacheDir.isEmpty()) {
        QString path = mCacheDir + "/" + QString::number(zoom) + "/" +
                QString::number(x) + "/" + QString::number(y) + ".png";
        QFile file;
        file.setFileName(path);
        if (file.exists()) {
            emitTile(OsmTile(QPixmap(path), zoom, x, y));
            retval = 1;
        } else {
            if (!mTileServer.isEmpty()) {
                downloadTile(zoom, x, y);
                retval = 2;
            } else {
                emit errorGetTile("Tile not cached and tile server not set.");
                retval = -2;
            }
        }
    } else {
        if (!mTileServer.isEmpty()) {
            downloadTile(zoom, x, y);
        } else {
            emit errorGetTile("Cache dir and tile server not set.");
            retval = -3;
        }
    }

    return retval;
}

OsmTile OsmClient::getTileMemory(int zoom, int x, int y, bool &ok)
{
    quint64 key = calcKey(zoom, x, y);
    OsmTile t = mMemoryTiles.value(key);

    ok = !t.pixmap().isNull();
    return t;
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

void OsmClient::downloadTile(int zoom, int x, int y)
{
    if (mDownloadingTiles.size() < mMaxDownloadingTiles) {
        quint64 key = calcKey(zoom, x, y);
        if (!mDownloadingTiles.contains(key)) {
            // Only add if this tile is not already downloading
            QString path = mTileServer + "/" + QString::number(zoom) + "/" + QString::number(x) +
                    "/" + QString::number(y) + ".png";
            QNetworkRequest request(path);
            request.setRawHeader("User-Agent", "MyBrowser");
            mWebCtrl.get(request);
            mDownloadingTiles.insert(key, true);
        }
    } else {
        emit errorGetTile("Too many tiles downloading");
    }
}

void OsmClient::emitTile(OsmTile tile)
{
    quint64 key = calcKey(tile.zoom(), tile.x(), tile.y());
    if (!mMemoryTiles.contains(key)) {
        mMemoryTiles.insert(key, tile);
        mMemoryTilesOrder.append(tile);

        // The list is used to keep track of when to delete the oldest tiles
        while (mMemoryTilesOrder.size() > mMaxMemoryTiles) {
            OsmTile t = mMemoryTilesOrder.takeFirst();
            quint64 k = calcKey(t.zoom(), t.x(), t.y());
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
