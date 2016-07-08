#include "osmclient.h"

OsmClient::OsmClient(QObject *parent) : QObject(parent)
{
    mMaxMemoryTiles = 100000;

    connect(&mWebCtrl, SIGNAL(finished(QNetworkReply*)),
            this, SLOT(fileDownloaded(QNetworkReply*)));
}

bool OsmClient::setCacheDir(QString path)
{
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
    int ind = mMemoryTiles.indexOf(OsmTile(zoom, x, y));

    if (ind >= 0) {
        ok = true;
        return mMemoryTiles[ind];
    } else {
        ok = false;
        return OsmTile();
    }
}

void OsmClient::fileDownloaded(QNetworkReply *pReply)
{
    if (pReply->error() == QNetworkReply::NoError) {
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
    QString path = mTileServer + "/" + QString::number(zoom) + "/" + QString::number(x) +
            "/" + QString::number(y) + ".png";
    QNetworkRequest request(path);
    request.setRawHeader("User-Agent", "MyBrowser");
    mWebCtrl.get(request);
}

void OsmClient::emitTile(OsmTile tile)
{
    mMemoryTiles.append(tile);
    while (mMemoryTiles.size() > mMaxMemoryTiles) {
        mMemoryTiles.removeFirst();
    }

    emit tileReady(tile);
}

