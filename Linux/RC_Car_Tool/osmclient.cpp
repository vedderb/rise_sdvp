#include "osmclient.h"

OsmClient::OsmClient(QObject *parent) : QObject(parent)
{
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
        return false;
    }
}

bool OsmClient::setTileServerUrl(QString path)
{
    QUrl url;
    url.setHost(path);

    if (url.isValid()) {
        mTileServer = path;
        return true;
    } else {
        return false;
    }
}

void OsmClient::getTile(int zoom, int x, int y)
{
    if (!mCacheDir.isEmpty()) {
        QString path = mCacheDir + "/" + zoom + "/" + x + "/" + y + ".png";
        QFile file;
        file.setFileName(path);
        if (file.exists()) {
            emit tileReady(OsmTile(QPixmap(path), zoom, x, y));
        } else {
            if (!mTileServer.isEmpty()) {
                downloadTile(zoom, x, y);
            } else {
                emit errorGetTile("Tile not cached and tile server not set.");
            }
        }
    } else {
        if (!mTileServer.isEmpty()) {
            downloadTile(zoom, x, y);
        } else {
            emit errorGetTile("Cache dir and tile server not set.");
        }
    }
}

void OsmClient::fileDownloaded(QNetworkReply *pReply)
{
    if (pReply->error() == QNetworkReply::NoError) {
        QString path = pReply->url().path();
        path = path.left(path.length() - 4);
        int ind = path.lastIndexOf("/");
        int y = path.mid(ind).toInt();
        path = path.left(ind);
        ind = path.lastIndexOf("/");
        int x = path.mid(ind).toInt();
        path = path.left(ind);
        ind = path.lastIndexOf("/");
        int zoom = path.mid(ind).toInt();

        QPixmap pm;
        QByteArray data = pReply->readAll();
        pm.loadFromData(data, "PNG");

        // Try to cache tile
        if (!mCacheDir.isEmpty()) {
            QString path = mCacheDir + "/" + zoom + "/" + x + "/" + y + ".png";
            QFile file;
            file.setFileName(path);
            if (!file.exists()) {
                QDir().mkpath(mCacheDir + "/" + zoom + "/" + x);
                if (file.open(QIODevice::ReadWrite)) {
                     file.write(data);
                     file.close();
                } else {
                    emit errorGetTile("Cache error: " + file.errorString());
                }
            }
        }

        emit tileReady(OsmTile(pm, zoom, x, y));
    } else {
        emit errorGetTile("Download error: " + pReply->errorString());
    }
}

void OsmClient::downloadTile(int zoom, int x, int y)
{
    QString path = mTileServer + "/" + zoom + "/" + x + "/" + y + ".png";
    QNetworkRequest request(path);
    mWebCtrl.get(request);
}

