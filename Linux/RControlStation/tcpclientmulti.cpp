#include "tcpclientmulti.h"

TcpClientMulti::TcpClientMulti(QObject *parent) : QObject(parent)
{

}

TcpClientMulti::~TcpClientMulti()
{
    disconnectAll();
}

void TcpClientMulti::addConnection(QString ip, int port)
{
    mTcpConns.append(new TcpConn(ip, port, this));
}

bool TcpClientMulti::isAnyConnected()
{
    for (auto c: mTcpConns) {
        if (c->isTcpConnected()) {
            return true;
        }
    }

    return false;
}

void TcpClientMulti::disconnectAll()
{
    for (auto c: mTcpConns) {
        delete c;
    }
    mTcpConns.clear();
}

void TcpClientMulti::sendAll(QByteArray data)
{
    for (auto c: mTcpConns) {
        c->sendData(data);
    }
}
