/*
    Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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

#ifndef NMEASERVER_H
#define NMEASERVER_H

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QFile>
#include "tcpbroadcast.h"

class NmeaServer : public QObject
{
    Q_OBJECT
public:
    typedef struct {
        double lat;
        double lon;
        double height;
        double t_tow;
        int n_sat;
        int fix_type;
        double h_dop;
        double diff_age;
    } nmea_gga_info_t;

    typedef struct {
        double lat;
        double lon;
        double t_tow;
        quint16 t_wn;
        double vel_x;
        double vel_y;
        double vel_z;
    } nmea_rmc_info_t;

    explicit NmeaServer(QObject *parent = 0);
    ~NmeaServer();
    bool startTcpServer(int port);
    bool sendNmeaGga(nmea_gga_info_t &nmea);
    bool sendNmeaZda(quint16 wn, double tow);
    bool sendNmeaRmc(nmea_rmc_info_t &nmea);
    bool sendNmeaRaw(QString msg);
    bool logToFile(QString file);
    void logStop();
    bool connectClientTcp(QString server, int port = 80);
    bool isClientTcpConnected();
    void disconnectClientTcp();

    static int decodeNmeaGGA(QByteArray data, nmea_gga_info_t &gga);

signals:
    void clientGgaRx(int fields, NmeaServer::nmea_gga_info_t gga);

private slots:
    void tcpInputConnected();
    void tcpInputDisconnected();
    void tcpInputDataAvailable();
    void tcpInputError(QAbstractSocket::SocketError socketError);

private:
    TcpBroadcast *mTcpBroadcast;
    QTcpSocket *mTcpClient;
    QFile mLog;

};

#endif // NMEASERVER_H
