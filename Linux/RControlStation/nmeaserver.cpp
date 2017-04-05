/*
    Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#include "nmeaserver.h"
#include <cstdio>
#include <cmath>
#include <ctime>
#include <cstring>
#include <locale.h>
#include <QMessageBox>

namespace
{
#define NMEA_SUFFIX_LEN 6
#define MINUTES(X) fabs(60 * ((X) - ((qint16)(X))))
#define MS2KNOTTS(x,y,z) sqrt((x)*(x) + (y)*(y) + (z)*(z)) * 1.94385

/** Calculate and append the checksum of an NMEA sentence.
 * Calculates the bitwise XOR of the characters in a string until the end of
 * the string or a `*` is encountered. If the first character is `$` then it
 * is skipped.
 *
 * \param s A null-terminated NMEA sentence, up to and optionally
 * including the '*'
 *
 * \param size Length of the buffer.
 *
 */
static void nmea_append_checksum(char *s, size_t size) {
    quint8 sum = 0;
    char *p = s;

    // '$' header not included in checksum calculation
    if (*p == '$') {
        p++;
    }

    // '*'  not included in checksum calculation
    while (*p != '*' && *p && p + NMEA_SUFFIX_LEN < s + size) {
        sum ^= *p;
        p++;
    }

    sprintf(p, "*%02X\r\n", sum);
}

static double nmea_parse_val(char *str) {
    int ind = -1;
    int len = strlen(str);
    double retval = 0.0;

    for (int i = 2;i < len;i++) {
        if (str[i] == '.') {
            ind = i - 2;
            break;
        }
    }

    if (ind >= 0) {
        char a[len + 1];
        memcpy(a, str, ind);
        a[ind] = ' ';
        memcpy(a + ind + 1, str + ind, len - ind);

        double l1, l2;
        if (sscanf(a, "%lf %lf", &l1, &l2) == 2) {
            retval = l1 + l2 / 60.0;
        }
    }

    return retval;
}

#ifndef Q_OS_UNIX
// see http://stackoverflow.com/questions/8512958/is-there-a-windows-variant-of-strsep
static char* mystrsep(char** stringp, const char* delim) {
    char* start = *stringp;
    char* p;

    p = (start != NULL) ? strpbrk(start, delim) : NULL;

    if (p == NULL) {
        *stringp = NULL;
    } else {
        *p = '\0';
        *stringp = p + 1;
    }

    return start;
}
#endif
}

NmeaServer::NmeaServer(QObject *parent) : QObject(parent)
{
    mTcpBroadcast = new TcpBroadcast(this);
    mTcpClient = new QTcpSocket(this);

    connect(mTcpClient, SIGNAL(readyRead()), this, SLOT(tcpInputDataAvailable()));
    connect(mTcpClient, SIGNAL(connected()), this, SLOT(tcpInputConnected()));
    connect(mTcpClient, SIGNAL(disconnected()),
            this, SLOT(tcpInputDisconnected()));
    connect(mTcpClient, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(tcpInputError(QAbstractSocket::SocketError)));
}

NmeaServer::~NmeaServer()
{
    logStop();
}

bool NmeaServer::startTcpServer(int port)
{
    if (!mTcpBroadcast->startTcpServer(port)) {
        qWarning() << "Unable to start TCP server: " << mTcpBroadcast->getLastError();
        return false;
    }

    return true;
}

bool NmeaServer::sendNmeaGga(NmeaServer::nmea_gga_info_t &nmea)
{
    qint16 lat_deg = nmea.lat;
    double lat_min = MINUTES(nmea.lat);
    qint16 lon_deg = nmea.lon;
    double lon_min = MINUTES(nmea.lon);
    lat_deg = abs(lat_deg);
    lon_deg = abs(lon_deg);

    char lat_dir = nmea.lat < 0.0 ? 'S' : 'N';
    char lon_dir = nmea.lon < 0.0 ? 'W' : 'E';

    int t = (int)nmea.t_tow;
    int hour = (t / 3600) % 24;
    int min = (t / 60) % 60;
    int sec = t % 60;
    int frac_s = fmod(nmea.t_tow, 1.0) * 100.0;

    QString nmea_str;
    nmea_str.sprintf("$GPGGA,%02d%02d%02d.%02d,"
                     "%02d%010.7f,%c,%03d%010.7f,%c,"
                     "%01d,%02d,%.1f,%.2f,M,,M,,",
                     hour, min, sec, frac_s,
                     lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir,
                     nmea.fix_type, nmea.n_sat, nmea.h_dop, nmea.height);
    nmea_str.append("*    ");
    QByteArray nmea_bytes = nmea_str.toLocal8Bit();
    nmea_append_checksum(nmea_bytes.data(), nmea_bytes.size());
    nmea_bytes.remove(nmea_bytes.size() - 1, 1);

    //qDebug() << nmea_bytes;

    if (mLog.isOpen()) {
        mLog.write(nmea_bytes);
    }

    mTcpBroadcast->broadcastData(nmea_bytes);

    return true;
}

bool NmeaServer::sendNmeaZda(quint16 wn, double tow)
{
    struct tm time_tm;
    time_tm.tm_sec = 0;
    time_tm.tm_hour = 1;
    time_tm.tm_min = 0;
    time_tm.tm_mday = 6;
    time_tm.tm_year = 1980 - 1900;
    time_tm.tm_mon = 0;

    time_t time = mktime(&time_tm);
    time += (time_t)wn * 24 * 7 * 60 * 60 + (time_t)tow;
    time_tm = *localtime(&time);

    int year = time_tm.tm_year + 1900;
    int month = time_tm.tm_mon + 1;
    int day = time_tm.tm_mday;
    int hour = time_tm.tm_hour;
    int min = time_tm.tm_min;
    int sec = time_tm.tm_sec;
    int frac_s = fmod(tow, 1.0) * 100.0;

    QString nmea_str;
    nmea_str.sprintf("$GPZDA,%02d%02d%02d.%02d,%02d,%02d,%04d,00,00",
                     hour, min, sec, frac_s, day, month, year);
    nmea_str.append("*    ");
    QByteArray nmea_bytes = nmea_str.toLocal8Bit();
    nmea_append_checksum(nmea_bytes.data(), nmea_bytes.size());
    nmea_bytes.remove(nmea_bytes.size() - 1, 1);

    //qDebug() << nmea_bytes;

    if (mLog.isOpen()) {
        mLog.write(nmea_bytes);
    }

    mTcpBroadcast->broadcastData(nmea_bytes);

    return true;
}

bool NmeaServer::sendNmeaRmc(NmeaServer::nmea_rmc_info_t &nmea)
{
    qint16 lat_deg = nmea.lat;
    double lat_min = MINUTES(nmea.lat);
    qint16 lon_deg = nmea.lon;
    double lon_min = MINUTES(nmea.lon);
    lat_deg = abs(lat_deg);
    lon_deg = abs(lon_deg);

    char lat_dir = nmea.lat < 0.0 ? 'S' : 'N';
    char lon_dir = nmea.lon < 0.0 ? 'W' : 'E';

    struct tm time_tm;
    time_tm.tm_sec = 0;
    time_tm.tm_hour = 1;
    time_tm.tm_min = 0;
    time_tm.tm_mday = 6;
    time_tm.tm_year = 1980 - 1900;
    time_tm.tm_mon = 0;

    time_t time = mktime(&time_tm);
    time += (time_t)nmea.t_wn * 24 * 7 * 60 * 60 + (time_t)nmea.t_tow;
    time_tm = *localtime(&time);

    int year = time_tm.tm_year + 1900;
    int month = time_tm.tm_mon + 1;
    int day = time_tm.tm_mday;
    int hour = time_tm.tm_hour;
    int min = time_tm.tm_min;
    int sec = time_tm.tm_sec;
    int frac_s = fmod(nmea.t_tow, 1.0) * 100.0;

    double velocity;
    double course = atan2(nmea.vel_y, nmea.vel_x);
    velocity = MS2KNOTTS(nmea.vel_x, nmea.vel_y, nmea.vel_z);

    QString nmea_str;
    nmea_str.sprintf("$GPRMC,%02d%02d%02d.%02d,A,"
                     "%02d%010.7f,%c,%03d%010.7f,%c,"
                     "%06.2f,%05.1f,"
                     "%02d%02d%02d,"
                     ",",
                     hour, min, sec, frac_s,
                     lat_deg, lat_min, lat_dir, lon_deg, lon_min, lon_dir,
                     velocity, course * (180.0 / M_PI),
                     day, month, year % 100);
    nmea_str.append("*    ");
    QByteArray nmea_bytes = nmea_str.toLocal8Bit();
    nmea_append_checksum(nmea_bytes.data(), nmea_bytes.size());
    nmea_bytes.remove(nmea_bytes.size() - 1, 1);

     //qDebug() << nmea_bytes;

    if (mLog.isOpen()) {
        mLog.write(nmea_bytes);
    }

    mTcpBroadcast->broadcastData(nmea_bytes);

    return true;
}

bool NmeaServer::sendNmeaRaw(QString msg)
{
    QByteArray nmea_bytes = msg.toLocal8Bit();

    if (mLog.isOpen()) {
        mLog.write(nmea_bytes);
    }

    mTcpBroadcast->broadcastData(nmea_bytes);

    return true;
}

bool NmeaServer::logToFile(QString file)
{
    if (mLog.isOpen()) {
        mLog.close();
    }

    mLog.setFileName(file);
    return mLog.open(QIODevice::ReadWrite | QIODevice::Truncate);
}

void NmeaServer::logStop()
{
    if (mLog.isOpen()) {
        qDebug() << "Closing NMEA log:" << mLog.fileName();
        mLog.close();
    } else {
        qDebug() << "Log not open";
    }
}

bool NmeaServer::connectClientTcp(QString server, int port)
{
    mTcpClient->abort();
    mTcpClient->connectToHost(server, port);

    return true;
}

bool NmeaServer::isClientTcpConnected()
{
    return mTcpClient->isOpen();
}

void NmeaServer::disconnectClientTcp()
{
    mTcpClient->close();
}

/**
 * @brief NmeaServer::decodeNmeaGGA
 * Decode NMEA GGA message.
 *
 * @param data
 * NMEA data.
 *
 * @param gga
 * GGA struct to fill.
 *
 * @return
 * -1: Type is not GGA
 * >= 0: Number of decoded fields.
 */
int NmeaServer::decodeNmeaGGA(QByteArray data, NmeaServer::nmea_gga_info_t &gga)
{
    static char nmea_str[1024];
    int ms = -1;
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    int fix_type = 0;
    int sats = 0;
    double hdop = 0.0;
    double diff_age = -1.0;

    int dec_fields = 0;

    setlocale(LC_NUMERIC, "C");

    bool found = false;
    const char *str = data.constData();
    int len = strlen(str);

    for (int i = 0;i < 10;i++) {
        if ((i + 5) >= len) {
            break;
        }

        if (    str[i] == 'G' &&
                str[i + 1] == 'G' &&
                str[i + 2] == 'A' &&
                str[i + 3] == ',') {
            found = true;
            strcpy(nmea_str, str + i + 4);
            break;
        }
    }

    if (found) {
        char *gga, *str;
        int ind = 0;

        str = nmea_str;
#ifdef Q_OS_UNIX
        gga = strsep(&str, ",");
#else
        gga = mystrsep(&str, ",");
#endif

        while (gga != 0) {
            switch (ind) {
            case 0: {
                // Time
                int h, m, s, ds;
                dec_fields++;

                if (sscanf(gga, "%02d%02d%02d.%d", &h, &m, &s, &ds) == 4) {
                    ms = h * 60 * 60 * 1000;
                    ms += m * 60 * 1000;
                    ms += s * 1000;
                    ms += ds * 10;
                } else {
                    ms = -1;
                }
            } break;

            case 1: {
                // Latitude
                dec_fields++;
                lat = nmea_parse_val(gga);
            } break;

            case 2:
                // Latitude direction
                dec_fields++;
                if (*gga == 'S' || *gga == 's') {
                    lat = -lat;
                }
                break;

            case 3: {
                // Longitude
                dec_fields++;
                lon = nmea_parse_val(gga);
            } break;

            case 4:
                // Longitude direction
                dec_fields++;
                if (*gga == 'W' || *gga == 'w') {
                    lon = -lon;
                }
                break;

            case 5:
                // Fix type
                dec_fields++;
                if (sscanf(gga, "%d", &fix_type) != 1) {
                    fix_type = 0;
                }
                break;

            case 6:
                // Sattelites
                dec_fields++;
                if (sscanf(gga, "%d", &sats) != 1) {
                    sats = 0;
                }
                break;

            case 7:
                // hdop
                dec_fields++;
                if (sscanf(gga, "%lf", &hdop) != 1) {
                    hdop = 0.0;
                }
                break;

            case 8:
                // Altitude
                dec_fields++;
                if (sscanf(gga, "%lf", &height) != 1) {
                    height = 0.0;
                }
                break;

            case 10: {
                // Altitude 2
                double h2 = 0.0;
                dec_fields++;
                if (sscanf(gga, "%lf", &h2) != 1) {
                    h2 = 0.0;
                }

                height += h2;
            } break;

            case 12: {
                // Correction age
                dec_fields++;
                if (sscanf(gga, "%lf", &diff_age) != 1) {
                    diff_age = -1.0;
                }
            } break;

            default:
                break;
            }

#ifdef Q_OS_UNIX
            gga = strsep(&str, ",");
#else
            gga = mystrsep(&str, ",");
#endif
            ind++;
        }
    } else {
        dec_fields = -1;
    }

    gga.lat = lat;
    gga.lon = lon;
    gga.height = height;
    gga.fix_type = fix_type;
    gga.n_sat = sats;
    gga.t_tow = ms;
    gga.h_dop = hdop;
    gga.diff_age = diff_age;

    return dec_fields;
}

void NmeaServer::tcpInputConnected()
{
    qDebug() << "NMEA TCP connected";
}

void NmeaServer::tcpInputDisconnected()
{
    qDebug() << "NMEA TCP disconnected";
}

void NmeaServer::tcpInputDataAvailable()
{
    QByteArray data =  mTcpClient->readAll();
    QTextStream in(data);

    while(!in.atEnd()) {
        QString line = in.readLine();
        nmea_gga_info_t gga;
        int res = decodeNmeaGGA(line.toLocal8Bit(), gga);
        emit clientGgaRx(res, gga);
    }
}

void NmeaServer::tcpInputError(QAbstractSocket::SocketError socketError)
{
    (void)socketError;

    QString errorStr = mTcpClient->errorString();
    qWarning() << "NMEA TcpError:" << errorStr;

    // TODO: casting parent to QWidget might not always work. This is however
    // required for making the dialog modal properly.
    QMessageBox::warning((QWidget*)this->parent(), "NMEA TCP Error", errorStr);

    mTcpClient->close();
}
