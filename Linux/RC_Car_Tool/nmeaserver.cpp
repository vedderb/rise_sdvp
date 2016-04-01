#include "nmeaserver.h"
#include <cstdio>
#include <cmath>
#include <ctime>

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
}

NmeaServer::NmeaServer(QObject *parent) : QObject(parent)
{
    mTcpBroadcast = new TcpBroadcast(this);
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
