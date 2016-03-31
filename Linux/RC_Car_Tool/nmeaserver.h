#ifndef NMEASERVER_H
#define NMEASERVER_H

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QFile>

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
    bool sendNmeaCustom(QString msg);
    bool logToFile(QString file);
    void logStop();

signals:

public slots:

private slots:
    void newTcpConnection();
    void tcpDisconnected();
    void tcpDataAvailable();

private:
    QTcpServer *mTcpServer;
    QTcpSocket *mTcpSocket;
    bool mTcpIsConnected;
    QFile mLog;

};

#endif // NMEASERVER_H
