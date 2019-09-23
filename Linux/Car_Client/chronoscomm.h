/*
    Copyright 2018 Benjamin Vedder	benjamin@vedder.se

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

#ifndef CHRONOSCOMM_H
#define CHRONOSCOMM_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>
#include <vbytearrayle.h>
#include <tcpserversimple.h>

typedef enum {
    COMM_MODE_UNDEFINED = 0,
    COMM_MODE_OBJECT,
    COMM_MODE_SUPERVISOR,
    COMM_MODE_SERVER
} COMM_MODE;

typedef struct {
    uint32_t tRel;
    double x;
    double y;
    double z;
    double heading;
    double long_speed;
    double lat_speed;
    double long_accel;
    double lat_accel;
    double curvature;
} chronos_traj_pt;

typedef struct {
    uint16_t traj_id;
    QString traj_name;
    uint16_t traj_ver;
    QVector<chronos_traj_pt> traj_pts;
    uint32_t object_id;
} chronos_traj;

typedef struct {
    double lat;
    double lon;
    double alt;
    double heading;
    quint32 gps_ms_of_week;
    quint16 gps_week;
} chronos_osem;

typedef struct {
    int armed;
} chronos_ostm;

typedef struct {
    uint32_t gps_ms_of_week;
    uint16_t gps_week;
} chronos_strt;

typedef struct {
    uint32_t gps_ms_of_week;
    uint8_t status;
} chronos_heab;

typedef struct {
    uint32_t gps_ms_of_week;
    double  x;
    double  y;
    double  z;
    double heading;
    double  lon_speed;
    double  lat_speed;
    double  lon_acc;
    double  lat_acc;
    uint8_t  direction; // [ 0 : Forward, 1 : Backward, 2 : Unavailable ]
    uint8_t  status;    // [ 0 : Off, 1 : Init, 2 : Armed, 3 : Disarmed, 4 : Running, 5 : Postrun, 6 : Remote controlled ]
    uint8_t  rdyToArm;  // [ 0 : Not ready, 1 : Ready, 2 : Unavailable ]
    uint8_t  error;     // Each bit represents an error status:
                        // [AbortReq, BrokeGeoFence, PoorPosAccuracy, EngineFault, BatFault, OtherObjError, Vendor, Vendor]
    uint8_t sender_id;
} chronos_monr;

typedef struct {
    uint32_t sync_point;
    uint32_t stop_time;
} chronos_sypm;

typedef struct {
    uint64_t time_est;
} chronos_mtsp;

typedef struct {
    uint8_t status;
} chronos_init_sup;

#define PROTOCOL_VERSION 0

// Chronos messaging

// SYNC Word
#define ISO_SYNC_WORD                   0x7E7E
#define ISO_PART_SYNC_WORD              0x7E

// ISO Message Types
#define ISO_MSG_TRAJ                    0x0001
#define ISO_MSG_OSEM                    0x0002
#define ISO_MSG_OSTM                    0x0003
#define ISO_MSG_STRT                    0x0004
#define ISO_MSG_HEAB                    0x0005
#define ISO_MSG_MONR                    0x0006

#define ISO_MSG_INIT_SUP                0xA102

// ISO Value Types
#define ISO_VALUE_ID_LAT                0x0020
#define ISO_VALUE_ID_LON                0x0021
#define ISO_VALUE_ID_ALT                0x0022
#define ISO_VALUE_ID_DateISO8601        0x0004
#define ISO_VALUE_ID_GPS_WEEK           0x0003
#define ISO_VALUE_ID_GPS_SEC_OF_WEEK    0x0002
#define ISO_VALUE_ID_MAX_WAY_DEV        0x0070
#define ISO_VALUE_ID_MAX_LATERAL_DEV    0x0072
#define ISO_VALUE_ID_MIN_POS_ACCURACY   0x0074
#define ISO_VALUE_ID_STATE_CHANGE_REQ   0x0064
#define ISO_VALUE_ID_DELAYED_START      0x0001
#define ISO_VALUE_ID_REL_TIME           0x0001
#define ISO_VALUE_ID_X_POS              0x0010
#define ISO_VALUE_ID_Y_POS              0x0011
#define ISO_VALUE_ID_Z_POS              0x0012
#define ISO_VALUE_ID_HEADING            0x0030
#define ISO_VALUE_ID_LONG_SPEED         0x0040
#define ISO_VALUE_ID_LAT_SPEED          0x0041
#define ISO_VALUE_ID_LONG_ACC           0x0050
#define ISO_VALUE_ID_LAT_ACC            0x0051
#define ISO_VALUE_ID_CURVATURE          0x0052
#define ISO_VALUE_ID_MONR_STRUCT        0x0080
#define ISO_VALUE_ID_HEAB_STRUCT        0x0090

#define ISO_VALUE_ID_TRAJECTORY_ID      0x0101
#define ISO_VALUE_ID_TRAJECTORY_NAME    0x0102
#define ISO_VALUE_ID_TRAJECTORY_VERSION 0x0103

#define ISO_VALUE_ID_INIT_SUP_STATUS    0x0200
#define AUX_VALUE_ID_OBJECT_ID          0xA000

class ChronosComm : public QObject
{
    Q_OBJECT
public:
    explicit ChronosComm(QObject *parent = nullptr);
    bool startObject(QHostAddress addr = QHostAddress::Any);
    bool startSupervisor(QHostAddress addr = QHostAddress::Any);
    bool connectAsServer(QString address);
    void closeConnection();
    COMM_MODE getCommMode();

    void sendTraj(chronos_traj traj);
    void sendHeab(chronos_heab heab);
    void sendOsem(chronos_osem osem);
    void sendOstm(chronos_ostm ostm);
    void sendStrt(chronos_strt strt);
    void sendMonr(chronos_monr monr);
    void sendInitSup(chronos_init_sup init_sup);

    quint8 transmitterId() const;
    void setTransmitterId(const quint8 &transmitterId);

    static quint32 gpsMsOfWeek();
    static quint32 gpsWeek();
    static quint32 gpsMsOfWeekToUtcToday(quint64 time);

signals:
    void connectionChanged(bool connected, QString address);
    void trajRx(chronos_traj traj);
    void heabRx(chronos_heab heab);
    void osemRx(chronos_osem osem);
    void ostmRx(chronos_ostm ostm);
    void strtRx(chronos_strt strt);
    void monrRx(chronos_monr monr);
    void insupRx(chronos_init_sup init_sup);

public slots:

private slots:
    void tcpRx(QByteArray data);
    void tcpConnectionChanged(bool connected, QString address);
    void readPendingDatagrams();

    void tcpInputConnected();
    void tcpInputDisconnected();
    void tcpInputDataAvailable();
    void tcpInputError(QAbstractSocket::SocketError socketError);

private:
    TcpServerSimple *mTcpServer;
    QTcpSocket *mTcpSocket;
    QUdpSocket *mUdpSocket;
    QHostAddress mUdpHostAddress;
    quint16 mUdpPort;
    quint8 mTransmitterId;
    quint8 mChronosSeqNum;
    COMM_MODE mCommMode;

    int mTcpState;
    quint16 mTcpType;
    quint32 mTcpLen;
    quint16 mTcpChecksum;
    VByteArrayLe mTcpData;

    void mkChronosHeader(VByteArrayLe &vb,
                         quint8 transmitter_id,
                         quint8 sequence_num, // per object sequence num
                         bool ack_req,
                         quint8 protocol_ver, // 7 bits
                         quint16 message_id);
    void appendChronosChecksum(VByteArrayLe &vb);
    bool decodeMsg(quint16 type, quint32 len, QByteArray payload, uint8_t sender_id);
    void sendData(QByteArray data, bool isUdp);

};

#endif // CHRONOSCOMM_H
