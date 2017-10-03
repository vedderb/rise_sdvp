#include "rtrange.h"
#include "ui_rtrange.h"
#include "utility.h"

#include <QDebug>
#include <cmath>

namespace
{
int32_t get_I3(uint8_t *msg, int *ind) {
    int32_t res =	((uint32_t) msg[*ind + 2]) << 16 |
                    ((uint32_t) msg[*ind + 1]) << 8 |
                    ((uint32_t) msg[*ind + 0]);
    *ind += 3;

    if (res & 0b00000000100000000000000000000000) {
        res |= 0xFF000000;
    }

    return res;
}

float get_R4(uint8_t *msg, int *ind) {
    uint32_t res =	((uint32_t) msg[*ind + 3]) << 24 |
                    ((uint32_t) msg[*ind + 2]) << 16 |
                    ((uint32_t) msg[*ind + 1]) << 8 |
                    ((uint32_t) msg[*ind + 0]);
    *ind += 4;

    union asd {
        float f;
        uint32_t i;
    } x;

    x.i = res;

    return x.f;
}

double get_R8(uint8_t *msg, int *ind) {
    uint64_t res =	((uint64_t) msg[*ind + 7]) << 56 |
                    ((uint64_t) msg[*ind + 6]) << 48 |
                    ((uint64_t) msg[*ind + 5]) << 40 |
                    ((uint64_t) msg[*ind + 4]) << 32 |
                    ((uint64_t) msg[*ind + 3]) << 24 |
                    ((uint64_t) msg[*ind + 2]) << 16 |
                    ((uint64_t) msg[*ind + 1]) << 8 |
                    ((uint64_t) msg[*ind + 0]);
    *ind += 8;

    union asd {
        double f;
        uint64_t i;
    } x;

    x.i = res;

    return x.f;
}

void put_I3(uint8_t *msg, int *ind, int32_t data) {
    msg[(*ind)++] = data;
    msg[(*ind)++] = data >> 8;
    msg[(*ind)++] = data >> 16;
}

void put_R4(uint8_t *msg, int *ind, float data) {
    union asd {
        float f;
        uint32_t i;
    } x;

    x.f = data;

    msg[(*ind)++] = x.i;
    msg[(*ind)++] = x.i >> 8;
    msg[(*ind)++] = x.i >> 16;
    msg[(*ind)++] = x.i >> 24;
}

void put_R8(uint8_t *msg, int *ind, double data) {
    union asd {
        double f;
        uint64_t i;
    } x;

    x.f = data;

    msg[(*ind)++] = x.i;
    msg[(*ind)++] = x.i >> 8;
    msg[(*ind)++] = x.i >> 16;
    msg[(*ind)++] = x.i >> 24;
    msg[(*ind)++] = x.i >> 32;
    msg[(*ind)++] = x.i >> 40;
    msg[(*ind)++] = x.i >> 48;
    msg[(*ind)++] = x.i >> 56;
}
}

RtRange::RtRange(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RtRange)
{
    ui->setupUi(this);
    mUdpSocket = new QUdpSocket(this);
    mPacketCounter = 0;
    mMap = 0;
    mMapCnt = 0;

    connect(mUdpSocket, SIGNAL(readyRead()),
            this, SLOT(readPendingDatagrams()));
}

RtRange::~RtRange()
{
    delete ui;
}

void RtRange::setMap(MapWidget *map)
{
    mMap = map;
}

void RtRange::sendNcom(double *illh,
                       double px,
                       double py,
                       double pz,
                       double heading,
                       double vel)
{
    double llh[3], xyz[3];

    xyz[0] = px;
    xyz[1] = py;
    xyz[2] = pz;

    utility::enuToLlh(illh, xyz, llh);
    double head = heading + (M_PI / 2.0);
    utility::norm_angle_rad(&head);

    double velN = vel * cos(head);
    double velE = vel * sin(head);
    double velD = 0.0;

    unsigned char packet[72];
    memset(packet, 0, sizeof(packet));

    int ind = 23;
    put_R8(packet, &ind, llh[0] * (M_PI / 180.0));
    put_R8(packet, &ind, llh[1] * (M_PI / 180.0));
    put_R4(packet, &ind, llh[2] * (M_PI / 180.0));
    ind = 43;
    put_I3(packet, &ind, (int32_t)(velN * 1e4));
    put_I3(packet, &ind, (int32_t)(velE * 1e4));
    put_I3(packet, &ind, (int32_t)(velD * 1e4));
    put_I3(packet, &ind, (int32_t)(head * 1e6));

    mUdpSocket->writeDatagram((char*)packet, sizeof(packet), QHostAddress::Broadcast, 3000);
}

void RtRange::readPendingDatagrams()
{
    while (mUdpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(mUdpSocket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        mUdpSocket->readDatagram(datagram.data(), datagram.size(),
                                 &sender, &senderPort);

        QHostAddress listenTo(ui->ipEdit->text());

        if (listenTo.toIPv4Address() != sender.toIPv4Address()) {
//            qDebug() << listenTo << sender;
            return;
        }

        int ind = 23;
        double lat = get_R8((uint8_t*)datagram.data(), &ind) * 180.0 / M_PI;
        double lon = get_R8((uint8_t*)datagram.data(), &ind) * 180.0 / M_PI;
        double alt = get_R4((uint8_t*)datagram.data(), &ind);
        ind = 43;
        double velN = (double)get_I3((uint8_t*)datagram.data(), &ind) * 1e-4;
        double velE = (double)get_I3((uint8_t*)datagram.data(), &ind) * 1e-4;
        double velD = (double)get_I3((uint8_t*)datagram.data(), &ind) * 1e-4;
        double head = (double)get_I3((uint8_t*)datagram.data(), &ind) * 1e-6;

//        qDebug() << fixed << qSetRealNumberPrecision(7) << lon << lat << alt;

        mMapCnt++;
        if (mMap && mMapCnt >= 10) {
            mMapCnt = 0;

            mPacketCounter++;

            QString pktStr;
            pktStr.sprintf("Packets RX: %d", mPacketCounter);
            ui->packetLabel->setText(pktStr);

            double illh[3], llh[3], xyz[3];
            mMap->getEnuRef(illh);
            llh[0] = lat;
            llh[1] = lon;
            llh[2] = alt;

            utility::llhToEnu(illh, llh, xyz);

            //double phi = (M_PI / 2.0) - head;
            double phi = head - (M_PI / 2.0);
            utility::norm_angle_rad(&phi);
            phi *= 180 / M_PI;

            LocPoint p;
            p.setXY(xyz[0], xyz[1]);
            QString info;

            info.sprintf("Head    : %.2f\n"
                         "X       : %.2f\n"
                         "Y       : %.2f\n"
                         "Height  : %.2f\n"
                         "velN    : %.2f\n"
                         "velE    : %.2f\n"
                         "velD    : %.2f\n",
                         phi,
                         xyz[0], xyz[1], xyz[2],
                         velN, velE, velD
                    );

            p.setInfo(info);

            QString posStr;
            posStr.sprintf("X %.2f, Y: %.2f, H: %.2f, YAW: %.2f",
                           xyz[0], xyz[1], xyz[2], phi);
            ui->posLabel->setText(posStr);

            mMap->addInfoPoint(p);
        }
    }
}

void RtRange::on_connectButton_clicked()
{
    mUdpSocket->close();
    mUdpSocket->bind(QHostAddress::Any, ui->portBox->value(), QUdpSocket::ShareAddress);
}

void RtRange::on_disconnectButton_clicked()
{
    mUdpSocket->close();
    mPacketCounter = 0;
    ui->packetLabel->setText("Packets RX: 0");
}
