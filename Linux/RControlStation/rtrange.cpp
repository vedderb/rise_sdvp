#include "rtrange.h"
#include "ui_rtrange.h"
#include "utility.h"

#include <QDebug>

namespace
{
static int32_t get_I3(uint8_t *msg, int *ind) {
    int32_t res =	((uint32_t) msg[*ind + 2]) << 16 |
                    ((uint32_t) msg[*ind + 1]) << 8 |
                    ((uint32_t) msg[*ind + 0]);
    *ind += 3;
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
}

RtRange::RtRange(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RtRange)
{
    ui->setupUi(this);
    mUdpSocket = new QUdpSocket(this);
    mPacketCounter = 0;
    mMap = 0;

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

void RtRange::readPendingDatagrams()
{
    while (mUdpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(mUdpSocket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        mUdpSocket->readDatagram(datagram.data(), datagram.size(),
                                 &sender, &senderPort);

        mPacketCounter++;
        QString pktStr;
        pktStr.sprintf("Packets RX: %d", mPacketCounter);
        ui->packetLabel->setText(pktStr);

        int ind = 23;
        double lat = get_R8((uint8_t*)datagram.data(), &ind) * 180.0 / M_PI;
        double lon = get_R8((uint8_t*)datagram.data(), &ind) * 180.0 / M_PI;
        double alt = get_R4((uint8_t*)datagram.data(), &ind);
        ind = 52;
        double head = (double)get_I3((uint8_t*)datagram.data(), &ind) * 1e-6 * 180.0 / M_PI;

        if (mMap) {
            double illh[3], llh[3], xyz[3];
            mMap->getEnuRef(illh);
            llh[0] = lat;
            llh[1] = lon;
            llh[2] = alt;

            utility::llhToEnu(illh, llh, xyz);

            LocPoint p;
            p.setXY(xyz[0], xyz[1]);
            QString info;

            info.sprintf("Head    : %.2f\n"
                         "X       : %.2f\n"
                         "Y       : %.2f\n"
                         "Height  : %.2f\n",
                         head,
                         xyz[0], xyz[1], xyz[2]);

            p.setInfo(info);

            mMap->addInfoPoint(p);
        }

        qDebug() << lat << lon << alt << head;
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
