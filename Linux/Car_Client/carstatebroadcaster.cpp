#include "carstatebroadcaster.h"
#include "vbytearray.h"
#include "datatypes.h"

CarStateBroadcaster::CarStateBroadcaster(QObject *parent, PacketInterface *carClientPacketInterface, quint8 idToBroadcast) : QObject(parent)
{
    mPacketInterface = carClientPacketInterface;
    connect(mPacketInterface, &PacketInterface::packetReceived, this, &CarStateBroadcaster::carPacketRx);

    mIdToBroadcast = idToBroadcast;

    mTcpBroadcast.startTcpServer(mCarStateBroadcastPort);

    connect(&mGetStateTimer, &QTimer::timeout, this, [this]() {
        mPacketInterface->getState(mIdToBroadcast);
    });
    mGetStateTimer.start(100);

    qDebug() << "Broadcasting state of car id" << mIdToBroadcast << "on TCP port" << mCarStateBroadcastPort;
}

void CarStateBroadcaster::carPacketRx(quint8 id, CMD_PACKET cmd, const QByteArray &data)
{
    if (cmd == CMD_GET_STATE && id == mIdToBroadcast) {
        //    Data that is broadcasted over TCP:
        //    Uint8     - vechicle id
        //    Uint8     - packet id
        //    Uint8     - FW_VERSION_MAJOR
        //    Uint8     - FW_VERSION_MINOR
        //    Double32  - roll [degrees]
        //    Double32  - pitch [degrees]
        //    Double32  - yaw [degrees]
        //    Double32  - accel_x
        //    Double32  - accel_y
        //    Double32  - accel_z
        //    Double32  - roll_rate
        //    Double32  - pitch_rate
        //    Double32  - yaw_rate
        //    Double32  - magnet x
        //    Double32  - magnet y
        //    Double32  - magnet z
        //    Double32  - postition x [meters]
        //    Double32  - postition y [meters]
        //    Double32  - speed [meters / second]
        //    Double32  - v_in
        //    Double32  - temp mos
        //    Uint8     - motor controller fault code
        //    Double32  - postition x GNSS
        //    Double32  - postition y GNSS
        //    Double32  - current goal postition x
        //    Double32  - current goal postition y
        //    Double32  - autopilot current rad
        //    Int32     - current time UTC
        //    Int16     - autopilot route left
        //    Double32  - postition x UWB
        //    Double32  - postition x UWB
        assert(data.size() == 107 && "ERROR: Received state had unexpected size.");

        mTcpBroadcast.broadcastData(data);
    }
}
