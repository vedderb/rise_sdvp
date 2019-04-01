# Build GUI
#DEFINES += HAS_GUI

# Build camera support
#DEFINES += HAS_CAMERA

QT += core
QT += widgets
QT += network
QT += serialport
QT += gui

contains(DEFINES, HAS_GUI) {
    QT += quick
    QT += quickcontrols2
}

contains(DEFINES, HAS_CAMERA) {
    QT += multimedia
}

CONFIG += c++11

TARGET = Car_Client
CONFIG += console

TEMPLATE = app

SOURCES += main.cpp \
    packetinterface.cpp \
    tcpbroadcast.cpp \
    utility.cpp \
    carclient.cpp \
    locpoint.cpp \
    rtcm3_simple.c \
    serialport.cpp \
    ublox.cpp \
    nmeaserver.cpp \
    packet.cpp \
    tcpserversimple.cpp \
    chronos.cpp \
    rtcmclient.cpp \
    chronoscomm.cpp \
    vbytearrayle.cpp \
    vbytearray.cpp

HEADERS += \
    packetinterface.h \
    tcpbroadcast.h \
    utility.h \
    carclient.h \
    datatypes.h \
    locpoint.h \
    rtcm3_simple.h \
    serialport.h \
    ublox.h \
    nmeaserver.h \
    packet.h \
    tcpserversimple.h \
    chronos.h \
    rtcmclient.h \
    chronoscomm.h \
    vbytearrayle.h \
    vbytearray.h

RESOURCES += \
    res.qrc

contains(DEFINES, HAS_CAMERA) {
    QT += multimedia
    SOURCES += camera.cpp
    HEADERS += camera.h
}

include(carsim/carsim.pri)
