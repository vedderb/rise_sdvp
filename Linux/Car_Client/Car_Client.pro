QT += core
QT -= gui
QT += network
QT += serialport

CONFIG += c++11

TARGET = Car_Client
CONFIG += console
CONFIG -= app_bundle

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
    vbytearray.cpp \
    rtcmclient.cpp

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
    vbytearray.h \
    rtcmclient.h

