QT += core
QT -= gui
QT += network
QT += serialport

TARGET = Car_Client
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    packetinterface.cpp \
    tcpbroadcast.cpp \
    utility.cpp \
    carclient.cpp \
    locpoint.cpp

HEADERS += \
    packetinterface.h \
    tcpbroadcast.h \
    utility.h \
    carclient.h \
    datatypes.h \
    locpoint.h

