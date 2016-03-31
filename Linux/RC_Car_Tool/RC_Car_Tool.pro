#-------------------------------------------------
#
# Project created by QtCreator 2016-03-11T14:49:19
#
#-------------------------------------------------

QT       += core gui
QT       += widgets
QT       += printsupport
QT       += serialport
QT       += network
QT       += opengl

# Req
# sudo apt-get install libassimp-dev

TARGET = RC_Car_Tool
TEMPLATE = app

LIBS += -lassimp

SOURCES += main.cpp\
        mainwindow.cpp \
    qcustomplot.cpp \
    packetinterface.cpp \
    utility.cpp \
    mapwidget.cpp \
    carinfo.cpp \
    locpoint.cpp \
    perspectivepixmap.cpp \
    carinterface.cpp \
    orientationwidget.cpp \
    nmeaserver.cpp \
    rtcm3_simple.c \
    rtcmclient.cpp \
    tcpbroadcast.cpp \
    rtcmwidget.cpp

HEADERS  += mainwindow.h \
    qcustomplot.h \
    datatypes.h \
    packetinterface.h \
    utility.h \
    mapwidget.h \
    carinfo.h \
    locpoint.h \
    perspectivepixmap.h \
    carinterface.h \
    orientationwidget.h \
    nmeaserver.h \
    rtcm3_simple.h \
    rtcmclient.h \
    tcpbroadcast.h \
    rtcmwidget.h

FORMS    += mainwindow.ui \
    carinterface.ui \
    rtcmwidget.ui
