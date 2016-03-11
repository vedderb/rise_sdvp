#-------------------------------------------------
#
# Project created by QtCreator 2016-03-11T14:49:19
#
#-------------------------------------------------

QT       += core gui
QT       += printsupport
QT       += serialport
QT       += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RC_Car_Tool
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    qcustomplot.cpp \
    packetinterface.cpp \
    utility.cpp

HEADERS  += mainwindow.h \
    qcustomplot.h \
    datatypes.h \
    packetinterface.h \
    utility.h

FORMS    += mainwindow.ui
