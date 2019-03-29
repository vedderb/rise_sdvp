#-------------------------------------------------
#
# Project created by QtCreator 2018-05-08T09:52:11
#
#-------------------------------------------------

QT       += core network
QT       -= gui

CONFIG   += c++11

TARGET = RControlStationComm
TEMPLATE = lib

DEFINES += RCONTROLSTATIONCOMM_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    rcontrolstationcomm_wrapper.cpp \
    rcontrolstationcomm.cpp

HEADERS += \
    rcontrolstationcomm_wrapper.h \
    rcontrolstationcomm.h \
    rcontrolstationcomm_global.h \
    rcontrolstationcomm_types.h

header_files.files = \
    rcontrolstationcomm.h \
    rcontrolstationcomm_wrapper.h \
    rcontrolstationcomm_types.h

INSTALLS += header_files

unix {
    target.path = /usr/lib
    header_files.path = /usr/include
    INSTALLS += target
}
