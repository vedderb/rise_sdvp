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
QT       += quick
QT       += quickcontrols2

CONFIG   += c++11

# Ubuntu
# sudo apt-get install libassimp-dev
#DEFINES += HAS_ASSIMP

# Linux only
unix:!macx {
    DEFINES += HAS_JOYSTICK
}

# OpenGL support
!android: DEFINES += HAS_OPENGL

# Lime SDR support
#DEFINES += HAS_LIME_SDR

# Simulation Scennarios
#DEFINES += HAS_SIM_SCEN
# Usage: From the RControlStation root do:
# git clone https://github.com/esmini/esmini esmini
# and uncomment this define. The the editor will show up
# as the last tab in RControlStation.

TARGET = RControlStation
TEMPLATE = app

release_win {
    DESTDIR = build/win
    OBJECTS_DIR = build/win/obj
    MOC_DIR = build/win/obj
    RCC_DIR = build/win/obj
    UI_DIR = build/win/obj
}

release_lin {
    # http://micro.nicholaswilson.me.uk/post/31855915892/rules-of-static-linking-libstdc-libc-libgcc
    # http://insanecoding.blogspot.se/2012/07/creating-portable-linux-binaries.html
    QMAKE_LFLAGS += -static-libstdc++ -static-libgcc
    DESTDIR = build/lin
    OBJECTS_DIR = build/lin/obj
    MOC_DIR = build/lin/obj
    RCC_DIR = build/lin/obj
    UI_DIR = build/lin/obj
}

release_android {
    DESTDIR = build/android
    OBJECTS_DIR = build/android/obj
    MOC_DIR = build/android/obj
    RCC_DIR = build/android/obj
    UI_DIR = build/android/obj
}

contains(DEFINES, HAS_ASSIMP) {
    LIBS += -lassimp
}

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
    nmeaserver.cpp \
    rtcm3_simple.c \
    rtcmclient.cpp \
    tcpbroadcast.cpp \
    rtcmwidget.cpp \
    basestation.cpp \
    ping.cpp \
    networklogger.cpp \
    osmclient.cpp \
    osmtile.cpp \
    tcpserversimple.cpp \
    packet.cpp \
    networkinterface.cpp \
    moteconfig.cpp \
    magcal.cpp \
    imuplot.cpp \
    copterinfo.cpp \
    copterinterface.cpp \
    nmeawidget.cpp \
    confcommonwidget.cpp \
    ublox.cpp \
    intersectiontest.cpp \
    ncom.cpp \
    correctionanalysis.cpp \
    historylineedit.cpp \
    imagewidget.cpp \
    tcpclientmulti.cpp

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
    nmeaserver.h \
    rtcm3_simple.h \
    rtcmclient.h \
    tcpbroadcast.h \
    rtcmwidget.h \
    basestation.h \
    ping.h \
    networklogger.h \
    osmclient.h \
    osmtile.h \
    tcpserversimple.h \
    packet.h \
    networkinterface.h \
    moteconfig.h \
    magcal.h \
    imuplot.h \
    copterinfo.h \
    copterinterface.h \
    nmeawidget.h \
    confcommonwidget.h \
    ublox.h \
    intersectiontest.h \
    ncom.h \
    correctionanalysis.h \
    historylineedit.h \
    imagewidget.h \
    tcpclientmulti.h

FORMS    += mainwindow.ui \
    carinterface.ui \
    rtcmwidget.ui \
    basestation.ui \
    networklogger.ui \
    networkinterface.ui \
    moteconfig.ui \
    magcal.ui \
    imuplot.ui \
    copterinterface.ui \
    nmeawidget.ui \
    confcommonwidget.ui \
    intersectiontest.ui \
    ncom.ui \
    correctionanalysis.ui

contains(DEFINES, HAS_JOYSTICK) {
    SOURCES += joystick.cpp
    HEADERS += joystick.h
}

contains(DEFINES, HAS_OPENGL) {
    SOURCES += orientationwidget.cpp
    HEADERS += orientationwidget.h
}

contains(DEFINES, HAS_LIME_SDR) {
    SOURCES += gpssim.cpp gpsgen.cpp limesdr.cpp
    HEADERS += gpssim.h gpsgen.h limesdr.h
    FORMS += gpssim.ui
    LIBS += -lLimeSuite
}

contains(DEFINES, HAS_SIM_SCEN) {
    include(esmini/EnvironmentSimulator.pri)
    SOURCES += pagesimscen.cpp
    HEADERS += pagesimscen.h \
            simscentree.h
    FORMS += pagesimscen.ui
}

RESOURCES += \
    resources.qrc

DISTFILES += \
    android/AndroidManifest.xml \
    android/gradle/wrapper/gradle-wrapper.jar \
    android/gradlew \
    android/res/values/libs.xml \
    android/build.gradle \
    android/gradle/wrapper/gradle-wrapper.properties \
    android/gradlew.bat

contains(ANDROID_TARGET_ARCH,armeabi-v7a) {
    ANDROID_PACKAGE_SOURCE_DIR = \
        $$PWD/android
}
