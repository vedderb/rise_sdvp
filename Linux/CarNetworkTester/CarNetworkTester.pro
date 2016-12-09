#-------------------------------------------------
#
# Project created by QtCreator 2016-12-09T12:31:32
#
#-------------------------------------------------

QT       += core gui
QT       += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = CarNetworkTester
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

SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui
