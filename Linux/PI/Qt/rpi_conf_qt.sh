#!/bin/bash

### Settings
# Exclude these Qt modules
EXCLUDE_MODULES=( qtscript qtwebengine qtwebsockets qtwebchannel
		qtwayland qtwinextras qtsensors multimedia
		imageformats location 3d )

# Qt "-no-" options
NO_INCLUDES=( sql-mysql sql-psql sql-sqlite journald mtdev
        compile-examples xcb-xlib directfb linuxfb kms
        xkbcommon-evdev evdev libproxy icu xinput2 openssl
)
# Your own system
PLATFORM="linux-g++"
# Qt version
QVERSION="5.9"
# Location of the Pi sysroot on your system
QT_SYSROOT_PREFIX="/opt/Qt5Pi/sysroot"
# Location on the Raspberry Pi where Qt will be installed
QT_PREFIX="/opt/Qt/${QVERSION}-pi/"


###
# Some general linking and build options
QT_OPTIONS="-opengl es2 -no-use-gold-linker -qt-freetype -nomake examples -nomake tests -qt-xkbcommon-x11 -release -opensource -confirm-license -v -make libs -v"
# Compilation options
QT_OPTIONS="${QT_OPTIONS} -device linux-rasp-pi3-g++ -device-option CROSS_COMPILE=arm-linux-gnueabihf- -platform ${PLATFORM} -make libs -v"

# Create the Qt configure options
EXCLUDE_STRING=""

for i in "${EXCLUDE_MODULES[@]}"
do
    EXCLUDE_STRING="${EXCLUDE_STRING} -skip ${i}"
done

NO_INCLUDES_STRING=""

for i in "${NO_INCLUDES[@]}"
do
    NO_INCLUDES_STRING="${NO_INCLUDES_STRING} -no-${i}"
done

# Create separate build directory
mkdir qt5build
cd qt5build

# Configure Qt for building
../qt-everywhere-opensource-src-5.9.1/configure -prefix $QT_PREFIX -sysroot ${QT_SYSROOT_PREFIX} ${QT_OPTIONS} ${EXCLUDE_STRING} ${NO_INCLUDES_STRING}
