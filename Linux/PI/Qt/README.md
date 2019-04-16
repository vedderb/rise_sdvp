## Setting up your system to cross compile for Raspberry Pi
This guide is a modified version of Qt's [official guide](https://wiki.qt.io/RaspberryPi2EGLFS). It has been tested with a Raspberry Pi 3 model B running Ubuntu Mate version 18.04 and Qt versions 5.9.1 and 5.12.0.

1. Prepare the Raspberry Pi (do this step on the Raspberry Pi):

   a) Install dependencies for running Qt executables:
      ```
      sudo apt-get update
      sudo apt-get build-dep qt4-x11
      sudo apt-get build-dep libqt5gui5
      sudo apt-get install libudev-dev libinput-dev libts-dev libxcb-xinerama0-dev libxcb-xinerama0
      ```
      
   b) Create a directory to hold the Qt installation:
      ```
      sudo mkdir /opt/Qt
      sudo chown $USER /opt/Qt
      ```
      
   c) Fix the linking to EGL/GLES graphics libraries. The device may have the Mesa version of libEGL and libGLESv2 in /usr/lib/arm-linux-gnueabihf, resulting in Qt apps picking these instead of the real thing from /opt/vc/lib:
      ```
      sudo mv /usr/lib/arm-linux-gnueabihf/libEGL.so.1.0.0 /usr/lib/arm-linux-gnueabihf/libEGL.so.1.0.0_backup
      sudo mv /usr/lib/arm-linux-gnueabihf/libGLESv2.so.2.0.0 /usr/lib/arm-linux-gnueabihf/libGLESv2.so.2.0.0_backup
      sudo ln -s /opt/vc/lib/libEGL.so /usr/lib/arm-linux-gnueabihf/libEGL.so.1.0.0
      sudo ln -s /opt/vc/lib/libGLESv2.so /usr/lib/arm-linux-gnueabihf/libGLESv2.so.2.0.0
      sudo ln -s /opt/vc/lib/libbrcmEGL.so /opt/vc/lib/libEGL.so
      sudo ln -s /opt/vc/lib/libbrcmGLESv2.so.2 /opt/vc/lib/libGLESv2.so
      sudo ln -s /opt/vc/lib/libEGL.so /opt/vc/lib/libEGL.so.1
      sudo ln -s /opt/vc/lib/libGLESv2.so /opt/vc/lib/libGLESv2.so.2
      ```
      
From here on, all steps are to be performed on your system, not the Raspberry Pi:
2. Some preliminaries to preprare your system for cross compiling:

   a) If you do not have an RSA key, generate one. Then, assuming your user on the Raspberry Pi is 'pi', and the IP address of the Raspberry Pi is 192.168.123.123, add yourself to the list of trusted users:
      ```
      cat ~/.ssh/id_rsa.pub | ssh -o StrictHostKeyChecking=no pi@192.168.123.123 "mkdir -p .ssh && chmod 700 .ssh && cat >> .ssh/authorized_keys"
      ```

   b) Install gcc ARM cross compilers and other dependencies:
      ```
      sudo apt-get update
      sudo apt-get upgrade
      sudo apt-get install build-essential
      sudo apt-get install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
      sudo apt-get install bison python gperf rsync
      ```

   c) Create working directory for Qt:
      ```
      sudo mkdir /opt/Qt5Pi
      sudo chown $USER /opt/Qt5Pi
      ```

   d) Mirror Pi filesystem (this may take some time):
      ```
      cd /opt/Qt5Pi
      mkdir sysroot sysroot/usr sysroot/opt
      rsync -avz pi@192.168.123.123:/lib sysroot
      rsync -avz pi@192.168.123.123:/usr/include sysroot/usr
      rsync -avz pi@192.168.123.123:/usr/lib sysroot/usr
      rsync -avz pi@192.168.123.123:/opt/vc sysroot/opt
      ```

   e) Convert absolute symlinks to relative ones:
      ```
      wget https://github.com/RI-SE/rise_sdvp/blob/master/Linux/PI/Qt/sysroot-relativelinks.py
      chmod +x sysroot-relativelinks.py
      python sysroot-relativelinks.py sysroot
      ```

3. Build and install Qt:

   a) Download your chosen Qt version (note also that 5.9.x does not support OpenSSL 1.1, and that 5.9.0 does not build properly with this configuration), and extract it:
      ```
      wget https://download.qt.io/official_releases/qt/5.9/5.9.1/single/qt-everywhere-opensource-src-5.9.1.tar.xz
      tar xf qt-everywhere-opensource-src-5.9.1.tar.xz
      ```

   b) Create configuration for hard floating point unit:
      ```
      cp -R qt-everywhere-opensource-src-5.9.1/qtbase/mkspecs/linux-arm-gnueabi-g++ qt-everywhere-opensource-src-5.9.1/qtbase/mkspecs/linux-arm-gnueabihf-g++
      sed -i -e 's/arm-linux-gnueabi-/arm-linux-gnueabihf-/g' qt-everywhere-opensource-src-5.9.1/qtbase/mkspecs/linux-arm-gnueabihf-g++/qmake.conf
      ```

   c) Modify mkspecs for the target device (linux-rasp-pi3-g++):
      ```
      mv qt-everywhere-opensource-src-5.9.1/qtbase/mkspecs/devices/linux-rasp-pi3-g++/qmake.conf qt-everywhere-opensource-src-5.9.1/qtbase/mkspecs/devices/linux-rasp-pi3-g++/backup.conf
      wget -q https://github.com/RI-SE/rise_sdvp/blob/master/Linux/PI/Qt/mkspecs/devices/linux-rasp-pi3-g++/qmake.conf -O qt-everywhere-opensource-src-5.9.1/qtbase/mkspecs/devices/linux-rasp-pi3-g++/qmake.conf
      ```

   d) Create a separate build directory which you can nuke if something goes wrong:
      ```
      cd /opt/Qt5Pi
      mkdir qt5build
      ```

   e) Run the qt configure script:
      ```
      wget -q https://github.com/RI-SE/rise_sdvp/blob/master/Linux/PI/Qt/rpi_conf_qt.sh
      chmod +x rpi_conf_qt.sh
      ./rpi_conf_qt.sh
      ```

   f) Build Qt (and go do something else):
      ```
      cd qt5build
      make
      ```

   g) Install Qt into Pi sysroot:
      ```
      make install
      ```

   h) Transfer the Qt installation to the Raspberry Pi:
      ```
      rsync -avz  /opt/Qt5Pi/sysroot/opt/Qt/5.9-pi pi@192.168.123.123:/opt/Qt
      ```
      
4) Setup Qt creator to use your new toolchain:

   a) Create a new device:
      ```
      Go to Tools > Options...
      Under Devices, click Add...
         Generic Linux Device
         Name the device (e.g. RC-car)
         Enter username, password/RSA key, IP address
      Click Test to ensure everything is configured properly
      ```
      
   b) Add the compiler to the list of compilers if it is not autodetected:
      ```
      Go to Tools > Options...
      Under Kits > Compilers, add the arm-linux-gnueabihf-g++/gcc compiler
         or select one of the arm-32 bit compilers
      ```
      
   c) Add the newly built Qt version:
      ```
      Go to Tools > Options...
      Under Kits > Qt Versions, click Add...
         Navigate to /opt/Qt5Pi/sysroot/opt/Qt/5.9-pi/bin and select qmake
         Name the Qt version (e.g. Qt5.9 for Raspberry Pi 3)
      ```
      
   d) Create a new kit:
      ```
      Go to Tools > Options...
      Under Kits > Kits, click Add, then modify the settings:
         Device type: Generic Linux Device
         Device: RC-car
         sysroot: /opt/Qt5Pi/sysroot
         Compiler: Select the compilers you chose in step b)
         Qt version: Qt5.9 for Raspberry Pi 3
      ```

