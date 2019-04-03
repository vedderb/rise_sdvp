# The RISE Self-Driving Model Vehicle Platform (SDVP)
## Quick start
This quick start guide assumes any firmware for motor controllers and USB communication are already set up. The quick start guide below uses SDVP_ROOT as the root directory of this repository. 

### Remote control station
On the device from which you want to control the RC car, build the RControlStation file:

```
cd ${SDVP_ROOT}
cd Linux/RControlStation
qmake -config release "CONFIG+=release_lin" RControlStation.pro
make clean
make -j8
rm -rf build/lin/obj
```

and then run the built file with

```
./RControlStation
```

You must create a new car tab and then change its ID (on the right hand side) to something else than zero before connecting.

### RC car software
On the RC car, build the Car_Client file:

```
cd ${SDVP_ROOT}
cd Linux/Car_Client
/opt/Qt/5.9-pi/bin/qmake "DEFINES += HAS_GUI"
make -j4
```

Then run the built file with
``` 
./Car_Client -p /dev/car --useudp --usetcp --usegui
```

## Introduction
This is a fork of the source code and hardware design for a model vehicle platform developed and maintained at RISE Research Institutes of Sweden. The platform currently has full support for cars with Ackermann steering, robots with differential steering and partial support for quadcopters.  

Self-Driving in this context means that the vehicles can follow a pre-programmed path outdoors accurately using RTK-GNSS. The paths can be edited using RControlStation as a set of points with different time stamps or velocities depending on the mode. It is also possible to send paths to the vehicles with time stamps in real-time from external applications (either using UDP or TCP to RControlStation or directly over a radio link) for e.g. following other vehicles.  

Some details about the position estimation and the autopilot can be found in our [published article](https://www.hindawi.com/journals/jr/2018/4907536/).  

Figure 1 shows a screenshot of RControlStation, which is a GUI where model vehicles can be viewed, configured and controlled in real time.

![RControlStation](Documentation/Pictures/GUI/map.png)

Here is a video with a short test of the SDVP on a Tero RC car:

[![RC Car Test](http://img.youtube.com/vi/4wPVpvPP-8w/0.jpg)](http://www.youtube.com/watch?v=4wPVpvPP-8w "Tero RC car with autopilot and RTK-GPS")

Currently a lot of documentation is missing, but this will be added over time along with tutorials on how to use this platform. The content of the documentation directory is incomplete and out of date, so don't rely on it too much.

