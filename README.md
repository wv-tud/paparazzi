# INDI Experiment Explanation:
To install and use Paparazzi, see the info below.

For the INDI experiment, select the following in Paparazzi Center:
Airframe: airframes/bebop\_indi.xml
Flight plan: flight\_plans/rotorcraft\_basic.xml (irrelevant, the experiment is not autonomous)
Settings: settings/rotorcraft\_basic.xml
Radio: radios/cockpitSX.xml (irrelevant, the Bebop does not have radio)
Telemetry: telemetry/default\_rotorcraft.xml

The Bebop has to be flown with a joystick connected to the computer that has a telemetry link with the drone. In Paparazzi, choose tools->joystick and follow the instructions.

The airframe file contains important settings, such as whether ADAPTIVE\_INDI is defined TRUE or FALSE or if the comparison PID controller is enabled (ATTITUDE\_PID TRUE/FALSE)

The INDI code is located in sw/airborne/firmwares/rotorcraft/stabilization/stabilization\_attitude\_quat\_indi.c

The doublet input can be applied by switching from MODE\_MANUAL to MODE\_AUTO1 (they are both defined to be attitude rc control in the airframe file)

Full speed logs can be made by using the file\_logger module

# MAIN README

Paparazzi UAS
=============

[![Build Status](https://travis-ci.org/paparazzi/paparazzi.png?branch=master)](https://travis-ci.org/paparazzi/paparazzi) [![Gitter chat](https://badges.gitter.im/paparazzi/discuss.svg)](https://gitter.im/paparazzi/discuss)

Paparazzi is an attempt to develop a free software Unmanned (Air) Vehicle System.
 As of today the system is being used successfuly by a number of hobbyists, universities and companies all over the world, on vehicle of various size ( 100g to 25Kg ) and of various nature ( fixed wing, rotorcrafts, boats and surface vehicles).

Up to date information is available in the wiki http://wiki.paparazziuav.org

and from the mailing list [paparazzi-devel@nongnu.org] (http://savannah.nongnu.org/mail/?group=paparazzi)
and the IRC channel (freenode, #paparazzi).


Required Software
-----------------

Installation is described in the wiki (http://wiki.paparazziuav.org/wiki/Installation).

For Ubuntu users, required packages are available in the [paparazzi-uav PPA] (https://launchpad.net/~paparazzi-uav/+archive/ppa),
Debian users can use the [OpenSUSE Build Service repository] (http://download.opensuse.org/repositories/home:/flixr:/paparazzi-uav/Debian_7.0/)

Debian/Ubuntu packages:
- **paparazzi-dev** is the meta-package that depends on everything needed to compile and run the ground segment and the simulator.
- **paparazzi-jsbsim** is needed for using JSBSim as flight dynamic model for the simulator.

Recommended cross compiling toolchain: https://launchpad.net/gcc-arm-embedded


Directories quick and dirty description:
----------------------------------------

_conf_: the configuration directory (airframe, radio, ... descriptions).

_data_: where to put read-only data (e.g. maps, terrain elevation files, icons)

_doc_: documentation (diagrams, manual source files, ...)

_sw_: software (onboard, ground station, simulation, ...)

_var_: products of compilation, cache for the map tiles, ...


Compilation and demo simulation
-------------------------------

1. type "make" in the top directory to compile all the libraries and tools.

2. "./paparazzi" to run the Paparazzi Center

3. Select the "Microjet" aircraft in the upper-left A/C combo box.
  Select "sim" from upper-middle "target" combo box. Click "Build".
  When the compilation is finished, select "Simulation" from
  the upper-right session combo box and click "Execute".

4. In the GCS, wait about 10s for the aircraft to be in the "Holding point" navigation block.
  Switch to the "Takeoff" block (lower-left blue airway button in the strip).
  Takeoff with the green launch button.

Uploading of the embedded software
----------------------------------

1. Power the flight controller board while it is connected to the PC with the USB cable.

2. From the Paparazzi center, select the "ap" target, and click "Upload".


Flight
------

1.  From the Paparazzi Center, select the flight session and ... do the same than in simulation !
