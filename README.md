# simple_drive [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__simple_drive__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__simple_drive__ubuntu_xenial_amd64__binary)

A simple robot drive system that includes skid steering joystick teleoperation, control of a panning servo to look around the robot, and Arduino firmware.

![controller](http://wiki.ros.org/simple_drive?action=AttachFile&do=get&target=Xbox_Controller.png)
##
![simple_drive](http://wiki.ros.org/simple_drive?action=AttachFile&do=get&target=Simple_Drive_Diagram.png)

## Quick Start

1. Install:

```
$ sudo apt-get install ros-kinetic-simple-drive
```

2. Launch ROS nodes:

```
$ roslaunch simple_drive drive_teleop.launch joy_dev:=/dev/input/js0
$ roslaunch simple_drive cmd_vel_mux.launch
$ roslaunch simple_drive simple_drive.launch serial_dev:=/dev/ttyACM0

OR all-in-one launch:
$ roslaunch simple_drive drive.launch
```

3. Install the drive_firmware onto a microcontroller connected to motors and wheels.

4. Drive your robot around.

**Full documentation on wiki: [http://wiki.ros.org/simple_drive](http://wiki.ros.org/simple_drive)**
