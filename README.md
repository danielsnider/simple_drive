# simple_drive [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__simple_drive__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__simple_drive__ubuntu_xenial_amd64__binary)

A simple robot drive system for car-like vehicles.

##
![simple_drive](http://wiki.ros.org/simple_drive?action=AttachFile&do=get&target=Simple_Drive_Diagram.png)

## Quick Start

1. Install:

```

2. Launch ROS nodes:

```
$ roslaunch simple_drive drive_teleop.launch joy_dev:=/dev/input/js0
$ roslaunch simple_drive cmd_vel_mux.launch
$ roslaunch simple_drive simple_drive.launch serial_dev:=/dev/ttyACM0

OR all-in-one launch:
$ roslaunch simple_drive drive.launch
```


4. Drive your robot around.

```
