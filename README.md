# simple_drive

A simple robot drive system for car-like vehicles.

##
![simple_drive](http://wiki.ros.org/simple_drive?action=AttachFile&do=get&target=Simple_Drive_Diagram.png)

## Quick Start

1. Install:



2. Launch ROS nodes:

```
$ roslaunch tractor_teleop drive_teleop.launch joy_dev:=/dev/input/js0
$ roslaunch tractor_teleop cmd_vel_mux.launch
```

OR all-in-one launch:
```
$ roslaunch tractor_teleop drive.launch
```


4. Drive your robot around.


