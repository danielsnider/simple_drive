#!/usr/bin/python

import rospy
import geometry_msgs.msg
import std_msgs.msg
import serial
import struct
import rover_drive.msg

cmd_byte_map = {
    'twist': b"\x00",
    'tank': b"\x01",
    'zed_servo': b"\x02"
}

def pre_transmit():
    if theSerial.in_waiting > 0:
        rospy.logwarn("Data in serial buffer, but the driveduino shouldn't be giving me any...")

def on_new_twist(data):
    # pre_transmit()
    dat = cmd_byte_map['twist'] + struct.pack("<ff", data.linear.x, (-data.angular.z))
    theSerial.write(dat)

def on_new_tank(data):
    # pre_transmit()
    dat = cmd_byte_map['tank'] + struct.pack("<ff", -data.left, -data.right)
    theSerial.write(dat)

# def on_new_zed_servo(data):
#     # pre_transmit()
#     dat = cmd_byte_map['zed_servo'] + struct.pack("<f", data.data)
#     theSerial.write(dat)

theSerial = serial.Serial(baudrate=9600)
theSerial.port = rospy.get_param("dev")
theSerial.open()

rospy.init_node("twist_sender")
subscriber_twist = rospy.Subscriber("/cmd_vel", geometry_msgs.msg.Twist, on_new_twist, queue_size=15)
subscriber_tank = rospy.Subscriber("/cmd_vel_tank", rover_drive.msg.Tank, on_new_tank, queue_size=15)
# subscriber_zed_servo = rospy.Subscriber("/zed_servo", std_msgs.msg.Float32, on_new_zed_servo, queue_size=15)
rospy.spin()

