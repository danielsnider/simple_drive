#!/usr/bin/python

import rospy
import serial
import struct

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

cmd_byte_map = {
    'twist': b"\x00",
    'servo': b"\x02"
}

def main():
    rospy.init_node("simple_drive")
    
    baudrate = rospy.get_param('~baudrate', 9600)
    Serial = serial.Serial(baudrate=baudrate)
    Serial.port = rospy.get_param("~serial_dev")
    Serial.open()
    
    def on_new_twist(data):
        serial_msg = cmd_byte_map['twist'] + struct.pack("<ff", data.linear.x, data.angular.z)
        Serial.write(serial_msg)

    def on_new_servo(data):
        serial_msg = cmd_byte_map['servo'] + struct.pack("<f", data.data)
        Serial.write(serial_msg)


    subscriber_twist = rospy.Subscriber("cmd_vel", Twist, on_new_twist, queue_size=10)
    subscriber_servo = rospy.Subscriber("servo_pos", Float32, on_new_servo, queue_size=10)

    rospy.spin()
