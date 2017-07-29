#!/usr/bin/python

import rospy
import time

from geometry_msgs.msg import Twist

timeout = 0
human_cmd_time = time.time()

def on_autonomous_cmd(twist):
    global timeout, human_cmd_time
    time_since_human_cmd = time.time() - human_cmd_time
    if time_since_human_cmd >= timeout:
        timeout = 0
        pub.publish(twist)

def on_human_cmd(twist):
    global timeout, human_cmd_time
    human_cmd_time = time.time()
    timeout = rospy.get_param('~timeout', 5)
    pub.publish(twist)

def main():
    rospy.init_node("cmd_vel_mux")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("move_base/cmd_vel", Twist, callback=on_autonomous_cmd)
    rospy.Subscriber("teleop/cmd_vel", Twist, callback=on_human_cmd)
    rospy.spin()

