#!/usr/bin/python

import rospy
import time

from geometry_msgs.msg import Twist

block_duration = 0 # do not block by default
human_cmd_time = time.time()

def on_autonomous_cmd(twist):
    global block_duration, human_cmd_time
    time_since_human_cmd = time.time() - human_cmd_time
    if time_since_human_cmd >= block_duration:
        block_duration = 0 # stop blocking
        pub.publish(twist)

def on_human_cmd(twist):
    global block_duration, human_cmd_time
    human_cmd_time = time.time()
    block_duration = rospy.get_param('~block_duration', 5)
    pub.publish(twist)

def main():
    rospy.init_node("cmd_vel_mux")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("move_base/cmd_vel", Twist, callback=on_autonomous_cmd)
    rospy.Subscriber("teleop/cmd_vel", Twist, callback=on_human_cmd)
    rospy.spin()

