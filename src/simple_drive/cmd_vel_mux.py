#!/usr/bin/python

import rospy
import time

from geometry_msgs.msg import Twist

class CmdVelMux:
    def __init__(self):
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("move_base/cmd_vel", Twist, callback=self.on_autonomous_cmd)
        rospy.Subscriber("teleop/cmd_vel", Twist, callback=self.on_human_cmd)
        self.block_duration = 0
        self.human_cmd_time = time.time()

    def on_autonomous_cmd(self, twist):
        time_since_human_cmd = time.time() - self.human_cmd_time
        if time_since_human_cmd >= self.block_duration:
            self.block_duration = 0 # stop blocking
            self.pub.publish(twist)

    def on_human_cmd(self, twist):
        self.human_cmd_time = time.time()
        self.block_duration = rospy.get_param('~block_duration', 5) # to get the lowest latency, comment this or hard code this so that there is no call to rospy.get_param
        self.pub.publish(twist)

def main():
    rospy.init_node("cmd_vel_mux")
    muxer = CmdVelMux()
    rospy.spin()
