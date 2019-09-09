#!/usr/bin/python

import rospy
import time

from geometry_msgs.msg import Twist


def translate(value, inMin, inMax, outMin, outMax):
    # Figure out how 'wide' each range is
    inSpan = inMax - inMin
    outSpan = outMax - outMin

    # Convert the in range into an out range (float)
    valueScaled = float(value - inMin) / float(inSpan)

    # Convert the in range into a value in the out range.
    return outMin + (valueScaled * outSpan)


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
	    # Scale msg
	    # value, outMin, inMax, outMin, outMax
	    cmd_angle = twist.angular
            if cmd_angle.z > 0.3:
                cmd_angle.z = 0.3
            elif cmd_angle.z < -0.3:
                cmd_angle.z = -0.3

	    new_angle = translate(cmd_angle.z, -0.3, 0.3, -1, 1)
	    current_twist = Twist()
	    current_twist.linear = twist.linear
	    current_twist.angular.z = new_angle
            self.pub.publish(current_twist)

    def on_human_cmd(self, twist):
        self.human_cmd_time = time.time()
        self.block_duration = 5 # to get the lowest latency, comment this or hard code this so that there is no call to rospy.get_param
        self.pub.publish(twist)

def main():
    rospy.init_node("cmd_vel_mux")
    muxer = CmdVelMux()
    rospy.spin()
