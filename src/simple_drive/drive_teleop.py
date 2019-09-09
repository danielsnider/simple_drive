#!/usr/bin/python

import rospy
import subprocess
import time

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID


class DriveTeleop:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("teleop/cmd_vel", Twist, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=1)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy, queue_size=1)


    def on_joy(self, data):

        # Drive sticks
        if data.buttons[4]: # deadman 5 button
            angular_vel = data.axes[0] 
            linear_vel = data.axes[1]

            # Publish Twist
            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
            self.cmd_vel_pub.publish(twist)


	# EStop
        if data.buttons[1]: # 2 button
            rospy.loginfo('Estop tractor')


	# Cancel move base goal
        if data.buttons[2]: # X button
            rospy.loginfo('Cancelling move_base goal')
            cancel_msg = GoalID()
            self.goal_cancel_pub.publish(cancel_msg)


def main():
    rospy.init_node("drive_teleop")
    # rospy.wait_for_service("/relay_cmd")
    controller = DriveTeleop()
    rospy.spin()
