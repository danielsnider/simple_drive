#!/usr/bin/python

import rospy
import subprocess
import time

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

def translate(value, inMin, inMax, outMin, outMax):
    # Figure out how 'wide' each range is
    inSpan = inMax - inMin
    outSpan = outMax - outMin

    # Convert the in range into an out range (float)
    valueScaled = float(value - inMin) / float(inSpan)

    # Convert the in range into a value in the out range.
    return outMin + (valueScaled * outSpan)

class DriveTeleop:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("teleop/cmd_vel", Twist, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=1)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy, queue_size=1)

        # get values for joystick from param server
        self.drive_forward_max = rospy.get_param('~drive_forward_max', 1.0) # set max forward speed
        self.drive_reverse_max = rospy.get_param('~drive_reverse_max', -1.0) # set max reverse speed
        self.drive_axis = rospy.get_param('~drive_axis', 1) # set joystick drive axis
        self.turn_axis = rospy.get_param('~turn_axis', 0) # set joystick turn axis
        self.deadman = rospy.get_param('~deadman', 4) # set deadman button
        self.estop = rospy.get_param('~estop', 1) # set estop button
        self.move_cancel = rospy.get_param('~move_cancel', 2) # set autonomous cancel button


    def on_joy(self, data):

        # Drive sticks
        if data.buttons[self.deadman]: # deadman button
            linear_vel = data.axes[self.drive_axis]
            angular_vel = data.axes[self.turn_axis] 

            # scale speed to max/min based on values in launch file
            if linear_vel >= 0:
                new_speed = translate(linear_vel, 0.0, 1.0, 0.0, self.drive_forward_max)
            elif linear_vel < 0:
                new_speed = translate(linear_vel, -1.0, 0.0, self.drive_reverse_max, 0.0)


            # Publish message
            twist = Twist()
            twist.linear.x = new_speed
            twist.angular.z = angular_vel
            self.cmd_vel_pub.publish(twist)


    	# EStop
        if data.buttons[self.estop]:
            #  TODO:  Add function to send message to relay server.
            rospy.loginfo('Estop tractor')


    	# Cancel move base goal
        if data.buttons[self.move_cancel]: 
            rospy.loginfo('Cancelling move_base goal')
            cancel_msg = GoalID()
            self.goal_cancel_pub.publish(cancel_msg)


def main():
    rospy.init_node("drive_teleop")
    try:
        controller = DriveTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
