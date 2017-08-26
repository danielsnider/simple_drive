#!/usr/bin/python

import rospy
import subprocess

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalID

class DriveTeleop:
    def __init__(self):
        self.speed_setting = 2 # default to medium speed
        self.servo_pan_speed = rospy.get_param('~servo_pan_speed', 5) # degrees of change per button press
        self.servo_pan_max = rospy.get_param('~servo_pan_max', 160) # max angle of servo rotation
        self.servo_pan_min = rospy.get_param('~servo_pan_min', 0) # min angle of servo rotation
        self.servo_position = self.servo_pan_max/2 # center servo position

        self.cmd_vel_pub = rospy.Publisher("teleop/cmd_vel", Twist, queue_size=1)
        self.goal_cancel_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=1)
        self.servo_pub = rospy.Publisher("servo_pos", Float32, queue_size=1)
        self.joy_sub = rospy.Subscriber("joy", Joy, self.on_joy)

    def on_joy(self, data):
        # Set speed ratio using d-pad
        if data.axes[7] == 1: # full speed (d-pad up)
            self.speed_setting = 1
        if data.axes[6] != 0: # medium speed (d-pad left or right)
            self.speed_setting = 2
        if data.axes[7] == -1: # low speed (d-pad down)
            self.speed_setting = 3

        # Drive sticks
        left_speed = -data.axes[1] / self.speed_setting # left stick
        right_speed = -data.axes[4] / self.speed_setting # right stick

        # Convert skid steering speeds to twist speeds
        linear_vel  = (left_speed + right_speed) / 2.0 # (m/s)
        angular_vel  = (right_speed - left_speed) / 2.0 # (rad/s)

        # Publish Twist
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)

        # Servo servo panning control
        if data.buttons[5]: # pan leftward (left bumper)
            if self.servo_position > self.servo_pan_min:
                self.servo_position -= self.servo_pan_speed
        if data.buttons[4]: # pan rightward (right bumper)
            if self.servo_position < self.servo_pan_max:
                self.servo_position += self.servo_pan_speed
        if data.buttons[3]: # center servo position (Y button)
            self.servo_position = self.servo_pan_max/2
        self.servo_pub.publish(self.servo_position)

        # Cancel move base goal
        if data.buttons[2]: # X button
            rospy.loginfo('Cancelling move_base goal')
            cancel_msg = GoalID()
            self.goal_cancel_pub.publish(cancel_msg)

def main():
    rospy.init_node("drive_teleop")
    controller = DriveTeleop()
    rospy.spin()