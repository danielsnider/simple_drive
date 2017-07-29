#!/usr/bin/python
import rospy
import subprocess
import std_msgs.msg
import rover_drive.msg

from sensor_msgs.msg import Joy

class DriveTeleop:
    def __init__(self):
        self.tank_pub = rospy.Publisher("cmd_vel_tank", rover_drive.msg.Tank, queue_size=1)
        self.zed_servo_pub = rospy.Publisher("zed_servo", std_msgs.msg.Float32, queue_size=1)

        self.speed_ratio = 2 # medium speed
        self.zed_pan_speed = 5 # degrees of change per button press
        self.zed_pan_max = 160 # degrees of change per button press
        self.zed_pan_min = 0 # degrees of change per button press
        self.zed_position = self.zed_pan_max/2 # forward/center zed position 

    def on_joy_data(self, data):
        # Set speed ratio using d-pad
        if data.axes[7] == 1: # full speed (d-pad up)
            self.speed_ratio = 1
        if data.axes[6] != 0: # medium speed (d-pad left or right)
            self.speed_ratio = 2
        if data.axes[7] == -1: # low speed (d-pad down)
            self.speed_ratio = 3

        # Drive sticks
        msg = rover_drive.msg.Tank()
        msg.left = -data.axes[1] / self.speed_ratio # left stick
        msg.right = -data.axes[4] / self.speed_ratio # right stick
        self.tank_pub.publish(msg)

        # Zed servo panning control
        if data.buttons[5]: # pan leftward (left bumper)
            if self.zed_position > self.zed_pan_min:
                self.zed_position -= self.zed_pan_speed
        if data.buttons[4]: # pan rightward (right bumper)
            if self.zed_position < self.zed_pan_max:
                self.zed_position += self.zed_pan_speed
        if data.buttons[3]: # forward/center zed position (Y button)
            self.zed_position = self.zed_pan_max/2
        self.zed_servo_pub.publish(self.zed_position)

        # Cancel move base goal
        if data.buttons[2] and data.axes[2] == -1: # X button
            rospy.loginfo('Cancelling move_base goal') # debug goal cancel
            out = subprocess.Popen(["rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}"], shell=True)
            rospy.loginfo(vars(out)) # debug goal cancel


controller = DriveTeleop()
rospy.init_node("drive_teleoperation_joy")
joy_sub = rospy.Subscriber("joy", Joy, controller.on_joy_data)
rospy.spin()