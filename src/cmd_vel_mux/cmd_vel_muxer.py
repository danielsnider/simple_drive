#!/usr/bin/python
import rospy
import time
import geometry_msgs.msg
import rover_drive.msg

auto_timeout = 0
dt = time.time()

rospy.init_node("cmd_vel_mux")
pub = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size=10)


def on_auto_data(data):
    data = data # type: geometry_msgs.msg.Twist
    global auto_timeout, dt
    dtt = time.time() - dt
    dt = time.time()
    auto_timeout -= dtt
    data.linear.x = data.linear.x
    data.angular.z = data.angular.z
    if auto_timeout <= 0:
        auto_timeout = 0
        pub.publish(data)

def on_twist(data):
    global auto_timeout, dt
    dt = time.time()
    auto_timeout = 5
    pub.publish(data)
#    time.sleep(5)

def on_tank(data):
    global auto_timeout, dt
    dt = time.time()
    auto_timeout = 5
    # pub.publish(data)
#    time.sleep(5)

rospy.Subscriber("cmd_vel_mux/move_base", geometry_msgs.msg.Twist, callback=on_auto_data)
rospy.Subscriber("cmd_vel_mux/teleoperation", geometry_msgs.msg.Twist, callback=on_twist)
rospy.Subscriber("cmd_vel_tank", rover_drive.msg.Tank, callback=on_tank)
rospy.spin()

