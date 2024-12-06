#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import moveit_commander
import sys
import math
import time

# 900 comp joystick: Bus 003 Device 055: ID 045e:028e Microsoft Corp. Xbox360 Controller
# Xbox controller: Bus 003 Device 045: ID 045e:02ea Microsoft Corp. Xbox One S Controller

rospy.init_node('teleop_node', anonymous=True)

pub = rospy.Publisher("/minibot/mecanum_drive_controller/cmd_vel", Twist)

def callback(msg):
    global intake_at_low_speed, LOW_INTAKE_SPEED, ANGULAR_SPEED_MULTIPLIER
    # publish cmd velocity with joystick input
    twist = Twist()
    twist.linear.x = -msg.axes[0]
    twist.linear.y = -msg.axes[1]
    twist.angular.z = -1 * (-1 if msg.axes[3] > 0 else 1) * ANGULAR_SPEED_MULTIPLIER * min(1, msg.axes[3] ** 2 + (0.5 if msg.axes[3] > 0 else 0))

    pub.publish(twist)

rospy.Subscriber("/minibot/joy", Joy, callback)

rospy.spin()
