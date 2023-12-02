#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


rospy.init_node('teleop_node', anonymous=True)

pub = rospy.Publisher("/minibot/mecanum_drive_controller/cmd_vel", Twist)

def callback(msg):
    # publish cmd velocity with joystick input
    twist = Twist()
    twist.linear.x = msg.axes[0]
    twist.linear.y = msg.axes[1]
    twist.angular.z = msg.axes[3]
    pub.publish(twist)


rospy.Subscriber("/minibot/joy", Joy, callback)

rospy.spin()