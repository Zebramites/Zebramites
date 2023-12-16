#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# 900 comp joystick: Bus 003 Device 055: ID 045e:028e Microsoft Corp. Xbox360 Controller

rospy.init_node('teleop_node', anonymous=True)

pub = rospy.Publisher("/minibot/mecanum_drive_controller/cmd_vel", Twist)
pub_intake = rospy.Publisher("/minibot/intake_controller/command", Float64)

def callback(msg):
    # publish cmd velocity with joystick input
    twist = Twist()
    twist.linear.x = msg.axes[0]
    twist.linear.y = msg.axes[1]
    twist.angular.z = msg.axes[3]

    # button 4 is intake, button 5 is outtake
    # if we have a cube then run at 80% speed inward to hold the cube in
    if msg.axes[2] < -0.5:
        pub_intake.publish(1)
    elif msg.axes[5] < -0.5:
        pub_intake.publish(-1)
    else:
        pub_intake.publish(0)
    
    pub.publish(twist)


rospy.Subscriber("/minibot/joy", Joy, callback)

rospy.spin()