#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


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
    if msg.buttons[4] == 1:
        pub_intake.publish(1)
    elif msg.buttons[5] == 1:
        pub_intake.publish(-1)
    else:
        pub_intake.publish(0)
    
    pub.publish(twist)


rospy.Subscriber("/minibot/joy", Joy, callback)

rospy.spin()