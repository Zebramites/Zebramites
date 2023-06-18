#!/usr/bin/env python3
# make a service that takes in a bool and runs the auto code 

import rospy
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

'''      
def callback(msg):
    if msg.buttons[6] == 1:

        rospy.loginfo("Auto mode activated")
        # run auto code
        # publish to cmd_vel and intake_controller/comman
        # make a rate and publish based on it
        r = rospy.Rate(100)
        
        t = Twist()
        t.linear.x = -1.3
        t.angular.z = 0
        starttime = time.time()
        while time.time() - starttime < 2:
            pub.publish(t)
        t.linear.x = 1.3
        t.angular.z = 12
        starttime = time.time()
        while time.time() - starttime < 1.5:
            pub.publish(t)
        t.linear.x = -1.3
        t.angular.z = 0
        starttime = time.time()
        while time.time() - starttime < 1.5:
            pub.publish(t)
        t.linear.x = 0
        t.angular.z = 0
        pub.publish(t)

rospy.init_node('auto_node', anonymous=True)

pub = rospy.Publisher("/minibot/diffbot_controller/cmd_vel", Twist)
rospy.Subscriber("/minibot/joy", Joy, callback, queue_size=1)

rospy.spin()
'''
