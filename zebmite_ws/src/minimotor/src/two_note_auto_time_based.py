#!/usr/bin/env python3
# export ROS_MASTER_URI=http://10.32.4.124:5802
import rospy
import tf2_ros
import math
from tf.transformations import euler_from_quaternion
import actionlib
from geometry_msgs.msg import Twist
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import Joy
from minimotor_msgs.msg import DriveToPointAction, DriveToPointGoal, DriveToPointResult, DriveToPointFeedback
from minimotor_msgs.msg import ShootAction, ShootGoal, ShootResult, ShootFeedback
from minimotor_msgs.msg import IntakeAction, IntakeGoal, IntakeResult, IntakeFeedback
import time

# "/goal" with geometry_msgs/PoseStamped, frame_id = "tag_19"

# In front of left note
'''
buntu@dino-laptop:~$ rostopic pub /goal geometry_msgs/PoseStamped "header:
    seq: 0
    stamp:
        secs: 0
        nsecs: 0
    frame_id: 'tag_19'
pose:
    position:
        x: -0.38
        y: -0.25
        z: 0.0
    orientation:
        x: 0.0
        y: 0.0
        z: 0.0'''

# "home"
'''
    seq: 0
    stamp:
        secs: 0
        nsecs: 0
    frame_id: 'tag_19'
pose:
    position:
        x: 0.0
        y: -0.1
        z: 0.0
    orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 0.0" -1''' 

ac_intake = actionlib.SimpleActionClient("/minibot/intake_server", IntakeAction)
ac_shoot = actionlib.SimpleActionClient("/minibot/shoot_server", ShootAction)
cmd_pub = rospy.Publisher('/minibot/mecanum_drive_controller/cmd_vel', Twist, queue_size=10)

DRIVE_FORWARD_TIME = 2.5
MAX_TIME = 1.5

def joy_cb(msg: Joy):
    if not msg.buttons[-3]:
        return
    
    # Shoot preload
    rospy.loginfo("auto: shooting preload")
    s = ShootGoal()
    s.dont_spin_up = False
    s.dont_shoot = False
    s.dont_spin_down = False

    done = False
    def done_callback(state, result):
        nonlocal done
        rospy.loginfo(f"shot note with state {state} and result {result}")
        done = True
    
    r = rospy.Rate(50)
    ac_shoot.send_goal(s, done_cb=done_callback)

    while not done and not rospy.is_shutdown():
        r.sleep()
    
    done = False
    def done_callback(state, result):
        nonlocal done
        rospy.loginfo(f"shot note with state {state} and result {result}")
        done = True

    time.sleep(0.5)

    # Activate intake
    rospy.loginfo("auto: intaking")
    i = IntakeGoal()
    ac_intake.send_goal(i, done_cb=done_callback)

    start = time.time()

    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 1.0
    twist.angular.z = 0.0

    while not done and not rospy.is_shutdown() and (not ((time.time() - start) > MAX_TIME)):
        cmd_pub.publish(twist)
        r.sleep()
    
    start = time.time()

    rospy.loginfo("auto: driving back")

    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = -1.0
    twist.angular.z = 0.0

    while not rospy.is_shutdown() and (not ((time.time() - start) > DRIVE_FORWARD_TIME)):
        cmd_pub.publish(twist)
        r.sleep()
    
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.angular.z = 0.0
    cmd_pub.publish(twist)
    
    rospy.loginfo("auto: shooting")
    # Shoot picked up note
    s = ShootGoal()
    s.dont_spin_up = False
    s.dont_shoot = False
    s.dont_spin_down = False

    ac_shoot.send_goal(s)

#rospy.Subscriber("/minibot/joy_operator", Joy, operator_callback)
# rospy.Subscriber("/minibot/joy", Joy, joy_cb)
rospy.init_node('auto_node', anonymous=True)

buttons = [1,1,1,1,1,1,1,1,1]
msg = Joy(buttons=buttons)
joy_cb(msg)

# rospy.spin()