#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import moveit_commander
import sys
import math
import time
from minimotor_msgs.msg import ShootAction, ShootGoal, ShootResult, ShootFeedback
from minimotor_msgs.msg import IntakeAction, IntakeGoal, IntakeResult, IntakeFeedback
import actionlib

# 900 comp joystick: Bus 003 Device 055: ID 045e:028e Microsoft Corp. Xbox360 Controller
# Xbox controller: Bus 003 Device 045: ID 045e:02ea Microsoft Corp. Xbox One S Controller
ANGULAR_SPEED_MULTIPLIER = 0.8
distance_value = 0
rospy.init_node('teleop_node', anonymous=True)

pub = rospy.Publisher("/minibot/mecanum_drive_controller/cmd_vel", Twist, queue_size=1)

intake_pub = rospy.Publisher("/minibot/intake_controller/command", Float64, queue_size=1) 
top_intake_pub = rospy.Publisher("/minibot/intake_top_controller/command", Float64, queue_size=1)
roof_roller_pub = rospy.Publisher("/minibot/roof_roller_controller/command", Float64, queue_size=1)
ac_shoot = actionlib.SimpleActionClient("/minibot/shoot_server", ShootAction)
ac_intake = actionlib.SimpleActionClient("/minibot/intake_server", IntakeAction)

shooter_pub = rospy.Publisher("/minibot/shooter_controller/command", Float64, queue_size=1)


def has_note(distance: Float64):
    if distance < 26:
        return True
    else:
        return False 

def dist_sensor_callback(msg):
    global distance_value
    distance_value = msg.data

last_shoot_trigger_state = False
last_intake_state = False

def callback(msg):
    global intake_at_low_speed, LOW_INTAKE_SPEED, ANGULAR_SPEED_MULTIPLIER, distance_value, last_shoot_trigger_state, last_intake_state
    # publish cmd velocity with joystick input
    twist = Twist()
    twist.linear.x = -msg.axes[0]
    twist.linear.y = -msg.axes[1]
    ROT_AXIS = 2
    twist.angular.z = -1 * (-1 if msg.axes[ROT_AXIS] > 0 else 1) * ANGULAR_SPEED_MULTIPLIER * min(1, msg.axes[ROT_AXIS] ** 2 + (0.5 if msg.axes[ROT_AXIS] > 0 else 0))

    rospy.loginfo(msg.buttons[8])
    shoot_trigger = msg.buttons[7] > 0
    if shoot_trigger and not last_shoot_trigger_state:
        goal = ShootGoal()
        ac_shoot.send_goal(goal)
    last_shoot_trigger_state = shoot_trigger

    intake_trigger = msg.buttons[8] > 0
    if intake_trigger and not last_intake_state:
        goal = IntakeGoal()
        ac_intake.send_goal(goal)
    if last_intake_state and not intake_trigger:
        ac_intake.cancel_all_goals()
    last_intake_state = intake_trigger
    pub.publish(twist)

rospy.Subscriber("/minibot/joy", Joy, callback)


rospy.spin()
