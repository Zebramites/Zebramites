#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import moveit_commander
import sys

# 900 comp joystick: Bus 003 Device 055: ID 045e:028e Microsoft Corp. Xbox360 Controller
# Xbox controller: Bus 003 Device 045: ID 045e:02ea Microsoft Corp. Xbox One S Controller

rospy.init_node('teleop_node', anonymous=True)

pub = rospy.Publisher("/minibot/mecanum_drive_controller/cmd_vel", Twist)
pub_intake = rospy.Publisher("/minibot/intake_controller/command", Float64)
pub_auto = rospy.Publisher("/minibot/auto_controller/command", Float64)

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("armplangroup")

def move_with_moveit(path_name: str):
    group.stop()
    group.clear_pose_targets()
    rospy.loginfo(f"Moving to {path_name} position")
    group.set_named_target(path_name)
    plan = group.go(wait=False)

ANGULAR_SPEED_MULTIPLIER = 2

def callback(msg):
    global intake_at_low_speed, LOW_INTAKE_SPEED, ANGULAR_SPEED_MULTIPLIER
    # publish cmd velocity with joystick input
    twist = Twist()
    twist.linear.x = msg.axes[0]
    twist.linear.y = msg.axes[1]
    twist.angular.z = (-1 if msg.axes[3] > 0 else 1) * msg.axes[3] ** ANGULAR_SPEED_MULTIPLIER

    # button 4 is intake, button 5 is outtake
    # if we have a cube then run at 80% speed inward to hold the cube in
    if msg.axes[2] < -0.5:
        pub_intake.publish(1)
    elif msg.axes[5] < -0.5:
        pub_intake.publish(-1)
        intake_at_low_speed = False
    elif intake_at_low_speed:
        pub_intake.publish(LOW_INTAKE_SPEED)
    else:
        pub_intake.publish(0)
    if msg.buttons[0] == 1:
        move_with_moveit("rest")
    
    pub.publish(twist)

LOW_INTAKE_SPEED = 0.8
intake_at_low_speed = False
op_cube_button_pressed = False
op_three_lines_button_pressed = False
op_A_pressed = False
op_B_pressed = False
op_X_pressed = False
op_Y_pressed = False
op_LT_pressed = False
op_RT_pressed = False
op_dpad_down_pressed = False
op_dpad_left_pressed = False
op_dpad_right_pressed = False

RED_TIME = 1000
BLUE_TIME = 500

def operator_callback(msg):
    global intake_at_low_speed, op_cube_button_pressed, op_A_pressed, op_B_pressed, op_X_pressed, op_Y_pressed, op_LT_pressed, op_RT_pressed, op_dpad_down_pressed, op_dpad_left_pressed, op_dpad_right_pressed, op_three_lines_button_pressed

    # Place high -> Y
    # Place mid -> B
    # Place low -> A

    # Intake double substation -> RT
    # Intake floor -> LT
    # Intake floor cube -> three lines button

    # Rest -> X

    # Low speed toggle -> cube shaped button

    # Dpad down -> moveit zero

    # AUTOS: dpad left = red, dpad right = blue
    
    # button mapping 
    # 0: A, 1: B, 2: X, 3: Y, 4: LB, 5: RB, 6: Back (cube shaped), 7: Start (three lines button), 8: Xbox, 9: Left joystick, 10: Right joystick
    
    # Left trigger -> intake from ground
    if msg.axes[2] < -0.5 and not op_LT_pressed:
        move_with_moveit("floor_intake")
    op_LT_pressed = msg.axes[2] < -0.5
    
    # Right trigger -> intake from double substation
    if msg.axes[5] < -0.5 and not op_RT_pressed:
        move_with_moveit("double_substation_intake")
    op_RT_pressed = msg.axes[5] < -0.5

    # ------------------------------------------------------------------------

    # cube shaped button -> toggle low speed holding
    if msg.buttons[6] and not op_cube_button_pressed:
        intake_at_low_speed = not intake_at_low_speed
        rospy.loginfo(f"Intake at low speed: {intake_at_low_speed}")
    op_cube_button_pressed = msg.buttons[6]

    # ------------------------------------------------------------------------

    # Three lines button -> intake from ground cube
    if msg.buttons[7] and not op_three_lines_button_pressed:
        move_with_moveit("floor_cube")
    op_three_lines_button_pressed = msg.buttons[7]

    # Y -> high arm position
    if msg.buttons[3] and not op_Y_pressed:
        move_with_moveit("score_high")
    op_Y_pressed = msg.buttons[3]

    # B -> Place mid
    if msg.buttons[1] and not op_B_pressed:
        move_with_moveit("score_mid")
    op_B_pressed = msg.buttons[1]
    
    # A -> low arm position
    if msg.buttons[0] and not op_A_pressed:
        move_with_moveit("rest") # TODO find low position
    op_A_pressed = msg.buttons[0]

    # X -> rest position
    if msg.buttons[2] and not op_X_pressed:
        move_with_moveit("rest")
    op_X_pressed = msg.buttons[2]

    # Dpad down -> moveit zero
    if msg.axes[7] == -1 and not op_dpad_down_pressed:
        move_with_moveit("moveit_zero")
    op_dpad_down_pressed = msg.axes[7] == -1

    # ------------------------------------------------------------------------

    # Dpad left -> red auto
    if msg.axes[6] == 1 and not op_dpad_left_pressed:
        pub_auto.publish(RED_TIME) # drive back
    op_dpad_left_pressed = msg.axes[6] == 1
    
    # Dpad right -> blue auto
    if msg.axes[6] == -1 and not op_dpad_right_pressed:
        pub_auto.publish(BLUE_TIME) # drive back
    op_dpad_right_pressed = msg.axes[6] == -1

rospy.Subscriber("/minibot/joy_operator", Joy, operator_callback)
rospy.Subscriber("/minibot/joy", Joy, callback)

rospy.spin()

moveit_commander.roscpp_shutdown()
moveit_commander.os._exit(0)