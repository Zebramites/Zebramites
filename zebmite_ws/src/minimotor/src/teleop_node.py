#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time
import math
from minimotor_msgs.msg import RPYEuler
from minimotor_msgs.msg import ArmState
from minimotor_msgs.msg import ClawState

MAX_MOTOR_WHEN_EXTENDED = 0.8

class TeleopControl:

    def __init__(self):
        self.front_left_pub = rospy.Publisher("/minibot/front_left_drive_controller/command", Float64,  queue_size=1, tcp_nodelay=True )
        self.front_right_pub = rospy.Publisher("/minibot/front_right_drive_controller/command", Float64, queue_size=1, tcp_nodelay=True  )
        self.back_left_pub = rospy.Publisher("/minibot/back_left_drive_controller/command", Float64,  queue_size=1, tcp_nodelay=True )
        self.back_right_pub = rospy.Publisher("/minibot/back_right_drive_controller/command", Float64,  queue_size=1, tcp_nodelay=True )

        self.joy_sub = rospy.Subscriber("/minibot/joy", Joy, self.joy_callback, queue_size=1)
        self.current_angle = 0
        self.imu_sub = rospy.Subscriber("/minibot/imu_state", RPYEuler, self.imu_callback, queue_size=1)
        
        self.arm_pub = rospy.Publisher("/minibot/arm_state", ArmState, queue_size=1, tcp_nodelay=True)
        self.claw_pub = rospy.Publisher("/minibot/claw_state", ClawState, queue_size=1, tcp_nodelay=True)
        self.arm_extended = False
        self.claw_open = False

        self.RB_pressed = False
        self.RT_pressed = False
        self.LB_pressed = False
        self.LT_pressed = False
        self.A_pressed = False
        self.B_pressed = False
        self.X_pressed = False
        self.Y_pressed = False
        self.DPAD_up_pressed = False
        self.DPAD_down_pressed = False
        self.DPAD_left_pressed = False
        self.DPAD_right_pressed = False

        rospy.loginfo("Teleop Control Node has Finished Initalization")

    def imu_callback(self, msg):
        # normal
        self.current_angle = msg.yaw

    def send_to_all(self, speed):
        self.front_left_pub.publish(speed)
        self.front_right_pub.publish(speed)
        self.back_left_pub.publish(speed)
        self.back_right_pub.publish(speed)
        #rospy.loginfo("Sent Speed to all Motors: " + str(speed))

    def sign(self, x):
        if x > 0:
            return 1
        elif x < 0:
            return -1
        else:
            return 0
    
    def remap(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def apply_deadband(self, angle):
        if -10 <= angle <= 10:
            return 0 
        elif 80 <= angle <= 100:
            return 90  
        elif -100 <= angle <= -80:
            return -90 
        elif 170 <= angle or angle <= -170:
            return 180
        else:
            return angle 

    def joy_callback(self, msg):
        # msg.axes = [left_stick_x, left_stick_y, left_trigger, right_stick_x, right_stick_y, right_trigger]
        A_button = msg.buttons[0] 
        B_button = msg.buttons[1]
        X_button = msg.buttons[2] 
        Y_button = msg.buttons[3] 
        LB = msg.buttons[4]
        RB = msg.buttons[5]


            
        

        #print(msg)
        # map input between 0.6 and 1
        # 0.6 is the minimum speed to move the robot
        # 1 is the maximum speed

        # 1 = x axis 0 = y axis
        # 2 = x axis of right stick
        new_y = -msg.axes[1]
        new_x = msg.axes[0] * 1.1
        rx = msg.axes[3]
        
        ''' 
        angle_rad = math.atan2(y, x)
        print("Angle: " + str(math.degrees(angle_rad)))
        adjusted_angle = self.apply_deadband(math.degrees(angle_rad))
        adjusted_angle_rad = math.radians(adjusted_angle)

        new_x = math.sin(adjusted_angle_rad)
        new_y = -math.cos(adjusted_angle_rad)
        '''
        # Denominator is the largest motor power (absolute value) or 1
        # This ensures all the powers maintain the same ratio, but only when
        # at least one is out of the range [-1, 1]
        denominator = max(abs(new_y) + abs(new_x) + abs(rx), 1)
        frontLeftPower = (new_y + new_x + rx) / denominator
        backLeftPower = (new_y - new_x + rx) / denominator
        frontRightPower = (new_y - new_x - rx) / denominator
        backRightPower = (new_y + new_x - rx) / denominator

        # scale all outputs to have abs(output) >= 0.5 but keep the sign 
        # This ensures the robot will move at all
        frontLeftPower = self.sign(frontLeftPower) * self.remap(abs(frontLeftPower), 0, 1, 0.5, 1) 
        backLeftPower = self.sign(backLeftPower) * self.remap(abs(backLeftPower), 0, 1, 0.5, 1)
        frontRightPower = self.sign(frontRightPower) * self.remap(abs(frontRightPower), 0, 1, 0.5, 1)
        backRightPower = self.sign(backRightPower) * self.remap(abs(backRightPower), 0, 1, 0.5, 1)
    
        
        if msg.axes[-1] == 1 and not self.DPAD_up_pressed: # dpad up
            self.DPAD_up_pressed = True
            rospy.loginfo("[Teleop] Incrementing Arm Up")
            arm_msg = ArmState()
            arm_msg.incrementBaseArm = 5
            self.arm_pub.publish(arm_msg)
        
        if msg.axes[-1] == 0 and self.DPAD_up_pressed:
            self.DPAD_up_pressed = False
        
        if msg.axes[-1] == -1 and not self.DPAD_down_pressed: # dpad down
            self.DPAD_down_pressed = True
            rospy.loginfo("[Teleop] Incrementing Arm Down")
            arm_msg = ArmState()
            arm_msg.incrementBaseArm = -5
            self.arm_pub.publish(arm_msg)

        if msg.axes[-1] == 0 and self.DPAD_down_pressed:
            self.DPAD_down_pressed = False

        if msg.axes[-2] == 1 and not self.DPAD_left_pressed: # dpad left
            self.DPAD_left_pressed = True
            rospy.loginfo("[Teleop] Incrementing Arm Left")
            arm_msg = ArmState()
            arm_msg.incrementStage2Arm = 5
            self.arm_pub.publish(arm_msg)

        if msg.axes[-2] == 0 and self.DPAD_left_pressed:
            self.DPAD_left_pressed = False

        if msg.axes[-2] == -1 and not self.DPAD_right_pressed: # dpad right
            self.DPAD_right_pressed = True
            rospy.loginfo("[Teleop] Incrementing Arm Right")
            arm_msg = ArmState()
            arm_msg.incrementStage2Arm = -5
            self.arm_pub.publish(arm_msg)

        if msg.axes[-2] == 0 and self.DPAD_right_pressed:
            self.DPAD_right_pressed = False
            

        if A_button == 1 and self.A_pressed == False:
            self.A_pressed = True
            rospy.loginfo("[Teleop] Extending Arm")
            arm_msg = ArmState()
            arm_msg.position = "double_substation"
            self.arm_pub.publish(arm_msg)


        if A_button == 0 and self.A_pressed == True: 
            self.A_pressed = False

        if Y_button == 1 and self.Y_pressed == False:
            self.Y_pressed = True
            
        
        if Y_button == 0 and self.Y_pressed == True:
            self.Y_pressed = False

        # open/close claw 
        if RB and not self.RB_pressed:
            self.RB_pressed = True
            rospy.loginfo("[Teleop] Inverting Claw")
            claw_msg = ClawState()
            claw_msg.just_invert = True
            self.claw_pub.publish(claw_msg)
        
        if not RB and self.RB_pressed:
            self.RB_pressed = False

        if msg.axes[5] < -0.5 and not self.RT_pressed:
            self.RT_pressed = True
            # extend arm and open claw
            rospy.loginfo("[Teleop] Extending Arm")
            arm_msg = ArmState()
            arm_msg.position = "score_high"
            
            self.arm_pub.publish(arm_msg)
            '''
            claw_msg = ClawState()
            claw_msg.open_claw = True
            self.claw_pub.publish(claw_msg)
            '''
        
        if msg.axes[5] > 0.9 and self.RT_pressed:
            self.RT_pressed = False
            # retract arm
            rospy.loginfo("[Teleop] Closing claw and retracting Arm")
            arm_msg = ArmState()
            arm_msg.position = "storage"
            self.arm_pub.publish(arm_msg)
            ''' 
            claw_msg = ClawState()
            claw_msg.open_claw = False
            self.claw_pub.publish(claw_msg)
            '''


        self.front_left_pub.publish(Float64(frontLeftPower))
        self.back_left_pub.publish(Float64(backLeftPower))
        self.front_right_pub.publish(Float64(frontRightPower))
        self.back_right_pub.publish(Float64(backRightPower))
        
        print("FL: " + str(frontLeftPower) + " BL: " + str(backLeftPower) + " FR: " + str(frontRightPower) + " BR: " + str(backRightPower))




rospy.init_node('teleop_node', anonymous=True)
teleop = TeleopControl()
rospy.spin()

