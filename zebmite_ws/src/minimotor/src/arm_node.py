#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from minimotor_msgs.msg import ArmState
import time
# position lookup contains the position of each servo in the arm 
# [base_rotate_servo, base_servo, stage_2_arm_servo]
PositionLookup = {}
PositionLookup["score_high"] = [-5.0, 120.0, 100.0]
PositionLookup["score_mid"] = None
PositionLookup["score_low"] = None
PositionLookup["double_substation"] = None
PositionLookup["single_substation"] = None
PositionLookup["storage"] = [-5.0, 71.0, 180.0]

MAX_ANGLE_MOVE = 3
STEP_DELAY = 0.02

def sign(x):
    return 1 if x >= 0 else -1

class ArmManager:

    def __init__(self) -> None:
        self.sub = rospy.Subscriber("/minibot/arm_state", ArmState, self.callback, queue_size=1)
        
        # wait for topic to exist before publishing
        rospy.loginfo("Waiting for arm controller topics to exist")
        time.sleep(3)
        # just hope hwinterface is running lmao
        self.base_rotation_servo_pub = rospy.Publisher("/minibot/base_rotate_controller/command", Float64, queue_size=1, tcp_nodelay=True )
        self.base_arm_servo_pub = rospy.Publisher("/minibot/base_arm_controller/command", Float64, queue_size=1, tcp_nodelay=True )
        self.stage_2_arm_servo_pub = rospy.Publisher("/minibot/stage_two_arm_controller/command", Float64, queue_size=1, tcp_nodelay=True )
        
        time.sleep(1)
        # need to keep track of the current position of the arm, so move to known position on startup 
        self.rotate_base_arm_angle = -5
        #self.base_rotation_servo_pub.publish(Float64(self.rotate_base_arm_angle))

        self.base_arm_joint_angle = 90
        #self.base_arm_servo_pub.publish(Float64(self.base_arm_joint_angle))

        self.stage_2_arm_joint_angle = 45
        #self.stage_2_arm_servo_pub.publish(Float64(self.stage_2_arm_joint_angle))
        rospy.loginfo("Arm manager initialized")

    def callback(self, msg):
        path = PositionLookup[msg.position]
        if path is None:
            rospy.logerr("Position not found in lookup table")
            return
        
        rospy.loginfo("Moving arm to position: " + msg.position)
        # break up writes to the servo to make sure we don't go too fast
        # but also have all three move at the same time
        final_rotate_base_arm_angle = path[0]
        final_base_arm_joint_angle = path[1]
        final_stage_2_arm_joint_angle = path[2]

        delta_rotate_base_arm_angle = final_rotate_base_arm_angle - self.rotate_base_arm_angle
        delta_base_arm_joint_angle = final_base_arm_joint_angle - self.base_arm_joint_angle
        delta_stage_2_arm_joint_angle = final_stage_2_arm_joint_angle - self.stage_2_arm_joint_angle

        while final_rotate_base_arm_angle != self.rotate_base_arm_angle or final_base_arm_joint_angle != self.base_arm_joint_angle or final_stage_2_arm_joint_angle != self.stage_2_arm_joint_angle:
            if final_rotate_base_arm_angle != self.rotate_base_arm_angle:
                less_than_final_pos = self.rotate_base_arm_angle < final_rotate_base_arm_angle
                greater_than_final_pos = self.rotate_base_arm_angle > final_rotate_base_arm_angle
                self.rotate_base_arm_angle += MAX_ANGLE_MOVE * sign(delta_rotate_base_arm_angle)
                if (less_than_final_pos and self.rotate_base_arm_angle > final_rotate_base_arm_angle) or (greater_than_final_pos and self.rotate_base_arm_angle < final_rotate_base_arm_angle):
                    self.rotate_base_arm_angle = final_rotate_base_arm_angle
                self.base_rotation_servo_pub.publish(Float64(self.rotate_base_arm_angle))
            
            if final_base_arm_joint_angle != self.base_arm_joint_angle:
                less_than_final_pos = self.base_arm_joint_angle < final_base_arm_joint_angle
                greater_than_final_pos = self.base_arm_joint_angle > final_base_arm_joint_angle
                self.base_arm_joint_angle += MAX_ANGLE_MOVE * sign(delta_base_arm_joint_angle)
                if (less_than_final_pos and self.base_arm_joint_angle > final_base_arm_joint_angle) or (greater_than_final_pos and self.base_arm_joint_angle < final_base_arm_joint_angle):
                    self.base_arm_joint_angle = final_base_arm_joint_angle
                self.base_arm_servo_pub.publish(Float64(self.base_arm_joint_angle))
            
            if final_stage_2_arm_joint_angle != self.stage_2_arm_joint_angle:
                less_than_final_pos = self.stage_2_arm_joint_angle < final_stage_2_arm_joint_angle
                greater_than_final_pos = self.stage_2_arm_joint_angle > final_stage_2_arm_joint_angle
                self.stage_2_arm_joint_angle += MAX_ANGLE_MOVE * sign(delta_stage_2_arm_joint_angle)
                if (less_than_final_pos and self.stage_2_arm_joint_angle > final_stage_2_arm_joint_angle) or (greater_than_final_pos and self.stage_2_arm_joint_angle < final_stage_2_arm_joint_angle):
                    self.stage_2_arm_joint_angle = final_stage_2_arm_joint_angle
                self.stage_2_arm_servo_pub.publish(Float64(self.stage_2_arm_joint_angle))
                
            rospy.sleep(STEP_DELAY)


rospy.init_node('claw_manager', anonymous=True)
arm_controller = ArmManager()
rospy.spin()



