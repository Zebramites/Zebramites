#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from minimotor_msgs.msg import ArmState
from minimotor_msgs.msg import ArmFastPass
import time
# position lookup contains the position of each servo in the arm 
# [base_rotate_servo, base_servo, stage_2_arm_servo]
PositionLookup = {}
PositionLookup["score_high"] = [-5.0, 80.0, 80.0, True]
PositionLookup["score_high_cube"] = [-5.0, 80.0, 65.0, True]
PositionLookup["balance"] = [-5.0, 175.0, 165.0, False]
PositionLookup["ground_pickup"] = [-5.0, 165.0, 155.0, False]
PositionLookup["double_substation"] = [-5.0, 80.0, 40.0, True]
PositionLookup["storage"] = [-5.0, 175.0, 40.0, True]

MAX_ANGLE_MOVE = 15
STAGE_2_MAX_ANGLE_MOVE = 10
STEP_DELAY = 0.15

def sign(x: int)d:
    return 1 if x >= 0 else -1
  
class ArmManager:

    def __init__(self) -> None:
        self.sub = rospy.Subscriber("/minibot/arm_state", ArmState, self.callback, queue_size=1, tcp_nodelay=True)
        
        # wait for topic to exist before publishing
        rospy.loginfo("Waiting for arm controller topics to exist")
        #time.sleep(3)
        # just hope hwinterface is running lmao
        self.base_rotation_servo_pub = rospy.Publisher("/minibot/base_rotate_controller/command", Float64, queue_size=1, tcp_nodelay=True )
        self.base_arm_servo_pub = rospy.Publisher("/minibot/base_arm_controller/command", Float64, queue_size=1, tcp_nodelay=True )
        self.stage_2_arm_servo_pub = rospy.Publisher("/minibot/stage_two_arm_controller/command", Float64, queue_size=1, tcp_nodelay=True )
        
        self.fast_pass_pub = rospy.Publisher("/minibot/arm_fast_pass_controller/command", ArmFastPass, queue_size=1, tcp_nodelay=True)
        # need to keep track of the current position of the arm, so move to known position on startup 
        self.rotate_base_arm_angle = -5
        #self.base_rotation_servo_pub.publish(Float64(self.rotate_base_arm_angle))

        self.base_arm_joint_angle = 110
        #self.base_arm_servo_pub.publish(Float64(self.base_arm_joint_angle))

        self.stage_2_arm_joint_angle = 40
        #self.stage_2_arm_servo_pub.publish(Float64(self.stage_2_arm_joint_angle))
        self.current_position = "storage"

        self.callback(ArmState("storage", 0, 0))
        rospy.loginfo("Arm manager initialized")

    def callback(self, msg):
        rospy.logerr("Received arm state message with position: " + str(msg))
        if msg.incrementBaseArm:
            rospy.loginfo("Incrementing base arm")
            self.base_arm_joint_angle += msg.incrementBaseArm
            self.base_arm_servo_pub.publish(Float64(self.base_arm_joint_angle))
            return
        
        if msg.incrementStage2Arm:
            rospy.loginfo("Incrementing stage 2 arm")
            self.stage_2_arm_joint_angle += msg.incrementStage2Arm
            self.stage_2_arm_servo_pub.publish(Float64(self.stage_2_arm_joint_angle))
            return
        
        path = PositionLookup[msg.position]
        if path is None:
            rospy.logerr("Position not found in lookup table")
            return
        
        TMP_MAX_ANGLE_MOVE = MAX_ANGLE_MOVE

        
        rospy.loginfo("Moving arm to position: " + msg.position)
        # break up writes to the servo to make sure we don't go too fast
        # but also have all three move at the same time
        final_rotate_base_arm_angle = path[0]
        final_base_arm_joint_angle = path[1]
        final_stage_2_arm_joint_angle = path[2]

        delta_rotate_base_arm_angle = final_rotate_base_arm_angle - self.rotate_base_arm_angle
        delta_base_arm_joint_angle = final_base_arm_joint_angle - self.base_arm_joint_angle
        delta_stage_2_arm_joint_angle = final_stage_2_arm_joint_angle - self.stage_2_arm_joint_angle

        while final_stage_2_arm_joint_angle != self.stage_2_arm_joint_angle or final_base_arm_joint_angle != self.base_arm_joint_angle:
            if msg.position == "storage" and self.current_position == "double_substation":
                self.stage_2_arm_servo_pub.publish(Float64(55.0))
                self.stage_2_arm_joint_angle = final_stage_2_arm_joint_angle
            else:
                if final_stage_2_arm_joint_angle != self.stage_2_arm_joint_angle:
                    less_than_final_pos = self.stage_2_arm_joint_angle < final_stage_2_arm_joint_angle
                    greater_than_final_pos = self.stage_2_arm_joint_angle > final_stage_2_arm_joint_angle
                    self.stage_2_arm_joint_angle += STAGE_2_MAX_ANGLE_MOVE * sign(delta_stage_2_arm_joint_angle)
                    if (less_than_final_pos and self.stage_2_arm_joint_angle > final_stage_2_arm_joint_angle) or (greater_than_final_pos and self.stage_2_arm_joint_angle < final_stage_2_arm_joint_angle):
                        self.stage_2_arm_joint_angle = final_stage_2_arm_joint_angle
                    self.stage_2_arm_servo_pub.publish(Float64(self.stage_2_arm_joint_angle))
                    rospy.loginfo("Publishing to stage 2 arm = " + str(self.stage_2_arm_joint_angle))
                    time.sleep(STEP_DELAY)
                
                if final_stage_2_arm_joint_angle != self.stage_2_arm_joint_angle:
                    less_than_final_pos = self.stage_2_arm_joint_angle < final_stage_2_arm_joint_angle
                    greater_than_final_pos = self.stage_2_arm_joint_angle > final_stage_2_arm_joint_angle
                    self.stage_2_arm_joint_angle += STAGE_2_MAX_ANGLE_MOVE * sign(delta_stage_2_arm_joint_angle)
                    if (less_than_final_pos and self.stage_2_arm_joint_angle > final_stage_2_arm_joint_angle) or (greater_than_final_pos and self.stage_2_arm_joint_angle < final_stage_2_arm_joint_angle):
                        self.stage_2_arm_joint_angle = final_stage_2_arm_joint_angle
                    self.stage_2_arm_servo_pub.publish(Float64(self.stage_2_arm_joint_angle))
                    rospy.loginfo("Publishing to stage 2 arm = " + str(self.stage_2_arm_joint_angle))
                    time.sleep(STEP_DELAY)

            if final_base_arm_joint_angle != self.base_arm_joint_angle:
                less_than_final_pos = self.base_arm_joint_angle < final_base_arm_joint_angle
                greater_than_final_pos = self.base_arm_joint_angle > final_base_arm_joint_angle
                self.base_arm_joint_angle += TMP_MAX_ANGLE_MOVE * sign(delta_base_arm_joint_angle)
                if (less_than_final_pos and self.base_arm_joint_angle > final_base_arm_joint_angle) or (greater_than_final_pos and self.base_arm_joint_angle < final_base_arm_joint_angle):
                    self.base_arm_joint_angle = final_base_arm_joint_angle
                self.base_arm_servo_pub.publish(Float64(self.base_arm_joint_angle))
                rospy.loginfo("Publishing to base arm = " + str(self.base_arm_joint_angle))
                time.sleep(STEP_DELAY)

        if msg.position == "storage" and self.current_position == "double_substation":
            self.stage_2_arm_servo_pub.publish(Float64(50.0))
            time.sleep(STEP_DELAY)
            self.stage_2_arm_servo_pub.publish(Float64(45.0))
            time.sleep(STEP_DELAY)
            self.stage_2_arm_servo_pub.publish(Float64(40.0))
            time.sleep(STEP_DELAY)
            self.stage_2_arm_joint_angle = final_stage_2_arm_joint_angle

        self.current_position = msg.position


rospy.init_node('claw_manager', anonymous=True)
arm_controller = ArmManager()
rospy.spin()



