#!/usr/bin/env python3
# make a service that takes in a bool and runs the auto code 

import rospy
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from minimotor_msgs.msg import ArmState, RPYEuler, ClawState
# std_srvs/Empty
from std_srvs.srv import Empty
from sensor_msgs.msg import Joy
# angles package
import angles
import math

P_CONSTANT = 0.1

class AutoControl:

    def __init__(self) -> None:
        self.front_left_pub = rospy.Publisher("/minibot/front_left_drive_controller/command", Float64,  queue_size=1, tcp_nodelay=True )
        self.front_right_pub = rospy.Publisher("/minibot/front_right_drive_controller/command", Float64, queue_size=1, tcp_nodelay=True  )
        self.back_left_pub = rospy.Publisher("/minibot/back_left_drive_controller/command", Float64,  queue_size=1, tcp_nodelay=True )
        self.back_right_pub = rospy.Publisher("/minibot/back_right_drive_controller/command", Float64,  queue_size=1, tcp_nodelay=True )
        self.joy_sub = rospy.Subscriber("/minibot/joy", Joy, self.callback, queue_size=1)
        self.arm_pub = rospy.Publisher("/minibot/arm_state", ArmState, queue_size=1, tcp_nodelay=True)
        self.claw_pub = rospy.Publisher("/minibot/claw_state", ClawState, queue_size=1, tcp_nodelay=True)

        self.current_pitch = 0
        self.current_yaw = 0
        self.wanted_yaw = None
        self.imu_sub = rospy.Subscriber("/minibot/imu_state", RPYEuler, self.imu_callback, queue_size=1)
        # make two service clients for starting and stopping imu
        self.imu_start_client = rospy.ServiceProxy("/imu/start_imu_reads", Empty)
        self.imu_stop_client = rospy.ServiceProxy("/imu/stop_imu_reads", Empty)
        self.joy_pub = rospy.Publisher("/minibot/joy", Joy, queue_size=1, tcp_nodelay=True)

        # wait for services to be available
        rospy.loginfo("Auto node waiting for imu services")
        rospy.wait_for_service("/imu/start_imu_reads")
        rospy.wait_for_service("/imu/stop_imu_reads")
        rospy.loginfo("Auto node successfully initialized")
    

    def imu_callback(self, msg): 
        pitch = msg.pitch

        self.current_pitch = msg.pitch
        
        self.current_yaw = msg.yaw
        ROTATE_P_CONSTANT = (1/(180*1)) * -1
        if self.wanted_yaw:
            print("Shortest angular distance is " + str(math.degrees(angles.shortest_angular_distance(math.radians(self.current_yaw), math.radians(self.wanted_yaw)))))
            print("Auto node yaw " + str(self.current_yaw) + " wanted yaw " + str(self.wanted_yaw))
            if math.degrees(angles.shortest_angular_distance(math.radians(self.current_yaw), math.radians(self.wanted_yaw))) > 0:
                effort = 0.3
            else:
                effort = -0.3
            joy_msg = Joy()
            joy_msg.axes = [0, 0, 0, effort, 0, 0, 0, 0]
            joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
            self.joy_pub.publish(joy_msg)

            if abs(math.degrees(angles.shortest_angular_distance(math.radians(self.current_yaw), math.radians(self.wanted_yaw)))) < 3:
                print("Shortest angular distance is less than 5 degrees, stopping")
                print("Auto node yaw " + str(self.current_yaw) + " wanted yaw " + str(self.wanted_yaw))
                print("shortest angular distance " + str(math.degrees(angles.shortest_angular_distance(math.radians(self.current_yaw), math.radians(self.wanted_yaw)))))
                self.wanted_yaw = None
                rospy.loginfo("=======Auto node finished rotating 180 degrees=====")
                final_joystick_msg = Joy()
                final_joystick_msg.axes = [0, 0, 0, 0, 0, 0, 0, 0]
                final_joystick_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
                self.joy_pub.publish(final_joystick_msg)

        rospy.loginfo_throttle(2, "[THROTTLED] Auto node current pitch: " + str(self.current_pitch))

    def send_to_all(self, speed):
        self.front_left_pub.publish(speed)
        self.front_right_pub.publish(speed)
        self.back_left_pub.publish(speed)
        self.back_right_pub.publish(speed)
        rospy.loginfo("Auto node sending speed to all Motors: " + str(speed))        
    
    def place_high_cone(self):
        msg = ArmState()
        msg.position = "score_high"
        self.arm_pub.publish(msg)
        rospy.loginfo("Auto node placing high cone")
        rospy.sleep(3)
        msg = ClawState()
        msg.open_claw = True
        self.claw_pub.publish(msg)
        rospy.loginfo("Auto node opening claw")

    def retract_arm(self):
        msg = ArmState()
        msg.position = "balance"
        self.arm_pub.publish(msg)
        rospy.loginfo("Auto node retracting arm")

    def drive_until_pitch(self, speed=-900, pitch=-900):
        assert speed != -900 and pitch != -900, "Auto node drive_until_pitch requires speed and pitch"
        self.send_to_all(speed)
        r = rospy.Rate(100)
        while self.current_pitch > pitch:
            self.send_to_all(speed)
            rospy.loginfo(f"Pitch {self.current_pitch} wanted pitch {pitch}")
            r.sleep()
        rospy.loginfo("Auto node reached pitch: " + str(pitch))
        self.send_to_all(0)

    def drive_for_time(self, speed=-900, seconds=-900):
        assert speed != -900 and seconds != -900, "Auto node drive_for_time requires speed and time"
        start = rospy.Time.now()
        while (rospy.Time.now() - start < rospy.Duration(seconds)):
            self.send_to_all(speed)
        self.send_to_all(0)
        rospy.loginfo("Auto node drove for seconds: " + str(seconds))
    
    def drive_for_time_exit_if_0(self, speed=-900, seconds=-900):
        assert speed != -900 and seconds != -900, "Auto node drive_for_time requires speed and time"
        start = rospy.Time.now()
        while (rospy.Time.now() - start < rospy.Duration(seconds)):
            self.send_to_all(speed)
            if (abs(self.current_pitch < 4)):
                rospy.logerr("Found pitch stopping!")
                break
        self.send_to_all(0)


    def pid_drive_to_pitch(self, pitch=-900):
        assert pitch != -900, "Auto node pid_drive_to_pitch requires pitch"
        self.send_to_all(0)
        rospy.loginfo("Auto node pid driving to pitch: " + str(pitch))
        r = rospy.Rate(50)
        while abs(self.current_pitch - pitch) > 2.5:
            self.send_to_all((self.current_pitch - pitch) * P_CONSTANT * -1)
            r.sleep()
        self.send_to_all(0)
        rospy.loginfo("Auto node reached pitch: " + str(pitch))

    def start_imu(self):
        self.imu_start_client()
        rospy.loginfo("Auto node starting imu")

    def stop_imu(self):
        self.imu_stop_client()
        rospy.loginfo("Auto node stopping imu")

    def rotate_180_PID(self):
        wanted_yaw = self.current_yaw + 180
        # normalize to 0, 360
        if wanted_yaw > 360:
            wanted_yaw = wanted_yaw - 360
        if wanted_yaw < 0:
            wanted_yaw = wanted_yaw + 360
        self.wanted_yaw = wanted_yaw

    def rotate_180_timings(self):
        rospy.loginfo("Auto node rotating 180 degrees")
        joy_msg = Joy()
        joy_msg.axes = [0, 0, 0, 0.37, 0, 0, 0, 0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        s = rospy.Time.now()
        while (rospy.Time.now() - s < rospy.Duration(1.8)):
            self.joy_pub.publish(joy_msg)
        for i in range(10):
            joy_msg.axes = [0, 0, 0, 0, 0, 0, 0, 0]
            joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
            self.joy_pub.publish(joy_msg)


    def callback(self, msg):
        ''' 
        if msg.buttons[0] == 1:
            rospy.logwarn("[Auto node] disabling imu")
            self.stop_imu()
        if msg.buttons[1] == 1:
            rospy.logwarn("[Auto node] enabling imu")
            self.start_imu()
        '''
        if msg.buttons[6] == 1:
            self.start_imu()
            #self.place_high_cone()
            self.retract_arm()
            time.sleep(1)
            #self.drive_for_time(speed=0.7, seconds=0.5)
            self.rotate_180_timings()
            time.sleep(1)
            self.drive_until_pitch(speed=-1, pitch=-20) # get over charge station
            self.drive_for_time(speed=-0.6, seconds=0.3)
            self.stop_imu()

            ''' 
            rospy.loginfo("[Auto node] running place and auto balance")

            rospy.sleep(2)
            exit()
            
            self.start_imu()

            self.drive_until_pitch(speed=-1, pitch=-30) # get over charge station
            self.drive_for_time(speed=-1, seconds=0.2) # get those mobility points
            exit()
            self.drive_for_time(speed=-1, seconds=2) # start getting back on charging station
            self.pid_drive_to_pitch(pitch=0) # get back on charging station
            self.stop_imu()
            rospy.loginfo("[Auto node] finished place and auto balance")
            '''
            

rospy.init_node('auto_node', anonymous=True)
auto = AutoControl()
rospy.spin()
