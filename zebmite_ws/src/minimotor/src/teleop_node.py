#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time
import math

class TeleopControl:

    def __init__(self):
        self.front_left_pub = rospy.Publisher("/minibot/front_left_drive_controller/command", Float64,  queue_size=1, tcp_nodelay=True )
        self.front_right_pub = rospy.Publisher("/minibot/front_right_drive_controller/command", Float64, queue_size=1, tcp_nodelay=True  )
        self.back_left_pub = rospy.Publisher("/minibot/back_left_drive_controller/command", Float64,  queue_size=1, tcp_nodelay=True )
        self.back_right_pub = rospy.Publisher("/minibot/back_right_drive_controller/command", Float64,  queue_size=1, tcp_nodelay=True )

        self.joy_sub = rospy.Subscriber("/minibot/joy", Joy, self.joy_callback, queue_size=1)
        rospy.loginfo("Teleop Control Node has Finished Initalization")
        self.rate = rospy.Rate(100)

    def send_to_all(self, speed):
        self.front_left_pub.publish(speed)
        # wait 5ms
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


    def setMecanumDrive(self, translationAngle, translationPower, turnPower):
        # calculate motor power

        # set the motors, and divide them by turningScale to make sure none of them go over the top, which would alter the translation angle
        self.front_left_pub.publish(Float64((ADPower - turningScale) / turningScale))
        self.back_left_pub.publish(Float64((BCPower - turningScale) / turningScale))
        self.front_right_pub.publish(Float64((BCPower + turningScale) / turningScale))
        self.back_right_pub.publish(Float64((ADPower + turningScale) / turningScale))
        print("ADPower: " + str(ADPower) + " BCPower: " + str(BCPower) + " turningScale: " + str(turningScale))
        print("Front Left: " + str((ADPower - turningScale) / turningScale) + " Back Left: " + str((BCPower - turningScale) / turningScale) + " Front Right: " + str((BCPower + turningScale) / turningScale) + " Back Right: " + str((ADPower + turningScale) / turningScale))

    def joy_callback(self, msg):
        # msg.axes = [left_stick_x, left_stick_y, left_trigger, right_stick_x, right_stick_y, right_trigger]
        #print(msg)
        # map input between 0.6 and 1
        # 0.6 is the minimum speed to move the robot
        # 1 is the maximum speed

        # 1 = x axis 0 = y axis
        # 2 = x axis of right stick
        y = -msg.axes[1]
        x = msg.axes[0] * 1.1
        rx = msg.axes[3]

        # Denominator is the largest motor power (absolute value) or 1
        # This ensures all the powers maintain the same ratio, but only when
        # at least one is out of the range [-1, 1]
        denominator = max(abs(y) + abs(x) + abs(rx), 1)
        frontLeftPower = (y + x + rx) / denominator
        backLeftPower = (y - x + rx) / denominator
        frontRightPower = (y - x - rx) / denominator
        backRightPower = (y + x - rx) / denominator
        self.front_left_pub.publish(Float64(frontLeftPower))
        self.back_left_pub.publish(Float64(backLeftPower))
        self.front_right_pub.publish(Float64(frontRightPower))
        self.back_right_pub.publish(Float64(backRightPower))
        #self.setMecanumDrive(angle, magnitude, twist)
        self.rate.sleep()




rospy.init_node('teleop_node', anonymous=True)
teleop = TeleopControl()
rospy.spin()




'''
public void loop() {

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive  = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist  = gamepad1.right_stick_x;
        
        /*
         * If we had a gyro and wanted to do field-oriented control, here
         * is where we would implement it.
         *
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:
         *
         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
            (drive + strafe + twist),
            (drive - strafe - twist),
            (drive - strafe + twist),
            (drive + strafe - twist)
        };

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);
'''