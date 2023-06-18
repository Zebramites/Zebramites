#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import time

class TeleopControl:

    def __init__(self):
        self.front_left_pub = rospy.Publisher("/minibot/front_left_drive_controller/command", Float64,  queue_size=1, tcp_nodelay=True )
        self.front_right_pub = rospy.Publisher("/minibot/front_right_drive_controller/command", Float64, queue_size=1, tcp_nodelay=True  )
        self.back_left_pub = rospy.Publisher("/minibot/back_left_drive_controller/command", Float64,  queue_size=1, tcp_nodelay=True )
        self.back_right_pub = rospy.Publisher("/minibot/back_right_drive_controller/command", Float64,  queue_size=1, tcp_nodelay=True )

        self.joy_sub = rospy.Subscriber("/minibot/joy", Joy, self.joy_callback, queue_size=1)
        rospy.loginfo("Teleop Control Node has Finished Initalization")
        self.rate = rospy.Rate(2)

    def send_to_all(self, speed):
        self.front_left_pub.publish(speed)
        # wait 5ms
        
        time.sleep(0.01)
        self.front_right_pub.publish(speed)
        time.sleep(0.01)
        self.back_left_pub.publish(speed)
        time.sleep(0.01)
        self.back_right_pub.publish(speed)
        
        rospy.loginfo("Sent Speed to all Motors: " + str(speed))

    def joy_callback(self, msg):
        # msg.axes = [left_stick_x, left_stick_y, left_trigger, right_stick_x, right_stick_y, right_trigger]
        print(msg)
        #if abs(msg.axes[0]) < 0.1:
        #    self.send_to_all(0)
        #    return
        if abs(msg.axes[0]) < 0.5:
            return 
        self.send_to_all(msg.axes[0])
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