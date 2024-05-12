#!/usr/bin/env python3
import rospy
import tf2_ros
import math

from geometry_msgs.msg import Twist
from tf2_geometry_msgs import PoseStamped

class DriveToPoint:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cmd_pub = rospy.Publisher('/minibot/mecanum_drive_controller/cmd_vel', Twist, queue_size=10)
        self.goal_sub = rospy.Subscriber('/goal', PoseStamped, self.goal_callback)
        self.floor_frame = "tag_19"
        self.robot_frame = "tag_9"

        self.goal = None
        
        self.MINIMUM_OUTPUT = 0.55
        self.MAXIMUM_OUTPUT = 0.65
        self.kP = 2.5
        self.tolerance = 0.02

    def goal_callback(self, msg: PoseStamped):
        rospy.loginfo("Received goal to drive to point")
        goal = self.tf_buffer.transform(msg, self.floor_frame)
        # -y is somehow forward which agrees with tag 19 (floor)
        # +x is left
        self.goal = goal
        rospy.loginfo(f"Driving to point: {goal.pose.position.x}, {goal.pose.position.y}")

    def clamp(self, output):
        unsigned_output = output
        if abs(output) > self.MAXIMUM_OUTPUT:
            unsigned_output = self.MAXIMUM_OUTPUT
        elif abs(output) < self.MINIMUM_OUTPUT:
            unsigned_output = self.MINIMUM_OUTPUT
        else:
            unsigned_output = abs(output)
        return unsigned_output * (1 if output > 0 else -1)
    
    def main_loop(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                current_pos = self.tf_buffer.lookup_transform(self.floor_frame, self.robot_frame, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"Could not get current position {e}")
                continue
            
            x = current_pos.transform.translation.x
            y = current_pos.transform.translation.y
            
            if self.goal is None:
                r.sleep()
                continue

            if abs(x - self.goal.pose.position.x) < self.tolerance and abs(y - self.goal.pose.position.y) < self.tolerance:
                self.goal = None
                rospy.loginfo("Reached goal - Done")
                # publish 0 twist
                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                
                # TODO make this an actionlib server
            
            if self.goal is None:
                r.sleep()
                continue

            twist = Twist()
            twist.linear.x = -self.clamp(self.kP * (self.goal.pose.position.x - x))
            twist.linear.x *= abs(x - self.goal.pose.position.x) > self.tolerance
            twist.linear.y = self.clamp(self.kP * (self.goal.pose.position.y - y))
            twist.linear.y *= abs(y - self.goal.pose.position.y) > self.tolerance
            twist.angular.z = 0.0
            rospy.loginfo_throttle(1, f"Driving to point: {self.goal.pose.position.x}, {self.goal.pose.position.y} and current position: {x}, {y}, error = {(self.goal.pose.position.x - x)}, {(self.goal.pose.position.y - y)}")
            self.cmd_pub.publish(twist)
            r.sleep()
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
    
if __name__ == '__main__':
    rospy.init_node('drive_to_point')
    dtp = DriveToPoint()
    dtp.main_loop()