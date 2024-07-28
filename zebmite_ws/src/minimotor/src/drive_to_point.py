#!/usr/bin/env python3
# export ROS_MASTER_URI=http://10.32.4.124:5802
import rospy
import tf2_ros
import math
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Twist
from tf2_geometry_msgs import PoseStamped
import actionlib
from minimotor_msgs.msg import DriveToPointAction, DriveToPointGoal, DriveToPointResult, DriveToPointFeedback
import angles

class DriveToPoint:

    _feedback = DriveToPointFeedback()
    _result = DriveToPointResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, DriveToPointAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cmd_pub = rospy.Publisher('/minibot/mecanum_drive_controller/cmd_vel', Twist, queue_size=10)
        self.floor_frame = "tag_19"
        self.robot_frame = "minifrc_robot"
        
        self.MINIMUM_OUTPUT_X = 0.1
        self.MAXIMUM_OUTPUT_X = 1.0
        self.MINIMUM_OUTPUT_Y = 0.1
        self.MAXIMUM_OUTPUT_Y = 1.0
        self.kP_X = 2.5
        self.kP_Y = 1.8
        self.kP_theta = 0.2
        self.default_tolerance = 0.02
        self.angle_tolerance = 0.05

    def goal_callback(self, msg: PoseStamped):
        rospy.loginfo("Received goal to drive to point")
        goal = self.tf_buffer.transform(msg, self.floor_frame)
        # -y is somehow forward which agrees with tag 19 (floor)
        # +x is left
        self.goal = goal
        rospy.loginfo(f"Driving to point: {goal.pose.position.x}, {goal.pose.position.y}")

    def clamp(self, output, minimum_output, maximum_output):
        unsigned_output = output
        if abs(output) > maximum_output:
            unsigned_output = maximum_output
        elif abs(output) < minimum_output:
            unsigned_output = minimum_output
        else:
            unsigned_output = abs(output)
        return unsigned_output * (1 if output > 0 else -1)
    
    def execute_cb(self, goal):
        self.goal = goal.pose
        if goal.tolerance > 0:
            self.tolerance = goal.tolerance
        else:
            self.tolerance = self.default_tolerance
        r = rospy.Rate(50)
        while not rospy.is_shutdown():

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self._result.success = False
                self._as.set_preempted(self._result)
                return

            try:
                current_pos = self.tf_buffer.lookup_transform(self.floor_frame, self.robot_frame, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"Could not get current position {e}")
                continue
            
            x = current_pos.transform.translation.x
            y = current_pos.transform.translation.y
            rotate_angle = math.pi - euler_from_quaternion([current_pos.transform.rotation.x, current_pos.transform.rotation.y, current_pos.transform.rotation.z, current_pos.transform.rotation.w])[2]
            normalized_angle = math.pi - rotate_angle
            normalized_angle = angles.shortest_angular_distance(normalized_angle, 0)

            if abs(x - self.goal.pose.position.x) < self.tolerance and abs(y - self.goal.pose.position.y) < self.tolerance and abs(normalized_angle) < self.angle_tolerance:
                rospy.loginfo("Reached goal - Done")
                # publish 0 twist
                twist = Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)

                self._result.success = True
                self._as.set_succeeded(self._result)
                return

            twist = Twist()
            x_command = self.kP_X * (self.goal.pose.position.x - x)
            # twist.linear.x *= abs(x - self.goal.pose.position.x) > self.tolerance
            y_command = self.kP_Y * (self.goal.pose.position.y - y)
            # twist.linear.y *= abs(y - self.goal.pose.position.y) > self.tolerance
            twist.linear.x = x_command * math.cos(rotate_angle) - y_command * math.sin(rotate_angle)
            twist.linear.y = x_command * math.sin(rotate_angle) + y_command * math.cos(rotate_angle)
            twist.angular.z = self.kP_theta * normalized_angle
            twist.linear.x = -self.clamp(twist.linear.x, self.MINIMUM_OUTPUT_X, self.MAXIMUM_OUTPUT_X) * (abs(x - self.goal.pose.position.x) >= self.tolerance)
            twist.linear.y = self.clamp(twist.linear.y, self.MINIMUM_OUTPUT_Y, self.MAXIMUM_OUTPUT_Y) * (abs(y - self.goal.pose.position.y) >= self.tolerance)
            rospy.loginfo(f"rotate_angle is {normalized_angle}")
            rospy.loginfo_throttle(0.5, f"driving to point: {self.goal.pose.position.x}, {self.goal.pose.position.y}, angle rotated is {rotate_angle}, current position: ({x}, {y}) @ {rotate_angle}, error = {(self.goal.pose.position.x - x)}, {(self.goal.pose.position.y - y)}, cmd_vel = {twist.linear.x}, {twist.linear.y}")
            self._feedback.x_error = self.goal.pose.position.x - x
            self._feedback.y_error = self.goal.pose.position.y - y
            self._as.publish_feedback(self._feedback)
            self.cmd_pub.publish(twist)
            r.sleep()
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
    
if __name__ == '__main__':
    rospy.init_node('drive_to_point')
    server = DriveToPoint("/minibot/drive_to_point")
    rospy.spin()