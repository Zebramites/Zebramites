#!/usr/bin/env python3
# export ROS_MASTER_URI=http://10.32.4.124:5802
import rospy
import tf2_ros
import math
from tf.transformations import euler_from_quaternion
import actionlib
from geometry_msgs.msg import Twist
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import Joy
from minimotor_msgs.msg import DriveToPointAction, DriveToPointGoal, DriveToPointResult, DriveToPointFeedback
from minimotor_msgs.msg import ShootAction, ShootGoal, ShootResult, ShootFeedback
from minimotor_msgs.msg import IntakeAction, IntakeGoal, IntakeResult, IntakeFeedback

# "/goal" with geometry_msgs/PoseStamped, frame_id = "tag_19"

# In front of left note
'''
buntu@dino-laptop:~$ rostopic pub /goal geometry_msgs/PoseStamped "header:
    seq: 0
    stamp:
        secs: 0
        nsecs: 0
    frame_id: 'tag_19'
pose:
    position:
        x: -0.38
        y: -0.25
        z: 0.0
    orientation:
        x: 0.0
        y: 0.0
        z: 0.0'''

# "home"
'''
    seq: 0
    stamp:
        secs: 0
        nsecs: 0
    frame_id: 'tag_19'
pose:
    position:
        x: 0.0
        y: -0.1
        z: 0.0
    orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 0.0" -1''' 

points_leg1 = [[0.0, -0.4], [0.0, -0.1]]
points_leg2 = [[-0.38, -0.25], [-0.38, -0.4], [0.0, -0.1]]
points_leg3 = [[0.35, -0.25], [0.35, -0.4], [0.0, -0.1]]

ac = actionlib.SimpleActionClient("/minibot/drive_to_point", DriveToPointAction)
ac_intake = actionlib.SimpleActionClient("/minibot/intake_server", IntakeAction)
ac_shoot = actionlib.SimpleActionClient("/minibot/shoot_server", ShootAction)

def joy_cb(msg: Joy):
    if not msg.buttons[-3]:
        return
    
    # Shoot preload
    rospy.loginfo("auto: shooting preload")
    s = ShootGoal()
    s.dont_spin_up = False
    s.dont_shoot = False
    s.dont_spin_down = True

    done = False
    def done_callback(state, result):
        nonlocal done
        rospy.loginfo(f"shot note with state {state} and result {result}")
        done = True
    
    r = rospy.Rate(50)
    ac_shoot.send_goal(goal, done_cb=done_callback)

    while not done and not rospy.is_shutdown():
        r.sleep()

    pose_stamps1 = [PoseStamped(pose=Pose(position=Point(x=point[0], y=point[1], z=0), orientation=Quaternion(x=0, y=0, z=0, w=1))) for point in points_leg1]
    pose_stamps2 = [PoseStamped(pose=Pose(position=Point(x=point[0], y=point[1], z=0), orientation=Quaternion(x=0, y=0, z=0, w=1))) for point in points_leg2]
    pose_stamps3 = [PoseStamped(pose=Pose(position=Point(x=point[0], y=point[1], z=0), orientation=Quaternion(x=0, y=0, z=0, w=1))) for point in points_leg3]
    poses_stamped = [pose_stamps1, pose_stamps2, pose_stamps3]

    for pose_stamps in poses_stamped:
        # Activate intake
        rospy.loginfo("auto: intaking")
        i = IntakeGoal()
        ac_intake.send_goal(i)

        for pose_stamped in pose_stamps:
            goal = DriveToPointGoal()
            rospy.loginfo(f"Driving to point {pose_stamped}")
            goal.pose = pose_stamped
            goal.tolerance = 0.02
            done = False
            def done_callback(state, result):
                nonlocal done
                rospy.loginfo(f"Drive to object actionlib finished with state {state} and result {result}")
                done = True

            r = rospy.Rate(50)
            ac.send_goal(goal, done_cb=done_callback)

            while not done and not rospy.is_shutdown():
                r.sleep()
            
        rospy.loginfo("auto: shooting")
        # Shoot picked up note
        s = ShootGoal()
        s.dont_spin_up = False
        s.dont_shoot = False
        s.dont_spin_down = True

#rospy.Subscriber("/minibot/joy_operator", Joy, operator_callback)
# rospy.Subscriber("/minibot/joy", Joy, joy_cb)
rospy.init_node('auto_node', anonymous=True)

buttons = [1,1,1,1,1,1,1,1,1]
msg = Joy(buttons=buttons)
joy_cb(msg)

# rospy.spin()