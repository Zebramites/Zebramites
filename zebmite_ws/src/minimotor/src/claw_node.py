#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from minimotor_msgs.msg import ClawState

OPEN = Float64(140.0)
CLOSED_CUBE = Float64(160.0)
CLOSED_CONE = Float64(180.0)
    
class ClawController:

    def __init__(self) -> None:
        self.sub = rospy.Subscriber("/minibot/claw_state", ClawState, self.callback, queue_size=1)
        self.claw_pub = rospy.Publisher("/minibot/claw_controller/command", Float64, queue_size=1)

    def callback(self, msg):
        if msg.open_claw:
            rospy.loginfo("Opening claw")
            self.claw_pub.publish(OPEN)

        else:
            if msg.wantCube:
                rospy.loginfo("Closing claw on cube")
                self.claw_pub.publish(CLOSED_CUBE)
            else:
                rospy.loginfo("Closing claw on cone")
                self.claw_pub.publish(CLOSED_CONE)


rospy.init_node('claw_manager', anonymous=True)
claw_controller = ClawController()
rospy.spin()



