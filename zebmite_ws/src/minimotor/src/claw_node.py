#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from minimotor_msgs.msg import ClawState

OPEN = Float64(100.0)
CLOSED_CUBE = Float64(160.0)
CLOSED_CONE = Float64(160.0)
    
class ClawController:

    def __init__(self) -> None:
        self.sub = rospy.Subscriber("/minibot/claw_state", ClawState, self.callback, queue_size=1)
        self.claw_pub = rospy.Publisher("/minibot/claw_controller/command", Float64, queue_size=1)
        self.claw_is_open = True
        self.claw_pub.publish(OPEN)

    def callback(self, msg):
        if msg.just_invert:
            rospy.loginfo("Inverting claw")
            if self.claw_is_open:
                self.claw_pub.publish(CLOSED_CUBE)
            else:
                self.claw_pub.publish(OPEN)
            self.claw_is_open = not self.claw_is_open
            return
        
        if msg.open_claw:
            self.claw_is_open = True
            rospy.loginfo("Opening claw")
            self.claw_pub.publish(OPEN)

        else:
            self.claw_is_open = False
            if msg.wantCube:
                rospy.loginfo("Closing claw on cube")
                self.claw_pub.publish(CLOSED_CUBE)
            else:
                rospy.loginfo("Closing claw on cone")
                self.claw_pub.publish(CLOSED_CONE)


rospy.init_node('claw_manager', anonymous=True)
claw_controller = ClawController()
rospy.spin()



