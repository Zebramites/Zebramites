#!/usr/bin/env python3
import rospy
import actionlib
from minimotor_msgs.msg import IntakeAction, IntakeGoal, IntakeResult, IntakeFeedback
from std_msgs.msg import Float64

class IntakeServer(object):

    _result = IntakeResult()

    def beambreak_cb(self, msg):
        self.has_note = True if msg.data == 0.0 else False # beam broken = 0.0 (note is there)

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, IntakeAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        self.has_note = False
        self.sub = rospy.Subscriber("/minibot/hardware_interface/dio", Float64, self.beambreak_cb)
        self.intake_pub = rospy.Publisher("/minibot/intake_controller/command", Float64, queue_size=1) 
        self.top_intake_pub = rospy.Publisher("/minibot/intake_top_controller/command", Float64, queue_size=1)
        self.roof_roller_pub = rospy.Publisher("/minibot/roof_roller_controller/command", Float64, queue_size=1)
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(50)
        
        if self.has_note:
            self._result.success = True
            self._as.set_succeeded(self._result)
            return
        
        rospy.loginfo("intake_server: intaking")
        self.intake_pub.publish(1.0)
        self.top_intake_pub.publish(1.0)
        self.roof_roller_pub.publish(1.0)

        while not self.has_note and not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self.intake_pub.publish(0.0)
                self.top_intake_pub.publish(0.0)
                self.roof_roller_pub.publish(0.0)
                self._result.success = False
                self._as.set_preempted(self._result)
                return
            r.sleep()
        
        rospy.loginfo("intake_server: stopping intake")
        self.intake_pub.publish(0.0)
        self.top_intake_pub.publish(0.0)
        self.roof_roller_pub.publish(0.0)

        self._result.success = True
        self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('intake_server')
    server = IntakeServer(rospy.get_name())
    rospy.spin()