#!/usr/bin/env python3
import rospy
import actionlib
from minimotor_msgs.msg import ShootAction, ShootGoal, ShootResult, ShootFeedback
from std_msgs.msg import Float64
import time

class ShootServer(object):

    _result = ShootResult()

    def beambreak_cb(self, msg):
        self.has_note = True if msg.data == 0.0 else False # beam broken = 0.0 (note is there)

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, ShootAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        self.has_note = False
        self.sub = rospy.Subscriber("/minibot/hardware_interface/dio", Float64, self.beambreak_cb)
        self.intake_pub = rospy.Publisher("/minibot/intake_controller/command", Float64, queue_size=1) 
        self.top_intake_pub = rospy.Publisher("/minibot/intake_top_controller/command", Float64, queue_size=1)
        self.roof_roller_pub = rospy.Publisher("/minibot/roof_roller_controller/command", Float64, queue_size=1)
        self.shooter_pub = rospy.Publisher("/minibot/shooter_controller/command", Float64, queue_size=1)
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(50)

        self.roof_roller_pub.publish(-1.0)
        time.sleep(0.2)

        if not goal.dont_spin_up:
            rospy.loginfo("shoot_server: spinning up")
            # if not don't spin up, then spin up
            self.shooter_pub.publish(-1.0)
            self.top_intake_pub.publish(0.6)
            start = time.time()
            while (time.time() - start) < 1.0 and not rospy.is_shutdown():
                if self._as.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self.intake_pub.publish(0.0)
                    self.top_intake_pub.publish(0.0)
                    self.roof_roller_pub.publish(0.0)
                    self.shooter_pub.publish(0.0)
                    self._result.success = False
                    self._as.set_preempted(self._result)
                    return
                r.sleep()
        
        if not goal.dont_shoot:
            # not don't shoot, so shoot
            # if not self.has_note:
            #     rospy.loginfo("shoot_server: no note to shoot")
            # else:
                rospy.loginfo("shoot_server: shooting")
                self.intake_pub.publish(1.0)
                self.top_intake_pub.publish(1.0)
                self.roof_roller_pub.publish(1.0)

                while self.has_note and not rospy.is_shutdown():
                    if self._as.is_preempt_requested():
                        rospy.loginfo('%s: Preempted' % self._action_name)
                        self.intake_pub.publish(0.0)
                        self.top_intake_pub.publish(0.0)
                        self.roof_roller_pub.publish(0.0)
                        self.shooter_pub.publish(0.0)
                        self._result.success = False
                        self._as.set_preempted(self._result)
                        return
                    r.sleep()
                
                time.sleep(0.5)

                rospy.loginfo("shoot_server: shot a note")
                self.intake_pub.publish(0.0)
                self.top_intake_pub.publish(0.0)

        if not goal.dont_spin_down:
            rospy.loginfo("shoot_server: spinning down")
            self.roof_roller_pub.publish(0.0)
            self.shooter_pub.publish(0.0)

        self._result.success = True
        self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('shoot_server')
    server = ShootServer(rospy.get_name())
    rospy.spin()