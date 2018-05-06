#!/usr/bin/env python

import rospy
from advanced_robotics_team6.srv import PololuCmd
from std_msgs.msg import Bool


class StopSignTest:

    def __init__(self):
        # Service for controlling motor speed
        rospy.wait_for_service('motor_cmd')
        rospy.wait_for_service('steering_cmd')
        self.motor_srv = rospy.ServiceProxy('motor_cmd', PololuCmd)
        self.steering_srv = rospy.ServiceProxy('steering_cmd', PololuCmd)
        # Default motor speeds for moving and stopping
        self.stopped = False
        self.motor_srv(6000)
        self.steering_srv(5800)
        # Subscribe to stop sign predictor
        self.image_sub = rospy.Subscriber('stop_sign/prediction',
                                          Bool,
                                          self.execute)

    # Stop sign predictor subscriber callback
    def execute(self, msg):
        # If predict a stop sign
        # then stop
        # else go
        if msg.data:
            if self.stopped:
                print 6000
                self.motor_srv(6000)
            else:
                self.stopped = True
                print 4095
                self.motor_srv(4095)
        else:
            self.stopped = False
            self.started = True
            self.motor_srv(6200)

# Method for directly calling script
if __name__ == '__main__':
    rospy.init_node('stop_sign_test_node', anonymous=True)
    StopSignTest()
    rospy.spin()
