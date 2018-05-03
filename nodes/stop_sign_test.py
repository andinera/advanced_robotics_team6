#!/usr/bin/env python

import rospy
from advanced_robotics_team6.srv import PololuCmd
from std_msgs.msg import Bool


class StopSignTest:

    def __init__(self):
        # Service for controlling motor speed
        rospy.wait_for_service('motor_cmd')
        self.motor_srv = rospy.ServiceProxy('motor_cmd', PololuCmd)
        # Default motor speeds for moving and stopping
        self.go_speed = 6200
        self.stop_speed = 6000

        # Subscribe to stop sign predictor
        self.image_sub = rospy.Subscriber('stop_sign/prediction',
                                          Bool,
                                          self.execute)

    # Stop sign predictor subscriber callback
    def execute(self, msg):
        # If predict a stop sign
        # then stop
        # else go
        if msg:
            self.motor_srv(self.stop_speed)
        else:
            self.motor_srv(self.go_speed)

# Method fir directly calling script
if __name__ == '__main__':
    rospy.init_node('stop_sign_test_node', anonymous=True)
    StopSignTest()
    rospy.spin()
