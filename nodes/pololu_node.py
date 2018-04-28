#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

from advanced_robotics_team6.srv import PololuCmd, PololuCmdResponse
rospy.init_node('pololu_node', anonymous=True)
if rospy.get_param('~offline'):
    from advanced_robotics_team6.drivers import dummy_pololu_driver as pololu
else:
    from advanced_robotics_team6.drivers import pololu_driver as pololu

class Pololu:

    def __init__(self):
        # Frequency pololu sensor data is published
        self.frequency = 500
        # Publisher for IR sensor data
        self.ir_one_pub = rospy.Publisher('pololu/ir/one/data',
                                        Float64,
                                        queue_size=1)
        self.ir_two_pub = rospy.Publisher('pololu/ir/two/data',
                                     Float64,
                                     queue_size=1)
        # Services for sending pololu commands
        self.ir_state = Float64()
        self.motor_srv = rospy.Service('motor_cmd',
                                       PololuCmd,
                                       self.motor_handler)
        self.steering_srv = rospy.Service('steering_cmd',
                                          PololuCmd,
                                          self.steering_handler)
        # Controllers for individual pololu peripheral devices
        self.steering = pololu.Controller(0)
        self.motor = pololu.Controller(1)
        self.ir_one = pololu.Controller(2)
        self.ir_two = pololu.Controller(3)
        # Initialize variables
        self.motor_cmd = 0
        self.steering_cmd = 0
        self.timer = rospy.get_rostime()

    # Handler for motor service call
    def motor_handler(self, req):
        self.motor_cmd = req.data
        return PololuCmdResponse()

    # Handler for steering service call
    def steering_handler(self, req):
        self.steering_cmd = req.data
        return PololuCmdResponse()

    # Method for iteratively sending and receiving data at set frequency
    def iterate(self):
        # Infinite loop
        while not rospy.is_shutdown():
            # Iterate at set frequency
            while not rospy.is_shutdown() and self.timer > rospy.get_rostime():
                rospy.sleep(0.1/self.frequency)
            self.timer += rospy.Duration(1.0/self.frequency)
            # Get and publish ir one data
            self.ir_state.data = self.ir_one.get_position()
            self.ir_one_pub.publish(self.ir_state)
            # Get and publish ir two data
            self.ir_state.data = self.ir_two.get_position()
            self.ir_two_pub.publish(self.ir_state)
            # If motor command is requested, set motor setpoint
            if self.motor_cmd != 0:
                self.motor.set_target(self.motor_cmd)
                self.motor_cmd = 0
            # If steering command is requested, set steering setpoint
            if self.steering_cmd != 0:
                self.steering.set_target(self.steering_cmd)
                self.steering_cmd = 0

# Method to make script directly callable
if __name__ == '__main__':
    pi = Pololu()
    pi.iterate()
