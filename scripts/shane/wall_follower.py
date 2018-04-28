#!/usr/bin/env python

import rospy
import math
import wall_follow_smach

from advanced_robotics_team6.drivers import *
from advanced_robotics_team6.srv import PololuCmd
from advanced_robotics_team6.scripts.shane import wall_follow_smach


class Wall_Follower:

    def __init__(self, event):
        self.num_states_stored = 10
        self.motor_center = 6000
        self.steering_center = 5800
        self.corner_error_threshold = 6250
        # Event for synchronizing processes
        self.event = event
        # Driver for sensor input gathering
        self.cns = cns_driver.CNS()
        # PID drivers
        self.ir_one_pid = pid_driver.PID("ir/one", self.num_states_stored)
        self.ir_two_pid = pid_driver.PID("ir/two", self.num_states_stored)
        self.wall_imu_pid = pid_driver.PID("imu/wall", self.num_states_stored)
        self.corner_imu_pid = pid_driver.PID("imu/corner", self.num_states_stored)
        # Publish PID setpoints
        self.ir_one_pid.ir_setpoint(setpoint=170)
        self.ir_two_pid.ir_setpoint(setpoint=140)
        self.wall_imu_pid.imu_setpoint(states=self.cns.imu_states['orientation']['z'])
        self.corner_imu_pid.imu_setpoint(setpoint=self.wall_imu_pid.setpoint.data)
        # Servo output services
        rospy.wait_for_service('motor_cmd')
        rospy.wait_for_service('steering_cmd')
        self.motor_srv = rospy.ServiceProxy('motor_cmd', PololuCmd)
        self.steering_srv = rospy.ServiceProxy('steering_cmd', PololuCmd)
        # Initialize servo and motor to neutral
        self.motor_srv(self.motor_center)
        self.steering_srv(self.steering_center)

    def execute(self):
        wall_follow_smach.main(self)

    def publish_states(self):
        self.ir_one_pid.ir_publish_state(self.cns.ir_one_states)
        self.ir_two_pid.ir_publish_state(self.cns.ir_two_states)
        self.wall_imu_pid.imu_publish_state(self.cns.imu_states['orientation']['z'])
        self.corner_imu_pid.imu_publish_state(state=self.wall_imu_pid.state.data)

    def publish_steering_cmd(self):
        # Set steering command as average of steering commands that we want to use
        i = 0
        steering_cmd = 0
        if not self.ir_two_pid.ignore:
            i += 1
            steering_cmd += self.ir_two_pid.control_effort
        if not self.ir_one_pid.ignore:
            i += 1
            steering_cmd += self.ir_one_pid.control_effort
        if not self.wall_imu_pid.ignore:
            i += 1
            steering_cmd += self.wall_imu_pid.control_effort
        if not self.corner_imu_pid.ignore:
            i += 1
            steering_cmd += self.corner_imu_pid.control_effort
        steering_cmd /= i
        self.steering_srv(STEERING_CENTER + steering_cmd)

    def finish(self):
        pass
