#!/usr/bin/env python

import rospy
import math
import wall_follow_smach
from threading import Thread

from advanced_robotics_team6.drivers import *
from advanced_robotics_team6.srv import PololuCmd
from advanced_robotics_team6.scripts.shane import wall_follow_smach


NUM_STATES_STORED = 10
MOTOR_CENTER = 6000
STEERING_CENTER = 5800

class Wall_Follower:

    def __init__(self, event):
        self.corner_error_threshold = 6250
        # Event for synchronizing processes
        self.event = event
        # Driver for sensor input gathering
        self.cns = cns_driver.CNS()
        # PID drivers
        self.bottom_ir_pid = pid_driver.PID("ir/bottom", NUM_STATES_STORED)
        self.top_ir_pid = pid_driver.PID("ir/top", NUM_STATES_STORED)
        self.wall_imu_pid = pid_driver.PID("imu/wall", NUM_STATES_STORED)
        self.corner_imu_pid = pid_driver.PID("imu/corner", NUM_STATES_STORED)

        self.bottom_ir_pid.ir_setpoint(setpoint=170)
        self.top_ir_pid.ir_setpoint(setpoint=140)
        self.wall_imu_pid.imu_setpoint(self.cns.imu_states['orientation']['z'])
        self.corner_imu_pid.imu_setpoint(setpoint=self.wall_imu_pid.setpoint.data)

        # Servo output services
        rospy.wait_for_service('motor_cmd')
        rospy.wait_for_service('steering_cmd')
        self.motor_srv = rospy.ServiceProxy('motor_cmd', PololuCmd)
        self.steering_srv = rospy.ServiceProxy('steering_cmd', PololuCmd)
        # Initialize state machine process
        self.t = Thread(target=wall_follow_smach.main, args=(self,))
        # Initialize servo and motor to neutral
        self.motor_srv(MOTOR_CENTER)
        self.steering_srv(STEERING_CENTER)

    def execute(self):
        self.t.start()

    def publish_states(self):
        self.bottom_ir_pid.ir_publish_state(self.cns.bottom_ir_states)
        self.top_ir_pid.ir_publish_state(self.cns.top_ir_states)
        self.wall_imu_pid.imu_publish_state(self.cns.imu_states['orientation']['z'])
        self.corner_imu_pid.imu_publish_state(state=self.wall_imu_pid.state.data)

    def pub_steering_cmd(self):
        # Set steering command as average of steering commands that we want to use
        i = 0
        steering_cmd = 0
        if not self.top_ir_pid.ignore:
            i += 1
            steering_cmd += self.top_ir_pid.control_effort
        if not self.bottom_ir_pid.ignore:
            i += 1
            steering_cmd += self.bottom_ir_pid.control_effort
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
