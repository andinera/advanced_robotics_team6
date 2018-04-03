#!/usr/bin/env python

import rospy
import math

import wall_follower_smach
import multiprocessing


class Wall_Follower:

    def __init__(self, ir_bottom_pid, ir_top_pid, imu_wall_pid, imu_corner_pid,
                 motor_srv, steering_srv):

        self.ir_bottom_pid = ir_bottom_pid
        self.ir_top_pid = ir_top_pid
        self.imu_wall_pid = imu_wall_pid
        self.imu_corner_pid = imu_corner_pid
        self.motor_srv = motor_srv
        self.steering_srv = steering_srv
        self.sync = 0

        self.motor_speed = 6250

        self.corner_error_threshold = 350

        self.imu_corner_pid.imu_setpoint()
        self.imu_wall_pid.imu_setpoint(self.imu_corner_pid.setpoint.data)
        self.ir_bottom_pid.ir_setpoint(170)
        self.ir_top_pid.ir_setpoint(140)

        # Set forward speed
        self.motor_srv(self.motor_speed)

        self.p = multiprocessing.Thread(target = wall_follower_smach.main,
                                  args = (self,))
        self.p.start()

    def execute(self):
        pass

    def finish(self):
        pass
