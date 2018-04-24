#!/usr/bin/env python

import rospy
import math

from advanced_robotics_team6.drivers import *
from advanced_robotics_team6.srv import PololuCmd

NUM_STATES_STORED = 10
MOTOR_CENTER = 6000
STEERING_CENTER = 5800


class Wall_Follower:

    def __init__(self, event):
        self.motor_speed = 6250

        self.corner_error_threshold = 350

        # Event for synchronizing processes
        self.event = event
        # Driver for sensor input gathering
        self.cns = cns_driver.CNS()
        # PID drivers
        self.bottom_ir_pid = pid_driver.PID("ir/bottom", NUM_STATES_STORED)
        self.top_ir_pid = pid_driver.PID("ir/top", NUM_STATES_STORED)
        self.wall_imu_pid = pid_driver.PID("imu/wall", NUM_STATES_STORED)
        self.corner_imu_pid = pid_driver.PID("imu/corner", NUM_STATES_STORED)
        # Publish PID setpoints
        self.bottom_ir_pid.ir_setpoint(setpoint=170)
        self.top_ir_pid.ir_setpoint(setpoint=140)
        self.wall_imu_pid.imu_setpoint(states=self.cns.imu_states['orientation']['z'])
        self.corner_imu_pid.imu_setpoint(setpoint=self.wall_imu_pid.setpoint.data)
        # Servo output services
        rospy.wait_for_service('motor_cmd')
        rospy.wait_for_service('steering_cmd')
        self.motor_srv = rospy.ServiceProxy('motor_cmd', PololuCmd)
        self.steering_srv = rospy.ServiceProxy('steering_cmd', PololuCmd)
        # Initialize servo and motor to neutral
        self.motor_srv(MOTOR_CENTER)
        self.steering_srv(STEERING_CENTER)

        rospy.sleep(0.5)

        # Set forward speed
        self.motor_srv(6250)
        print "MOTOR SPEED: ", self.motor_speed

        self.state = "wall_follow"
        self.wall_imu_pid.ignore = True
        self.corner_imu_pid.ignore = True

        self.time_since_turn = rospy.get_time()

    def execute(self):
        while not rospy.is_shutdown():
            print self.cns.top_ir_states
            while not rospy.is_shutdown() and len(self.cns.imu_states['orientation']['z']) < 9:
                self.event.wait()
                self.event.clear()
                self.publish_states()

            self.event.wait()
            self.event.clear()

            # define setpoint error values for state switching logic
            bottom_ir_error = math.fabs(self.bottom_ir_pid.setpoint.data - self.bottom_ir_pid.state.data)
            top_ir_error = math.fabs(self.top_ir_pid.setpoint.data - self.top_ir_pid.state.data)
            wall_imu_error = math.fabs(self.wall_imu_pid.setpoint.data - self.corner_imu_pid.state.data)
            corner_imu_error = math.fabs(self.corner_imu_pid.setpoint.data - self.corner_imu_pid.state.data)

            # finite differencing on state to estimate derivative (divide by timestep?)
            bottom_ir_diff = math.fabs(self.bottom_ir_pid.state.data - self.cns.bottom_ir_states[-9])
            top_ir_diff = math.fabs(self.top_ir_pid.state.data - self.cns.top_ir_states[-9])
            wall_imu_diff = math.fabs(self.wall_imu_pid.state.data - self.cns.imu_states['orientation']['z'][-9])
            corner_imu_diff = math.fabs(self.corner_imu_pid.state.data - self.cns.imu_states['orientation']['z'][-9])
            corner_count = 0

            if self.state == 'wall_follow':
                print "WALL-FOLLOW"
                self.motor_srv(6250)
                rospy.loginfo("bottom_ir_diff:\t%f", bottom_ir_diff)
                rospy.loginfo("top_ir_diff:\t%f", top_ir_diff)
                rospy.loginfo("bottom_ir_error:\t%f",bottom_ir_error)
                rospy.loginfo("top_ir_error:\t%f",top_ir_error)
                # either top or bottom IR has detected corner
                if bottom_ir_error > 1000 and bottom_ir_diff > 1000 and corner_count < 3:
                    print "CORNER DETECTED"
                    self.state = 'corner'
                    self.motor_srv(6150)
                    corner_count += 1
                    self.bottom_ir_pid.ignore = True
                    self.top_ir_pid.ignore = True
                    self.wall_imu_pid.ignore = True      # don't know of any reason this should be False at this point

                    # enable corner_imu_pid
                    self.corner_imu_pid.ignore = False

                    # reset IMU setpoint for cornering task
                    imu_setpoint = self.corner_imu_pid.state.data - math.radians(90)
                    self.wall_imu_pid.imu_setpoint(setpoint=imu_setpoint)
                    self.corner_imu_pid.imu_setpoint(setpoint=imu_setpoint)

                # either top or bottom IR has detected doorway
                elif top_ir_error > 500 and top_ir_error < 5000 and \
                            top_ir_diff > 500 and top_ir_diff < 5000:
                    print "DOORWAY DETECTED"
                    self.state = 'wall_follow'

                    # reset IMU setpoint for cornering task
                    imu_setpoint = self.wall_imu_pid.state.data
                    self.wall_imu_pid.imu_setpoint(setpoint=imu_setpoint)

                    self.bottom_ir_pid.ignore = True
                    self.top_ir_pid.ignore = True
                    # use imu wall-following PID controller
                    self.wall_imu_pid.ignore = False

                else:
                    #protect against entering or exiting a corner
                    if bottom_ir_error < self.corner_error_threshold and top_ir_error < self.corner_error_threshold:
                        self.bottom_ir_pid.ignore = False
                        self.top_ir_pid.ignore = False
                    elif bottom_ir_error > self.corner_error_threshold:
                        self.bottom_ir_pid.ignore = True
                    elif top_ir_error > self.corner_error_threshold:
                        self.top_ir_pid.ignore = True

            elif self.state == 'corner':
                print "CORNERING"
                rospy.loginfo("CORNERING:\t{}".format(corner_imu_pid))
                if corner_imu_error < math.pi/4.5:
                    print "REACHED IMU SETPOINT WITHIN IMU_THRESHOLD"

                    # both IR errors are less than corner state

                    if top_ir_error < 100 and bottom_ir_error < 100:
                        # turn top and bottom IR PID control back on
                        self.bottom_ir_pid.ignore = False
                        self.top_ir_pid.ignore = False
                        self.wall_imu_pid.ignore = True
                        self.corner_imu_pid.ignore = True

                        self.state = 'wall_follow'

                    elif top_ir_error < 100 :
                        # turn top IR PID control back on
                        self.bottom_ir_pid.ignore = True
                        self.top_ir_pid.ignore = False
                        self.wall_imu_pid.ignore = True     # may not want to use imu_pid to do wall-following
                        self.corner_imu_pid.ignore = True

                        self.state = 'corner'
                        print "Using top ir sensor for wall follow"
                else:
                    # log corner_imu_pid state and setpoint error during turn
                    rospy.loginfo("CORNERING:\t{}\t{}".format(math.degrees(self.corner_imu_pid.state.data), math.degrees(corner_imu_error)))

            else:
                print "Entered default case in state machine."

            # Set steering command as average of steering commands that we want to use
            i = 0
            steering_cmd = 0
            if not self.top_ir_pid.ignore:
                i += 1
                steering_cmd += self.top_ir_pid.control_effort
                #rospy.loginfo("steering_cmd_top:\t{}".format(top_ir_pid.control_effort))
            if not self.bottom_ir_pid.ignore:
                i += 1
                steering_cmd += self.bottom_ir_pid.control_effort
                #rospy.loginfo("steering_cmd_bottom:\t{}".format(bottom_ir_pid.control_effort))

            if not self.wall_imu_pid.ignore:
                i += 1
                steering_cmd += self.wall_imu_pid.control_effort
                #rospy.loginfo("steering_cmd_wall:\t{}".format(wall_imu_pid.control_effort))

            if not self.corner_imu_pid.ignore:
                i += 1
                steering_cmd += self.corner_imu_pid.control_effort
                #rospy.loginfo("steering_cmd_corner:\t{}".format(corner_imu_pid.control_effort))

            steering_cmd /= i
            rospy.loginfo("steering_cmd:\t{}".format(steering_cmd))

            self.publish_steering_cmd()
            self.publish_states()

    def publish_states(self):
        self.bottom_ir_pid.ir_publish_state(self.cns.bottom_ir_states)
        self.top_ir_pid.ir_publish_state(self.cns.top_ir_states)
        self.wall_imu_pid.imu_publish_state(self.cns.imu_states['orientation']['z'])
        self.corner_imu_pid.imu_publish_state(state=self.wall_imu_pid.state.data)

    def publish_steering_cmd(self):
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
