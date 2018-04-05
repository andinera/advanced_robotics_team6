#!/usr/bin/env python

import rospy
import csv
import math

from advanced_robotics_team6.drivers import *
from advanced_robotics_team6.srv import PololuCmd

NUM_STATES_STORED = 10
MOTOR_CENTER = 6000
STEERING_CENTER = 5800

class Wall_Follower:

    def __init__(self):
        self.motor_speed = 6250
        self.imu_threshold = math.radians(15)
        self.write_data = False
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

        # used for recording data
        if self.write_data:
            print "OPENING CSV"
            csv_out = open("/home/odroid/ros_ws/src/advanced_robotics_team6/data/ir_course_data_blockedwindow_1.csv", 'a')
            # csv_out = open("ir_course_data_doorway1.csv", 'a')
            self.writer = csv.writer(csv_out)

        # Publish PID setpoints
        self.bottom_ir_pid.ir_setpoint(setpoint=125)
        self.top_ir_pid.ir_setpoint(setpoint=130)
        self.wall_imu_pid.imu_setpoint(states=self.cns.imu_states['orientation']['z'])
        self.corner_imu_pid.imu_setpoint(setpoint=self.wall_imu_pid.setpoint.data)

        # Set forward speed
        self.motor_srv(self.motor_speed)
        print "MOTOR SPEED: ", self.motor_speed

        # Initialize stateMachine()
        self.state = "wall_follow"
        self.wall_imu_pid.ignore = True
        self.corner_imu_pid.ignore = True

        self.time_since_turn = rospy.get_time()
        self.start_time = rospy.Time.now()

    def execute(self):
        while not rospy.is_shutdown():
            self.elapsed_time = rospy.Time.now() - self.start_time

            # written based on the ir_course_data_<obstacle>_1 dataset
            if len(self.cns.imu_states['orientation']['z']) < 2:
                continue

            # define setpoint error values for state switching logic
            bottom_ir_error = math.fabs(self.bottom_ir_pid.setpoint.data - self.bottom_ir_pid.state.data)     # [cm]
            top_ir_error = math.fabs(self.top_ir_pid.setpoint.data - self.top_ir_pid.state.data)              # [cm]
            wall_imu_error = math.fabs(self.wall_imu_pid.setpoint.data - self.corner_imu_pid.state.data)      # [rad]
            corner_imu_error = math.fabs(self.corner_imu_pid.setpoint.data - self.corner_imu_pid.state.data)  # [rad]

            # finite differencing on state to estimate derivative (divide by timestep?)
            bottom_ir_diff = math.fabs(self.bottom_ir_pid.state.data - self.cns.bottom_ir_states[-2])    # [cm]
            top_ir_diff = math.fabs(self.top_ir_pid.state.data - self.cns.top_ir_states[-2])             # [cm]
            wall_imu_diff = math.fabs(self.wall_imu_pid.state.data - self.cns.imu_states['orientation']['z'][-2])     # [rad]
            corner_imu_diff = math.fabs(self.corner_imu_pid.state.data - self.cns.imu_states['orientation']['z'][-2]) # [rad]

            if self.write_data:
                print "WRITING DATA"
                self.writer.writerow([bottom_ir_error, top_ir_error, bottom_ir_diff, top_ir_diff])

            rospy.loginfo("bottom_ir_diff:\t%f", bottom_ir_diff)
            rospy.loginfo("top_ir_diff:\t%f", top_ir_diff)
            rospy.loginfo("bottom_ir_error:\t%f", bottom_ir_error)
            rospy.loginfo("top_ir_error:\t%f", top_ir_error)

            if self.state == 'wall_follow':
                print "WALL-FOLLOW"

                # doorway detected (window/door combos will be mistaken for doorways)
                # may want to increase to aroun 650 based on data
                if top_ir_error > 500 or top_ir_diff > 500:
                    print "DOORWAY DETECTED"
                    self.state = 'doorway'

                    # ignore both IR sensores and switch to IMU PID
                    self.bottom_ir_pid.ignore = True
                    self.top_ir_pid.ignore = True

                    # use imu wall-following PID controller and (maybe?) reset IMU setpoints
                    self.wall_imu_pid.ignore = False
                    imu_setpoint = wall_imu_pid.state.data
                    self.wall_imu_pid.imu_setpoint(setpoint=imu_setpoint)
                    self.corner_imu_pid.imu_setpoint(setpoint=imu_setpoint)

                # corner detected - don't think this will aver happen
                # want to decrease value of top_ir_error as much as possible
                elif (bottom_ir_error > 1500 or bottom_ir_diff > 1500) and top_ir_error > 100:
                    print "CORNER DETECTED"
                    self.state = 'corner'

                    # ignore both IR sensores and switch to IMU PID
                    self.bottom_ir_pid.ignore = True
                    self.top_ir_pid.ignore = True
                    self.wall_imu_pid.ignore = True      # don't know of any reason this should be False at this point

                    # enable corner_imu_pid
                    self.corner_imu_pid.ignore = False

                    # reset IMU setpoint for cornering task relative to current heading
                    # (this is to account for IMU heading drift)
                    imu_setpoint = self.wall_imu_pid.state.data - math.radians(90)
                    self.wall_imu_pid.imu_setpoint(setpoint=imu_setpoint)
                    self.corner_imu_pid.imu_setpoint(setpoint=imu_setpoint)

                    # decrease motor speed during turn:
                    self.motor_srv(self.motor_speed - 50)

                # continue wall-following
                elif False:
                    # momentarily ignore IR top or bottom steering commands for semi-large derivative spikes
                    if top_ir_diff > 75 and not self.top_ir_pid.ignore:
                        print "DISABLING TOP IR IN DEFAULT CASE"
                        top_ir_pid.ignore = True
                    elif top_ir_diff < 75 and self.top_ir_pid.ignore:
                        print "RE-ENABLE TOP IR IN DEFAULT CASE"
                        self.top_ir_pid.ignore = False

                    if bottom_ir_diff > 75 and not self.bottom_ir_pid.ignore:
                        print "DISABLING BOTTOM IR IN DEFAULT CASE"
                        self.bottom_ir_pid.ignore = True
                    elif bottom_ir_diff < 75 and self.bottom_ir_pid.ignore:
                        print "RE-ENABLE BOTTOM IR IN DEFAULT CASE"
                        self.bottom_ir_pid.ignore = False

                    # if both IR derivs are semi-large use IMU WALL PID momentarily
                    if self.top_ir_pid.ignore and self.bottom_ir_pid.ignore:
                        print "IR DERIVS UNSTABLE - ENABLE IMU WALL PID"
                        self.wall_imu_pid.ignore = False
                    elif not self.wall_imu_pid.ignore:
                        print "DISABLING IMU WALL PID"
                        self.wall_imu_pid.ignore = True

                else:
                    pass

            elif self.state == 'doorway':
                print "DOORWAY"

                # exit doorway andd switch back to wall-following
                # if bottom_ir_error < 50 and top_ir_error < 50 and top_ir_diff < 50:
                # want to decrease error difference as much as possible
                if top_ir_diff < 50 and (top_ir_error - bottom_ir_error) < 40:
                    print "EXITING DOORWAY: RETURNING TO WALL-FOLLOW"
                    self.state = 'wall_follow'

                    self.bottom_ir_pid.ignore = False
                    self.top_ir_pid.ignore = False
                    self.wall_imu_pid.ignore = True

                # doorway mistaken for corner - needs further tuning (could decrease these values)
                elif (bottom_ir_error > 1500 or bottom_ir_diff > 1000) and top_ir_error > 500:
                    print "CORNER MISTAKEN FOR DOORWAY: ENTERING CORNER"
                    self.state = 'corner'

                    self.bottom_ir_pid.ignore = True
                    self.top_ir_pid.ignore = True
                    self.wall_imu_pid.ignore = True      # don't know of any reason this should be False at this point

                    # enable corner_imu_pid
                    self.corner_imu_pid.ignore = False

                    # reset IMU setpoint for cornering task relative to current heading
                    # (this is to account for IMU heading drift)
                    imu_setpoint = self.wall_imu_pid.state.data - math.radians(90)
                    self.wall_imu_pid.imu_setpoint(setpoint=imu_setpoint)
                    self.corner_imu_pid.imu_setpoint(setpoint=imu_setpoint)

                    # decrease motor speed during turn:
                    self.motor_srv(self.motor_speed - 50)

                else:
                    # re-enable top IR PID once it has cleared doorway
                    if top_ir_error < 60:
                        self.top_ir_pid.ignore = False

            elif self.state == 'corner':
                print "CORNERING"
                if corner_imu_error < self.imu_threshold:    # note: make sure IMU_THRESHOLD is in radians
                    print "REACHED IMU SETPOINT WITHIN IMU_THRESHOLD"

                    # the following if statement will never be true if the corner_imu_error
                    # grows large again after it has reached the IMU_THRESHOLD, even if
                    # ir derivates have stabilized. This will likely cause some problems.
                    # Need to think of another way to handle this...

                    # both IR derivatives have stabilized (states not necessarily within DOOR_THRESHOLD)
                    if bottom_ir_diff < 100 and top_ir_diff < 100:  # will want to decrease these as much as possible
                        print "EXITING CORNER: IR STATE DERIVATIVES HAVE STABILIZED"
                        self.state = 'wall_follow'

                        # turn IR PID control back on
                        self.bottom_ir_pid.ignore = False
                        self.top_ir_pid.ignore = False
                        self.wall_imu_pid.ignore = True      # may not want to use imu_pid to do wall-following
                        self.corner_imu_pid.ignore = True

                        # increase motor speed after turn:
                        self.motor_srv(self.motor_speed)

                else:
                    # log corner_imu_pid state and setpoint error during turn
                    rospy.loginfo("CORNERING:\t{}\t{}\t{}".format(math.degrees(self.corner_imu_pid.setpoint.data), \
                    math.degrees(self.corner_imu_pid.state.data),math.degrees(corner_imu_error)))

            else:
                print "FAULT: Entered default case in state machine."

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
        if self.write_data:
            print "CLOSING CSV"
            self.csv_out.close()
