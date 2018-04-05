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
        self.motor_speed = 6400

        self.top_c_min = 75
        self.top_c_max = 500
        self.bottom_c_min = 700
        self.top_d_min = 500
        self.bottom_d_min = 90
        self.bottom_d_max = 700
        self.wall_speed = 6350
        self.door_speed = 6250
        self.corner_speed = 6200
        self.near_corner_speed = 4500
        self.near_corner_stopped_speed = 6200
        self.acceleration_min = 0.0001
        self.doorways_seen_threshold = 0

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

        self.write_data = False

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

        self.motor_srv(6450)
        rospy.sleep(0.5)

        # Set forward speed
        self.motor_srv(6400)
        print "MOTOR SPEED: ", self.motor_speed

        self.state = "wall_follow"
        self.wall_imu_pid.ignore = True
        self.corner_imu_pid.ignore = True
        self.corner_imu_pid.turns_completed = 0
        self.time_since_turn = rospy.get_time()

        self.previous_state = self.state

    def execute(self):
        while not rospy.is_shutdown():
            #set speeds for different states
            if self.previous_state != self.state:
                if self.state == 'wall_follow':
                    self.motor_srv(self.motor_speed)
                elif self.state == 'corner':
                    self.motor_srv(self.corner_speed)
                elif self.state == 'near_corner':
                    self.motor_srv(self.near_corner_speed)
                elif self.state == 'near_corner_stopped':
                    self.motor_srv(self.near_corner_stopped_speed)
                else:
                    self.motor_srv(self.door_speed)

            if len(self.cns.imu_states['orientation']['z']) < 4:
                continue

            # define setpoint error values for state switching logic
            bottom_ir_error = math.fabs(self.bottom_ir_pid.setpoint.data - self.bottom_ir_pid.state.data)
            top_ir_error = math.fabs(self.top_ir_pid.setpoint.data - self.top_ir_pid.state.data)
            wall_imu_error = math.fabs(self.wall_imu_pid.setpoint.data - self.corner_imu_pid.state.data)
            corner_imu_error = math.fabs(self.corner_imu_pid.setpoint.data - self.corner_imu_pid.state.data)

            # finite differencing on state to estimate derivative (divide by timestep?)

            bottom_ir_diff = math.fabs(self.bottom_ir_pid.state.data - self.cns.bottom_ir_states[-2])
            top_ir_diff = math.fabs(self.top_ir_pid.state.data - self.cns.top_ir_states[-2])
            top_ir_difference = self.top_ir_pid.state.data - self.cns.top_ir_states[-2]
            wall_imu_diff = math.fabs(self.wall_imu_pid.state.data - self.cns.imu_states['orientation']['z'][-2])
            corner_imu_diff = math.fabs(self.corner_imu_pid.state.data - self.cns.imu_states['orientation']['z'][-2])

            bottom_ir_average_error = math.fabs(self.bottom_ir_pid.setpoint.data - (self.cns.bottom_ir_states[-1] + self.cns.bottom_ir_states[-2] + self.cns.bottom_ir_states[-3])/3)

            if self.write_data:
                print "WRITING DATA"
                self.writer.writerow([bottom_ir_error, top_ir_error, bottom_ir_diff, top_ir_diff])

            if self.state == 'wall_follow':
                print "WALL-FOLLOW"
                rospy.loginfo("bottom_ir_diff:\t%f", bottom_ir_diff)
                rospy.loginfo("top_ir_diff:\t%f", top_ir_diff)
                rospy.loginfo("bottom_ir_error:\t%f",bottom_ir_error)
                rospy.loginfo("top_ir_error:\t%f",top_ir_error)
                #corner near state
                if bottom_ir_error > 200 and bottom_ir_error < 1800 and bottom_ir_diff > 500 and self.corner_imu_pid.doorways_seen > self.doorways_seen_threshold and self.corner_imu_pid.turns_completed < 2:
                    self.bottom_ir_pid.ignore = True
                    self.top_ir_pid.ignore = True
                    self.wall_imu_pid.ignore = False
                    self.corner_imu_pid.ignore = True
                    self.state = 'corner_near'

                # either top or bottom IR has detected corner
                elif bottom_ir_error > self.bottom_c_min and top_ir_error > self.top_c_min and top_ir_error < self.top_c_max and corner_imu_pid.turns_completed < 2 and top_ir_diff < 100 and bottom_ir_diff > 1000:
                    print "CORNER DETECTED"
                    bottom_ir_pid.ignore = True
                    top_ir_pid.ignore = True
                    wall_imu_pid.ignore = True      # don't know of any reason this should be False at this point

                    # enable corner_imu_pid
                    corner_imu_pid.ignore = False
                    imu_setpoint = wall_imu_pid.setpoint.data - math.radians(90)
                    print "set imu setpoint to 90"
                    wall_imu_pid.imu_setpoint(setpoint=imu_setpoint)
                    corner_imu_pid.imu_setpoint(setpoint=imu_setpoint)
                    robot["state"] = 'corner'
                # either top or bottom IR has detected doorway
                elif top_ir_error > self.top_d_min and top_ir_diff > 50  or (bottom_ir_error > self.bottom_d_min and bottom_ir_error < self.bottom_d_max and bottom_ir_diff > 50):
                    print "DOORWAY DETECTED"
                    self.corner_imu_pid.doorways_seen += 1
                    # ignore IR sensor that has detected doorway
                    self.bottom_ir_pid.ignore = True
                    self.top_ir_pid.ignore = True

                    # use imu wall-following PID controller
                    self.wall_imu_pid.ignore = False
                    self.state = 'doorway'

                else:
                    #protect against entering or exiting a corner
                    if bottom_ir_error < 5 and top_ir_error < 5:
                        # reset IMU setpoint for cornering task
                        imu_setpoint = 0
                        headings = self.cns.imu_states['orientation']['z'][:]
                        for i in range(-1,-9,-1):
                            imu_setpoint = imu_setpoint + headings[i]/8

                        self.wall_imu_pid.imu_setpoint(setpoint=imu_setpoint)
                        self.corner_imu_pid.imu_setpoint(setpoint=imu_setpoint)

                    if bottom_ir_error < self.bottom_c_min and top_ir_error < self.top_c_min:
                        self.bottom_ir_pid.ignore = False
                        self.top_ir_pid.ignore = False
                    elif bottom_ir_error > self.bottom_c_min:
                        self.bottom_ir_pid.ignore = True
                        print "ignoring bottom IR while wall following"
                    elif top_ir_error > self.top_c_min:
                        self.top_ir_pid.ignore = True
                        print "ignoring top IR while wall following"


            elif self.state == 'doorway':
                print "DOORWAY"
                rospy.loginfo("bottom_ir_diff:\t%f", bottom_ir_diff)
                rospy.loginfo("top_ir_diff:\t%f", top_ir_diff)
                rospy.loginfo("bottom_ir_error:\t%f", bottom_ir_error)
                rospy.loginfo("top_ir_error:\t%f", top_ir_error)

                #if top_ir_error > CORNER_THRESHOLD:
                    #top_ir_pid.ignore = True
                    #robot["state"] = 'wall_follow'
                    #print "exit becasue top corner threshold"
                if bottom_ir_error > self.bottom_c_min and top_ir_error > self.top_c_min and top_ir_error < self.top_c_max and self.corner_imu_pid.turns_completed < 2 and top_ir_diff < 100 and bottom_ir_diff > 1000:
                    self.corner_imu_pid.ignore = False
                    self.wall_imu_pid.ignore = True
                    imu_setpoint = self.wall_imu_pid.recorded_states[-1] - math.radians(90)
                    self.wall_imu_pid.imu_setpoint(setpoint=imu_setpoint)
                    self.corner_imu_pid.imu_setpoint(setpoint=imu_setpoint)
                    rospy.sleep(.001)
                    self.state = 'corner'
                    print "exit to corner because bottom corner threshold"


                elif bottom_ir_error < 100 and top_ir_error < 100 and bottom_ir_diff < 30 and top_ir_diff < 30:
                    self.bottom_ir_pid.ignore = False
                    self.top_ir_pid.ignore = False
                    self.wall_imu_pid.ignore = True

                    self.state = 'wall_follow'
                    print "Exited Doorway with standard method"

            elif self.state == 'corner':
                print "CORNERING"
                rospy.loginfo("bottom_ir_state:\t%f", self.bottom_ir_pid.state.data)
                rospy.loginfo("top_ir_state:\t%f", self.top_ir_pid.state.data)
                rospy.loginfo("bottom_ir_setpoint:\t%f", self.bottom_ir_pid.setpoint.data)
                rospy.loginfo("top_ir_setpoint:\t%f", self.top_ir_pid.setpoint.data)
                rospy.loginfo("CORNERING:\t{}\t{}".format(math.degrees(self.corner_imu_pid.state.data), math.degrees(corner_imu_error)))
                #if corner_imu_error > math.radians(85) and top_ir_difference < 0 and top_ir_pid.state.data < top_ir_pid.setpoint.data :
                #bottom_ir_pid.ignore = False
                #    top_ir_pid.ignore = False
                #    wall_imu_pid.ignore = True
                #    corner_imu_pid.ignore = True
                #    robot["state"] = 'wall_follow'
                #    print "exited turn due to top IR getting closer"
                if corner_imu_error < math.pi/9:
                    print "REACHED IMU SETPOINT WITHIN IMU_THRESHOLD"

                    # both IR errors are less than corner state

                    if top_ir_error < 100 and bottom_ir_error < 100:


                        # turn top and bottom IR PID control back on
                        self.bottom_ir_pid.ignore = False
                        self.top_ir_pid.ignore = False
                        self.wall_imu_pid.ignore = True
                        self.corner_imu_pid.ignore = True

                        self.state = 'wall_follow'
                        self.corner_imu_pid.turns_completed += 1

                    elif top_ir_error < TOP_C_MIN:
                        # turn top IR PID control back on
                        self.bottom_ir_pid.ignore = True
                        self.top_ir_pid.ignore = False
                        self.wall_imu_pid.ignore = True
                        self.corner_imu_pid.ignore = True
                        print "Using top ir sensor for wall follow"

            elif self.state == 'corner_near':
        		#enter corner
                if bottom_ir_error > self.bottom_c_min and top_ir_error > self.to_c_min and top_ir_error < self.top_c_max and self.corner_imu_pid.turns_completed < 2 and top_ir_diff < 100 and bottom_ir_diff > 1000:
                    self.bottom_ir_pid.ignore = True
                    self.top_ir_pid.ignore = True
                    self.wall_imu_pid.ignore = True
                    self.corner_imu_pid.ignore = False
                    self.state = 'corner'
        		#enter wall follow
                elif bottom_ir_error < 100 and top_ir_error < 100 and bottom_ir_diff < 30 and top_ir_diff < 30:
                    self.corner_imu_pid.doorways_seen += 1
                    self.bottom_ir_pid.ignore = False
                    self.top_ir_pid.ignore = False
                    self.wall_imu_pid.ignore = True
                    self.state = 'wall_follow'
                    print "Exited Doorway with standard method"
        		#enter doorway
                elif top_ir_error > TOP_D_MIN and top_ir_diff > 50  or (bottom_ir_error > self.bottom_d_min and bottom_ir_error < self.bottom_d_max and bottom_ir_diff > 50):
            		if self.corner_imu_pid.accel_data_states[-1] < self.acceleration_min:
            			self.state = 'corner_near_stopped'

            elif self.state == 'corner_near_stopped':
                if bottom_ir_error > self.bottom_c_min and top_ir_error > self.top_c_min and top_ir_error < self.top_c_max and self.corner_imu_pid.turns_completed < 2 and top_ir_diff < 100 and bottom_ir_diff > 1000:
                    self.bottom_ir_pid.ignore = True
                    self.top_ir_pid.ignore = True
                    self.wall_imu_pid.ignore = False
                    self.corner_imu_pid.ignore = True
                    self.state = 'corner'
        		#enter doorway
                elif bottom_ir_error < 100 and top_ir_error < 100 and bottom_ir_diff < 30 and top_ir_diff < 30:
                    self.corner_imu_pid.doorways_seen += 1
                    self.bottom_ir_pid.ignore = False
                    self.top_ir_pid.ignore = False
                    self.wall_imu_pid.ignore = True
                    state = 'wall_follow'
                    print "Exited Doorway with standard method"
        		#enter doorway
                elif top_ir_error > TOP_D_MIN and top_ir_diff > 50  or (bottom_ir_error > self.bottom_d_min and bottom_ir_error < self.bottom_d_max and bottom_ir_diff > 50):
                    pass

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
