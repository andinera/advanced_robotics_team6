#!/usr/bin/env python

import rospy
import csv
import math
import time
from advanced_robotics_team6.drivers import *
from advanced_robotics_team6.srv import PololuCmd
from sklearn import linear_model
import numpy as np
NUM_STATES_STORED = 10
NUM_RECORDED_STATES = 500
MOTOR_CENTER = 6000
STEERING_CENTER = 5800

class Wall_Follower:

    def __init__(self,event):
        self.drift = False
        self.take_data = False
        #stage 0 corner values
        self.top_c_min_0 = 75
        self.top_c_max_0 = 250
        self.bottom_c_min_0 = 700
        #doorway offset values
        self.doorway_offset = 100
        #stage 1 corner values
        self.top_c_min_1 = 75
        self.top_c_max_1 = self.top_c_max_0 - self.doorway_offset
        self.bottom_c_min_1 = 700

        #doorway values
        self.top_d_min = 250
        self.bottom_d_min = 90
        self.bottom_d_max = 700
        #drift values
        self.top_drift = 200
        self.side_acceration_limit = 0.01
        #state speeds
        self.motor_speed = 6400
        self.wall_speed = 6350
        self.door_speed = 6250
        self.corner_speed = 6200
        self.near_corner_speed = 4500
        self.near_corner_stopped_speed = 6200
        self.finishing_speed = 6900
        #drift speeds
        self.drift_wall_speed = 6900
        self.drift_speed = 4005

        self.acceleration_min = 0.001
        self.doorways_seen_threshold = 0
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
        rospy.sleep(1)

        self.write_data = True

        # used for recording data
        if self.write_data:
            print "OPENING CSV"
            file_name = "/home/odroid/ros_ws/src/advanced_robotics_team6/data/ir_course_data_{}.csv".format(time.time())
            csv_out = open(file_name , 'a')
            # csv_out = open("ir_course_data_doorway1.csv", 'a')
            self.writer = csv.writer(csv_out)

        self.bottom_ir_pid.ir_setpoint(setpoint=125)
        self.top_ir_pid.ir_setpoint(setpoint=125)
        self.wall_imu_pid.imu_setpoint(states=self.cns.imu_states['orientation']['z'])
        self.corner_imu_pid.imu_setpoint(setpoint=self.wall_imu_pid.setpoint.data)


        #self.motor_srv(6450)
        #rospy.sleep(0.5)

        # Set forward speed
        self.motor_srv(6300)
        print "MOTOR SPEED: ", self.motor_speed

        self.state = "wall_follow"
        self.stage = 0 #to know if on first, second or third straightaway
        self.top_ir_pid.ignore = True
        self.wall_imu_pid.ignore = True
        self.corner_imu_pid.ignore = True
        #self.time_since_turn = rospy.get_time()

        self.previous_state = self.state
        self.previous_stage = self.stage
        self.do_regression = False
        self.predicted_wall_distance = None
        if self.take_data:
            self.state = "data"
            self.data_timer = time.time()
    def execute(self):
        #set speeds for different states
        while not rospy.is_shutdown():

            #set speeds for different states
            if self.previous_state != self.state:
                print "Changing Speed"
                if self.drift:
                    if self.state == 'drift':
                        self.motor_srv(self.drift_speed)
                    else:
                        self.motor_srv(self.drift_wall_speed)
                else:
                    if self.stage == 2:
                        self.motor_srv(self.finishing_speed)
                    elif self.state == 'wall_follow':
                        self.motor_srv(self.motor_speed)
                    elif self.state == 'corner':
                        self.motor_srv(self.corner_speed)
                    elif self.state == 'near_corner':
                        self.motor_srv(self.near_corner_speed)
                    elif self.state == 'near_corner_stopped':
                        self.motor_srv(self.near_corner_stopped_speed)
                    else:
                        self.motor_srv(self.door_speed)

            self.event.wait()
            self.event.clear()

            while len(self.corner_imu_pid.reported_states) < 4 or len(self.bottom_ir_pid.reported_states) < 4:
                rospy.sleep(.1)
                self.publish_states()
            #do linear regression on last NUM_RECORDED_STATES to determine validity of measurements
            if len(self.cns.top_ir_states) > NUM_RECORDED_STATES-1:
                x_range = [x for x in range(NUM_RECORDED_STATES)]
                x_range = np.array(x_range)
                x_range = x_range.reshape(-1, 1)
                top_ir_states = np.zeros((NUM_RECORDED_STATES,2))
                top_ir_states = np.array(self.cns.top_ir_states[0:NUM_RECORDED_STATES])
                x_range.reshape(-1,1)
                top_ir_states.reshape(-1,1)
                self.regression = linear_model.LinearRegression()
                self.regression.fit(x_range, top_ir_states)
                self.predicted_wall_distance =  self.regression.predict([NUM_RECORDED_STATES])
                #self.regression_score = self.regression.score()
                self.do_regression = True
            #create top and bottom ir variables for simplisity
            self.ir_top = self.top_ir_pid.state.data
            self.ir_bottom = self.bottom_ir_pid.state.data
            # define setpoint error values for state switching logic
            self.ir_bottom_error = math.fabs(self.bottom_ir_pid.setpoint.data - self.bottom_ir_pid.state.data)
            self.imu_wall_error = math.fabs(self.wall_imu_pid.setpoint.data - self.wall_imu_pid.state.data)
            self.imu_corner_error = math.fabs(self.corner_imu_pid.setpoint.data - self.corner_imu_pid.state.data)
            # finite differencing on state to estimate derivative
            self.ir_bottom_diff = math.fabs(self.bottom_ir_pid.state.data - self.bottom_ir_pid.reported_states[-2])
            self.ir_top_diff = math.fabs(self.top_ir_pid.state.data - self.top_ir_pid.reported_states[-2])
            self.ir_top_difference = self.top_ir_pid.state.data - self.top_ir_pid.reported_states[-2]
            self.imu_wall_diff = math.fabs(self.wall_imu_pid.state.data - self.wall_imu_pid.reported_states[-2])
            self.imu_corner_diff = math.fabs(self.corner_imu_pid.state.data - self.corner_imu_pid.reported_states[-2])

            self.ir_bottom_average_error = math.fabs(self.bottom_ir_pid.setpoint.data - (self.bottom_ir_pid.reported_states[-1] + self.bottom_ir_pid.reported_states[-2] + self.bottom_ir_pid.reported_states[-3])/3)
            #accelerameter states for simplicity
            self.x_accel = self.cns.imu_states['linear_acceleration']['x'][-1]
            self.y_accel = self.cns.imu_states['linear_acceleration']['y'][-1]
            self.imu_heading = self.corner_imu_pid.state.data

            #print all useful information
            print self.state
            print self.stage
            rospy.loginfo("ir_bottom:\t%f",self.ir_bottom)
            rospy.loginfo("ir_bottom_diff:\t%f", self.ir_bottom_diff)
            rospy.loginfo("ir_bottom_error:\t%f",self.ir_bottom_error)
            rospy.loginfo("ir_top:\t%f",self.ir_top)
            rospy.loginfo("ir_top_diff:\t%f", self.ir_top_diff)
            rospy.loginfo("corner_imu:\t%f", self.imu_heading)
            rospy.loginfo("corner_imu_error:\t%f", self.imu_corner_error)
            rospy.loginfo("x_acceleration:\t%f", self.x_accel)
            rospy.loginfo("y_acceleration:\t%f",self.y_accel)
            if self.do_regression:
                rospy.loginfo("Regression Coef:\t%f", self.regression.coef_)
                rospy.loginfo("Predicted Wall Distance:\t%f",self.predicted_wall_distance )
                #rospy.loginfo("Regression Score:\t%f",self.regression_score)
            if self.write_data and self.do_regression:
                self.writer.writerow([time.time(),self.state,self.stage,self.ir_bottom,\
                 self.ir_bottom_error, self.ir_bottom_diff, self.ir_top, self.ir_top_diff,\
                  self.imu_heading, self.imu_corner_error, self.x_accel, self.y_accel,\
                   self.regression.coef_, self.predictied_wall_distance])
            elif self.write_data:
                self.writer.writerow([time.time(),self.state,self.stage,self.ir_bottom,\
                 self.ir_bottom_error, self.ir_bottom_diff, self.ir_top, self.ir_top_diff,\
                  self.imu_heading, self.imu_corner_error, self.x_accel, self.y_accel])
            if self.state == 'data':
                if time.time() - self.data_timer > 3:
                    self.motor_srv(MOTOR_CENTER)
                    self.write_data = False
            if self.state == 'wall_follow':
                #check corner near state
                if self.cornernear_logic():
                    print "CORNER NEAR"
                    self.cornernear_config()
                #check for corner detection
                elif self.corner_logic():
                    print "CORNER DETECTED"
                    print "Set Imu Setpoint to + 90"
                    self.corner_config()
                # doorway detected
                elif self.doorway_logic():
                    print "DOORWAY DETECTED"
                    self.doorway_config()
                elif self.startdrift_logic():
                    self.start_drift_config()
                else:
                    pass

            elif self.state == 'doorway':
                #corner detected in doorway state
                if self.corner_logic():
                    self.corner_config()
                    print "exit to corner because bottom corner threshold"
                #exit conditions for doorway state
            elif self.wall_logic():
                    self.wall_config()
                    print "Exited Doorway with standard method"
            elif self.state == 'corner':
                rospy.loginfo("CORNERING:Corner state-\t{}, Corner error-\t{}".format(math.degrees(self.corner_imu_pid.state.data), math.degrees(imu_corner_error)))
                if imu_corner_error < math.radians(20):
                    print "REACHED IMU SETPOINT WITHIN IMU_THRESHOLD"
                    #continues to control off of imu until it meets wall follow conditions
                    #enter wall follow
                    if ir_bottom_error < 150 and ir_bottom_diff < 10:
                       self. wall_config()
            elif self.state == 'corner_near':
                #enter corner from corner near
                if self.corner_logic():
                    self.corner_config()
    		    #enter wall follow from corner near
                elif self.wall_logic():
                    self.wall_config()
                    print "Exited corner near with standard method"
                #car has stoped to enter corner near stopped state
                elif math.fabs(self.x_accel) < self.acceleration_min:
        		    self.state = 'corner_near_stopped'

            elif self.state == 'corner_near_stopped':
                if self.corner_logic():
                    self.corner_config()
    		    #enter wall follow
                elif self.wall_logic():
                    self.wall_config()
                    print "Exited to wall follow"
            elif self.state == 'drift':
                if self.end_drift_logic():
                    self.end_drift_config()
            elif self.state == 'end_drift':
                if self.wall_logic():
                    self.wall_config()
            else:
                print "Entered default case in state machine."

            self.publish_states()
            self.publish_steering_cmd()

    def publish_states(self):
        self.bottom_ir_pid.ir_publish_state(self.cns.bottom_ir_states)
        self.top_ir_pid.ir_publish_state(self.cns.top_ir_states)
        self.wall_imu_pid.imu_publish_state(self.cns.imu_states['orientation']['z'])
        self.corner_imu_pid.imu_publish_state(state=self.wall_imu_pid.state.data)

    def publish_steering_cmd(self):
            # Set steering command as average of steering commands that we want to use
        i = 0
        steering_cmd = 0
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
        print steering_cmd
        self.steering_srv(STEERING_CENTER + steering_cmd)

    def wall_logic(self):
        if self.ir_bottom_error < 100 and self.ir_top > self.top_d_min and self.ir_bottom_diff < 30:
            return True
        else:
            return False

    def corner_logic(self):
        if self.stage == 0 and self.do_regression:
            if self.ir_bottom_error > self.bottom_c_min_0 and self.ir_top < self.top_c_max_0 and \
            self.predicted_wall_distance < self.top_c_max_0 and self.ir_top_diff < 100 and \
            self.ir_bottom_diff > 1000:
                return Trueself.predicted_wall_distance < self.top_c_max_0
            else:
                return False
        elif self.stage == 1 and self.do_regression:
            if self.ir_bottom_error > self.bottom_c_min_1 and self.ir_top < self.top_c_max_1 and \
            self.predicted_wall_distance < self.top_c_max_0 and self.ir_top_diff < 100 and \
            self.ir_bottom_diff > 1000:
                return True
            else:
                return False
        elif self.stage > 1:
            #could be something else
            return False
        else:
            return False

    def doorway_logic(self):
        if self.ir_top > self.top_d_min and (self.ir_bottom_error > self.bottom_d_min and \
            self.ir_bottom_error < self.bottom_d_max and self.ir_bottom_diff > 50):
            return True
        else:
            return False

    def cornernear_logic(self):
        if self.ir_bottom_error < 200 and self.ir_top < 300 and self.ir_top_difference < 0 \
        and self.stage < 2 and self.imu_corner_error < math.pi/4 \
        and self.ir_top_difference > -200 and self.drift == False:
            return True
        else:
            return False
    def startdrift_logic(self):
        if self.drift:
            #add derivative component
            if self.ir_top < self.top_drift:
                return True
        else:
            return False
    def enddrift_logic(self):
        if self.y_accel < self.side_acceration_limit:
            return True
        else:
            return False

    def wall_config(self):
        self.bottom_ir_pid.ignore = False
        self.wall_imu_pid.ignore = True
        self.corner_imu_pid.ignore = True
        self.state = 'wall_follow'
    def corner_config(self):
        self.bottom_ir_pid.ignore = True
        self.wall_imu_pid.ignore = True      # don't know of any reason this should be False at this point
        self.corner_imu_pid.ignore = False
        imu_setpoint = self.imu_wall_pid.setpoint.data - math.radians(90)
        self.wall_imu_pid.imu_setpoint(setpoint=imu_setpoint)
        self.corner_imu_pid.imu_setpoint(setpoint=imu_setpoint)
        self.state = 'corner'
        self.stage += 1
    def doorway_config(self):
        self.bottom_ir_pid.ignore = True
        self.wall_imu_pid.ignore = False
        self.corner_imu_pid.ignore = True
        self.state = 'doorway'
    def cornernear_config(self):
        self.bottom_ir_pid.ignore = True
        self.wall_imu_pid.ignore = False
        self.corner_imu_pid.ignore = True
        self.state = 'corner_near'
    def start_drift_config(self):
        self.bottom_ir_pid.ignore = True
        self.wall_imu_pid.ignore = True      # don't know of any reason this should be False at this point
        self.corner_imu_pid.ignore = False
        imu_setpoint = self.imu_wall_pid.setpoint.data - math.radians(90)
        self.wall_imu_pid.imu_setpoint(setpoint=imu_setpoint)
        self.corner_imu_pid.imu_setpoint(setpoint=imu_setpoint)
        self.state = 'drift'
    def end_drift_config(self):
        self.bottom_ir_pid.ignore = True
        self.wall_imu_pid.ignore = True
        self.corner_imu_pid.ignore = False
        self.state = 'end_drift'
        self.stage += 1
    def finish(self):
        pass
