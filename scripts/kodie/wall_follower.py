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

    def __init__(self,event):

        self.motor_speed = 6400

        self.top_c_min = 75
        self.top_c_max = 300
        self.bottom_c_min = 700
        self.bottom_d_min = 90
        self.bottom_d_max = 700
        self.wall_speed = 6350
        self.door_speed = 6250
        self.corner_speed = 6200
        self.near_corner_speed = 4500
        self.near_corner_stopped_speed = 6200
        self.finishing_speed = 6900
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


        self.write_data = False

        # used for recording data
        if self.write_data:
            print "OPENING CSV"
            csv_out = open("/home/odroid/ros_ws/src/advanced_robotics_team6/data/ir_course_data_blockedwindow_1.csv", 'a')
            # csv_out = open("ir_course_data_doorway1.csv", 'a')
            self.writer = csv.writer(csv_out)

        self.bottom_ir_pid.ir_setpoint(setpoint=125)
        self.top_ir_pid.ir_setpoint(setpoint=125)
        self.wall_imu_pid.imu_setpoint(states=self.cns.imu_states['orientation']['z'])
        self.corner_imu_pid.imu_setpoint(setpoint=self.wall_imu_pid.setpoint.data)


        self.motor_srv(6450)
        rospy.sleep(0.5)

        # Set forward speed
        self.motor_srv(6400)
        print "MOTOR SPEED: ", self.motor_speed

        self.state = "wall_follow"
        self.stage = 0 #to know if on first, second or third straightaway
        self.top_ir_pid.ignore = True
        self.wall_imu_pid.ignore = True
        self.corner_imu_pid.ignore = True
        #self.time_since_turn = rospy.get_time()

        self.previous_state = self.state
        self.previous_stage = self.stage


    def execute(self):
        #set speeds for different states
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

           # if  len(self.cns.imu_states['orientation']['z']) > 4:
            #    self.event.wait()
             #   self.event.clear()
              #  self.publish_states()

            self.event.wait()
            self.event.clear()

            if self.previous_stage != self.stage:
                if self.stage == 1:
                    #change values of self.top_c_min = 75 self.top_c_max = 300
                    pass
                if self.stage == 2:
                    self.motor_srv(self.finishing_speed)

            while len(self.corner_imu_pid.reported_states) < 4 or len(self.bottom_ir_pid.reported_states) < 4:
                rospy.sleep(.1)
                print self.bottom_ir_pid.reported_states
                print self.corner_imu_pid.reported_states
                self.publish_states()
            ir_top = self.top_ir_pid.state.data
            # define setpoint error values for state switching logic
            ir_bottom_error = math.fabs(self.bottom_ir_pid.setpoint.data - self.bottom_ir_pid.state.data)
            imu_wall_error = math.fabs(self.wall_imu_pid.setpoint.data - self.wall_imu_pid.state.data)
            imu_corner_error = math.fabs(self.corner_imu_pid.setpoint.data - self.corner_imu_pid.state.data)

        # finite differencing on state to estimate derivative (divide by timestep?)

            ir_bottom_diff = math.fabs(self.bottom_ir_pid.state.data - self.bottom_ir_pid.reported_states[-2])
            ir_top_diff = math.fabs(self.top_ir_pid.state.data - self.top_ir_pid.reported_states[-2])
            ir_top_difference = self.top_ir_pid.state.data - self.top_ir_pid.reported_states[-2]
            imu_wall_diff = math.fabs(self.wall_imu_pid.state.data - self.wall_imu_pid.reported_states[-2])
            imu_corner_diff = math.fabs(self.corner_imu_pid.state.data - self.corner_imu_pid.reported_states[-2])

            ir_bottom_average_error = math.fabs(self.bottom_ir_pid.setpoint.data - (self.bottom_ir_pid.reported_states[-1] + self.bottom_ir_pid.reported_states[-2] + self.bottom_ir_pid.reported_states[-3])/3)
            x_accel = self.cns.imu_states['linear_acceleration']['x'][-1]





            if self.write_data:
                print "WRITING DATA"
                self.writer.writerow([ir_bottom_error, ir_top_error, ir_bottom_diff, ir_top_diff])

            if self.state == 'wall_follow':
                print "WALL-FOLLOW"
                rospy.loginfo("ir_bottom_diff:\t%f", ir_bottom_diff)
                rospy.loginfo("ir_top_diff:\t%f", ir_top_diff)
                rospy.loginfo("ir_bottom_error:\t%f",ir_bottom_error)
                #corner near state
                if ir_bottom_error < 200 and ir_top < 300 and ir_top_difference < 0 \
                and self.stage < 2 and imu_corner_error < math.pi/4 \
                and ir_top_difference > -200:
                    self.bottom_ir_pid.ignore = True
                    self.wall_imu_pid.ignore = False
                    self.corner_imu_pid.ignore = True
                    self.state = 'corner_near'

            # either top or bottom IR has detected corner
                elif ir_bottom_error > self.bottom_c_min and ir_top < self.top_c_max and \
                self.stage < 2 and ir_top_diff < 100 and ir_bottom_diff > 1000:
                    print "CORNER DETECTED"
                    bottom_ir_pid.ignore = True
                    wall_imu_pid.ignore = True      # don't know of any reason this should be False at this point

                # enable imu_corner_pid
                    corner_imu_pid.ignore = False
                    imu_setpoint = wall_imu_pid.setpoint.data - math.radians(90)
                    print "set imu setpoint to 90"
                    wall_imu_pid.imu_setpoint(imu_setpoint)
                    corner_imu_pid.imu_setpoint(imu_setpoint)
                    self.state = 'corner'
                    self.stage += 1
                    # either top or bottom IR has detected doorway
                elif ir_top > self.top_c_max and (ir_bottom_error > self.bottom_d_min and \
                    ir_bottom_error < self.bottom_d_max and ir_bottom_diff > 50):
                    print "DOORWAY DETECTED"
                    self.corner_imu_pid.doorways_seen += 1
                    # ignore IR sensor that has detected doorway
                    self.bottom_ir_pid.ignore = True

                    # use imu wall-following PID controller
                    self.wall_imu_pid.ignore = False
                    self.state = 'doorway'

                else:
                #protect against entering or exiting a corner
                #if ir_bottom_error < 5:
                    # reset IMU setpoint for cornering task
                    #imu_setpoint = 0
                    #headings = self.imu_wall_pid.recorded_states[:]
                    #for i in range(-1,-9,-1):
                        #imu_setpoint = imu_setpoint + headings[i]/8

                    #self.imu_wall_pid.imu_setpoint(imu_setpoint)
                    #self.imu_corner_pid.imu_setpoint(imu_setpoint)

                    if ir_bottom_error < self.bottom_c_min and ir_top > self.top_c_max:
                        self.bottom_ir_pid.ignore = False
                    elif ir_bottom_error > self.bottom_c_min:
                        self.bottom_ir_pid.ignore = True
                        print "ignoring bottom IR while wall following"

            elif self.state == 'doorway':
                print "DOORWAY"
                rospy.loginfo("ir_bottom_diff:\t%f", ir_bottom_diff)
                rospy.loginfo("ir_top_diff:\t%f", ir_top_diff)
                rospy.loginfo("ir_bottom_error:\t%f", ir_bottom_error)

            #if ir_top_error > CORNER_THRESHOLD:
                #ir_top_pid.ignore = True
                #robot["state"] = 'wall_follow'
                #print "exit becasue top corner threshold"
                if  ir_bottom_error > self.bottom_c_min and ir_top < self.top_c_max and \
                self.stage < 2 and ir_top_diff < 100 and ir_bottom_diff > 1000:
                    self.corner_imu_pid.ignore = False
                    self.wall_imu_pid.ignore = True
                    imu_setpoint = self.imu_wall_pid.recorded_states[-1] - math.radians(90)
                    self.wall_imu_pid.imu_setpoint(imu_setpoint)
                    self.corner_imu_pid.imu_setpoint(imu_setpoint)
                    rospy.sleep(.001)
                    self.state = 'corner'
                    print "exit to corner because bottom corner threshold"


                elif ir_bottom_error < 50 and ir_bottom_diff < 30:
                    self.bottom_ir_pid.ignore = False
                    self.wall_imu_pid.ignore = True

                    self.state = 'wall_follow'
                    print "Exited Doorway with standard method"

            elif self.state == 'corner':
                print "CORNERING"
                rospy.loginfo("ir_bottom_state:\t%f", self.bottom_ir_pid.state.data)
                rospy.loginfo("ir_top_state:\t%f", self.top_ir_pid.state.data)
                rospy.loginfo("ir_bottom_setpoint:\t%f", self.bottom_ir_pid.setpoint.data)
                rospy.loginfo("ir_top_setpoint:\t%f", self.top_pid.setpoint.data)
                rospy.loginfo("CORNERING:\t{}\t{}".format(math.degrees(self.corner_imu_pid.state.data), math.degrees(imu_corner_error)))

            #    print "exited turn due to top IR getting closer"
                if imu_corner_error < math.pi/9:
                    print "REACHED IMU SETPOINT WITHIN IMU_THRESHOLD"

                # both IR errors are less than corner state

                    if ir_bottom_error < 150 and ir_bottom_diff < 10:
                    # turn top and bottom IR PID control back on
                        self.bottom_ir_pid.ignore = False
                        self.wall_imu_pid.ignore = True
                        self.corner_imu_pid.ignore = True

                        self.state = 'wall_follow'
                        self.corner_imu_pid.turns_completed += 1

            elif self.state == 'corner_near':
    		#enter corner
                if ir_bottom_error > self.bottom_c_min and ir_top < self.top_c_max and \
                self.stage < 2 and ir_top_diff < 100 and ir_bottom_diff > 1000:
                    self.bottom_ir_pid.ignore = True
                    self.wall_imu_pid.ignore = True
                    self.corner_imu_pid.ignore = False
                    self.state = 'corner'
                    self.stage += 1
    		#enter wall follow
                elif ir_bottom_error < 100 and ir_top > self.top_c_max and ir_bottom_diff < 30:
                    self.corner_imu_pid.doorways_seen += 1
                    self.bottom_ir_pid.ignore = False
                    self.wall_imu_pid.ignore = True
                    self.state = 'wall_follow'
                    print "Exited corner near with standard method"
                elif math.fabs(x_accel) < self.acceleration_min:
        		    self.state = 'corner_near_stopped'

            elif self.state == 'corner_near_stopped':
                if ir_bottom_error > self.bottom_c_min and ir_top < self.top_c_max and \
                self.stage < 2 and ir_top_diff < 100 and ir_bottom_diff > 1000:
                    self.bottom_ir_pid.ignore = True
                    self.wall_imu_pid.ignore = True
                    self.corner_imu_pid.ignore = False
                    self.state = 'corner'
                    self.stage += 1
    		              #enter wall follow
                elif ir_bottom_error < 100 and ir_top > self.top_c_max and ir_bottom_diff < 30:
                    self.corner_imu_pid.doorways_seen += 1
                    self.bottom_ir_pid.ignore = False
                    self.wall_imu_pid.ignore = True
                    self.state = 'wall_follow'
                    print "Exited corner near with standard method"

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
