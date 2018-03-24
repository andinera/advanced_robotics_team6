#!/usr/bin/env python

import rospy
import math


class Wall_Follower:

    def __init__(self, ir_bottom_pid, ir_top_pid, imu_wall_pid, imu_corner_pid,
                 motor_srv):

        self.ir_bottom_pid = ir_bottom_pid
        self.ir_top_pid = ir_top_pid
        self.imu_wall_pid = imu_wall_pid
        self.imu_corner_pid = imu_corner_pid
        self.motor_srv = motor_srv
        self.sync = 0

        self.motor_speed = 6250

        self.corner_error_threshold = 350

        self.imu_corner_pid.imu_setpoint()
        self.imu_wall_pid.imu_setpoint(self.imu_corner_pid.setpoint.data)
        self.ir_bottom_pid.ir_setpoint(170)
        self.ir_top_pid.ir_setpoint(140)

        rospy.sleep(0.5)

        # Set forward speed
        self.motor_srv(6250)
        print "MOTOR SPEED: ", self.motor_speed

        self.state = "wall_follow"
        self.imu_wall_pid.ignore = True
        self.imu_corner_pid.ignore = True

        self.time_since_turn = rospy.get_time()

    def execute(self):

        if len(self.imu_corner_pid.reported_states) < 9:
            return 0

        # define setpoint error values for state switching logic
        ir_bottom_error = math.fabs(self.ir_bottom_pid.setpoint.data - self.ir_bottom_pid.state.data)
        ir_top_error = math.fabs(self.ir_top_pid.setpoint.data - self.ir_top_pid.state.data)
        imu_wall_error = math.fabs(self.imu_wall_pid.setpoint.data - self.imu_corner_pid.state.data)
        imu_corner_error = math.fabs(self.imu_corner_pid.setpoint.data - self.imu_corner_pid.state.data)

        # finite differencing on state to estimate derivative (divide by timestep?)
        ir_bottom_diff = math.fabs(self.ir_bottom_pid.state.data - self.ir_bottom_pid.reported_states[-9])
        ir_top_diff = math.fabs(self.ir_top_pid.state.data - self.ir_top_pid.reported_states[-9])
        imu_wall_diff = math.fabs(self.imu_wall_pid.state.data - self.imu_corner_pid.reported_states[-9])
        imu_corner_diff = math.fabs(self.imu_corner_pid.state.data - self.imu_corner_pid.reported_states[-9])
        corner_count = 0

        if self.state == 'wall_follow':
            print "WALL-FOLLOW"
            self.motor_srv(6250)
            rospy.loginfo("ir_bottom_diff:\t%f", ir_bottom_diff)
            rospy.loginfo("ir_top_diff:\t%f", ir_top_diff)
            rospy.loginfo("ir_bottom_error:\t%f",ir_bottom_error)
            rospy.loginfo("ir_top_error:\t%f",ir_top_error)
            # either top or bottom IR has detected corner
            if ir_bottom_error > 1000 and ir_bottom_diff > 1000 and corner_count < 3:
                print "CORNER DETECTED"
                self.state = 'corner'
                self.motor_srv(6150)
                corner_count += 1
                self.ir_bottom_pid.ignore = True
                self.ir_top_pid.ignore = True
                self.imu_wall_pid.ignore = True      # don't know of any reason this should be False at this point

                # enable imu_corner_pid
                self.imu_corner_pid.ignore = False

                # reset IMU setpoint for cornering task
                imu_setpoint = self.imu_corner_pid.state.data - math.radians(90)
                self.imu_wall_pid.imu_setpoint(imu_setpoint)
                self.imu_corner_pid.imu_setpoint(imu_setpoint)

            # either top or bottom IR has detected doorway
            elif ir_top_error > 500 and ir_top_error < 5000 and \
                        ir_top_diff > 500 and ir_top_diff < 5000:
                print "DOORWAY DETECTED"
                self.state = 'wall_follow'

                # reset IMU setpoint for cornering task
                imu_setpoint = self.imu_wall_pid.state.data
                self.imu_wall_pid.imu_setpoint(imu_setpoint)

                self.ir_bottom_pid.ignore = True
                self.ir_top_pid.ignore = True
                # use imu wall-following PID controller
                self.imu_wall_pid.ignore = False

            else:
                #protect against entering or exiting a corner
                if ir_bottom_error < self.corner_error_threshold and ir_top_error < self.corner_error_threshold:
                    self.ir_bottom_pid.ignore = False
                    self.ir_top_pid.ignore = False
                elif ir_bottom_error > self.corner_error_threshold:
                    self.ir_bottom_pid.ignore = True
                elif ir_top_error > self.corner_error_threshold:
                    self.ir_top_pid.ignore = True

        elif self.state == 'corner':
            print "CORNERING"
            rospy.loginfo("CORNERING:\t{}".format(imu_corner_pid))
            if imu_corner_error < math.pi/4.5:
                print "REACHED IMU SETPOINT WITHIN IMU_THRESHOLD"

                # both IR errors are less than corner state

                if ir_top_error < 100 and ir_bottom_error < 100:
                    # turn top and bottom IR PID control back on
                    self.ir_bottom_pid.ignore = False
                    self.ir_top_pid.ignore = False
                    self.imu_wall_pid.ignore = True
                    self.imu_corner_pid.ignore = True

                    self.state = 'wall_follow'

                elif ir_top_error < 100 :
                    # turn top IR PID control back on
                    self.ir_bottom_pid.ignore = True
                    self.ir_top_pid.ignore = False
                    self.imu_wall_pid.ignore = True     # may not want to use imu_pid to do wall-following
                    self.imu_corner_pid.ignore = True

                    self.state = 'corner'
                    print "Using top ir sensor for wall follow"
            else:
                # log imu_corner_pid state and setpoint error during turn
                rospy.loginfo("CORNERING:\t{}\t{}".format(math.degrees(self.imu_corner_pid.state.data), math.degrees(imu_corner_error)))

        else:
            print "Entered default case in state machine."

        # Set steering command as average of steering commands that we want to use
        i = 0
        steering_cmd = 0
        if not self.ir_top_pid.ignore:
            i += 1
            steering_cmd += self.ir_top_pid.control_effort
            #rospy.loginfo("steering_cmd_top:\t{}".format(ir_top_pid.control_effort))
        if not self.ir_bottom_pid.ignore:
            i += 1
            steering_cmd += self.ir_bottom_pid.control_effort
            #rospy.loginfo("steering_cmd_bottom:\t{}".format(ir_bottom_pid.control_effort))

        if not self.imu_wall_pid.ignore:
            i += 1
            steering_cmd += self.imu_wall_pid.control_effort
            #rospy.loginfo("steering_cmd_wall:\t{}".format(imu_wall_pid.control_effort))

        if not self.imu_corner_pid.ignore:
            i += 1
            steering_cmd += self.imu_corner_pid.control_effort
            #rospy.loginfo("steering_cmd_corner:\t{}".format(imu_corner_pid.control_effort))

        steering_cmd /= i
        rospy.loginfo("steering_cmd:\t{}".format(steering_cmd))

        return steering_cmd

    def finish(self):
        pass
