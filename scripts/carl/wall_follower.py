#!/usr/bin/env python

import rospy
import csv
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

        self.imu_threshold = math.radians(15)

        self.write_data = False

        # used for recording data
        if self.write_data:
            print "OPENING CSV"
            csv_out = open("/home/odroid/ros_ws/src/advanced_robotics_team6/data/ir_course_data_blockedwindow_1.csv", 'a')
            # csv_out = open("ir_course_data_doorway1.csv", 'a')
            self.writer = csv.writer(csv_out)

        # Send setpoints to PIDs
        # Wait for recorded sensor data before publishing setpoint
        self.imu_corner_pid.imu_setpoint()
        self.imu_wall_pid.imu_setpoint(imu_corner_pid.setpoint.data)
        self.ir_bottom_pid.ir_setpoint(170)
        self.ir_top_pid.ir_setpoint(140)

        # Set forward speed
        motor_srv(self.motor_speed)
        print "MOTOR SPEED: ", self.motor_speed

        # Initialize stateMachine()
        self.state = "wall_follow"
        self.imu_wall_pid.ignore = True
        self.imu_corner_pid.ignore = True

        self.time_since_turn = rospy.get_time()
        self.start_time = rospy.Time.now()

    def execute(self):
        self.elapsed_time = rospy.Time.now() - self.start_time

        # written based on the ir_course_data_<obstacle>_1 dataset
        if len(self.imu_corner_pid.reported_states) < 2:
            return 0

        # define setpoint error values for state switching logic
        ir_bottom_error = math.fabs(self.ir_bottom_pid.setpoint.data - self.ir_bottom_pid.state.data)     # [cm]
        ir_top_error = math.fabs(self.ir_top_pid.setpoint.data - self.ir_top_pid.state.data)              # [cm]
        imu_wall_error = math.fabs(self.imu_wall_pid.setpoint.data - self.imu_corner_pid.state.data)      # [rad]
        imu_corner_error = math.fabs(self.imu_corner_pid.setpoint.data - self.imu_corner_pid.state.data)  # [rad]

        # finite differencing on state to estimate derivative (divide by timestep?)
        ir_bottom_diff = math.fabs(self.ir_bottom_pid.state.data - self.ir_bottom_pid.reported_states[-2])    # [cm]
        ir_top_diff = math.fabs(self.ir_top_pid.state.data - self.ir_top_pid.reported_states[-2])             # [cm]
        imu_wall_diff = math.fabs(self.imu_wall_pid.state.data - self.imu_corner_pid.reported_states[-2])     # [rad]
        imu_corner_diff = math.fabs(self.imu_corner_pid.state.data - self.imu_corner_pid.reported_states[-2]) # [rad]

        if self.write_data:
            print "WRITING DATA"
            self.writer.writerow([ir_bottom_error, ir_top_error, ir_bottom_diff, ir_top_diff])

        rospy.loginfo("ir_bottom_diff:\t%f", ir_bottom_diff)
        rospy.loginfo("ir_top_diff:\t%f", ir_top_diff)
        rospy.loginfo("ir_bottom_error:\t%f", ir_bottom_error)
        rospy.loginfo("ir_top_error:\t%f", ir_top_error)

        if self.state == 'wall_follow':
            print "WALL-FOLLOW"

            # doorway detected (window/door combos will be mistaken for doorways)
            # may want to increase to aroun 650 based on data
            if ir_top_error > 500 or ir_top_diff > 500:
                print "DOORWAY DETECTED"
                self.state = 'doorway'

                # ignore both IR sensores and switch to IMU PID
                self.ir_bottom_pid.ignore = True
                self.ir_top_pid.ignore = True

                # use imu wall-following PID controller and (maybe?) reset IMU setpoints
                self.imu_wall_pid.ignore = False
                imu_setpoint = imu_wall_pid.state.data
                self.imu_wall_pid.imu_setpoint(imu_setpoint)
                self.imu_corner_pid.imu_setpoint(imu_setpoint)

            # corner detected - don't think this will aver happen
            # want to decrease value of ir_top_error as much as possible
            elif (ir_bottom_error > 1500 or ir_bottom_diff > 1500) and ir_top_error > 100:
                print "CORNER DETECTED"
                self.state = 'corner'

                # ignore both IR sensores and switch to IMU PID
                self.ir_bottom_pid.ignore = True
                self.ir_top_pid.ignore = True
                self.imu_wall_pid.ignore = True      # don't know of any reason this should be False at this point

                # enable imu_corner_pid
                self.imu_corner_pid.ignore = False

                # reset IMU setpoint for cornering task relative to current heading
                # (this is to account for IMU heading drift)
                imu_setpoint = self.imu_wall_pid.state.data - math.radians(90)
                self.imu_wall_pid.imu_setpoint(imu_setpoint)
                self.imu_corner_pid.imu_setpoint(imu_setpoint)

                # decrease motor speed during turn:
                self.motor_srv(self.motor_speed - 50)

            # continue wall-following
            elif False:
                # momentarily ignore IR top or bottom steering commands for semi-large derivative spikes
                if ir_top_diff > 75 and not self.ir_top_pid.ignore:
                    print "DISABLING TOP IR IN DEFAULT CASE"
                    ir_top_pid.ignore = True
                elif ir_top_diff < 75 and self.ir_top_pid.ignore:
                    print "RE-ENABLE TOP IR IN DEFAULT CASE"
                    self.ir_top_pid.ignore = False

                if ir_bottom_diff > 75 and not self.ir_bottom_pid.ignore:
                    print "DISABLING BOTTOM IR IN DEFAULT CASE"
                    self.ir_bottom_pid.ignore = True
                elif ir_bottom_diff < 75 and self.ir_bottom_pid.ignore:
                    print "RE-ENABLE BOTTOM IR IN DEFAULT CASE"
                    self.ir_bottom_pid.ignore = False

                # if both IR derivs are semi-large use IMU WALL PID momentarily
                if self.ir_top_pid.ignore and self.ir_bottom_pid.ignore:
                    print "IR DERIVS UNSTABLE - ENABLE IMU WALL PID"
                    self.imu_wall_pid.ignore = False
                elif not self.imu_wall_pid.ignore:
                    print "DISABLING IMU WALL PID"
                    self.imu_wall_pid.ignore = True

            else:
                pass

        elif self.state == 'doorway':
            print "DOORWAY"

            # exit doorway andd switch back to wall-following
            # if ir_bottom_error < 50 and ir_top_error < 50 and ir_top_diff < 50:
            # want to decrease error difference as much as possible
            if ir_top_diff < 50 and (ir_top_error - ir_bottom_error) < 40:
                print "EXITING DOORWAY: RETURNING TO WALL-FOLLOW"
                self.state = 'wall_follow'

                self.ir_bottom_pid.ignore = False
                self.ir_top_pid.ignore = False
                self.imu_wall_pid.ignore = True

            # doorway mistaken for corner - needs further tuning (could decrease these values)
            elif (ir_bottom_error > 1500 or ir_bottom_diff > 1000) and ir_top_error > 500:
                print "CORNER MISTAKEN FOR DOORWAY: ENTERING CORNER"
                self.state = 'corner'

                self.ir_bottom_pid.ignore = True
                self.ir_top_pid.ignore = True
                self.imu_wall_pid.ignore = True      # don't know of any reason this should be False at this point

                # enable imu_corner_pid
                self.imu_corner_pid.ignore = False

                # reset IMU setpoint for cornering task relative to current heading
                # (this is to account for IMU heading drift)
                imu_setpoint = self.imu_wall_pid.state.data - math.radians(90)
                self.imu_wall_pid.imu_setpoint(imu_setpoint)
                self.imu_corner_pid.imu_setpoint(imu_setpoint)

                # decrease motor speed during turn:
                self.motor_srv(self.motor_speed - 50)

            else:
                # re-enable top IR PID once it has cleared doorway
                if ir_top_error < 60:
                    self.ir_top_pid.ignore = False

        elif self.state == 'corner':
            print "CORNERING"
            if imu_corner_error < self.imu_threshold:    # note: make sure IMU_THRESHOLD is in radians
                print "REACHED IMU SETPOINT WITHIN IMU_THRESHOLD"

                # the following if statement will never be true if the imu_corner_error
                # grows large again after it has reached the IMU_THRESHOLD, even if
                # ir derivates have stabilized. This will likely cause some problems.
                # Need to think of another way to handle this...

                # both IR derivatives have stabilized (states not necessarily within DOOR_THRESHOLD)
                if ir_bottom_diff < 100 and ir_top_diff < 100:  # will want to decrease these as much as possible
                    print "EXITING CORNER: IR STATE DERIVATIVES HAVE STABILIZED"
                    self.state = 'wall_follow'

                    # turn IR PID control back on
                    self.ir_bottom_pid.ignore = False
                    self.ir_top_pid.ignore = False
                    self.imu_wall_pid.ignore = True      # may not want to use imu_pid to do wall-following
                    self.imu_corner_pid.ignore = True

                    # increase motor speed after turn:
                    self.motor_srv(self.motor_speed)

            else:
                # log imu_corner_pid state and setpoint error during turn
                rospy.loginfo("CORNERING:\t{}\t{}\t{}".format(math.degrees(self.imu_corner_pid.setpoint.data), \
                math.degrees(self.imu_corner_pid.state.data),math.degrees(imu_corner_error)))

        else:
            print "FAULT: Entered default case in state machine."

        # Set steering command as average of steering commands that we want to use
        i = 0
        steering_cmd = 0
        if not self.ir_top_pid.ignore:
            i += 1
            steering_cmd += self.ir_top_pid.control_effort
        if not self.ir_bottom_pid.ignore:
            i += 1
            steering_cmd += self.ir_bottom_pid.control_effort
        if not self.imu_wall_pid.ignore:
            i += 1
            steering_cmd += self.imu_wall_pid.control_effort
        if not self.imu_corner_pid.ignore:
            i += 1
            steering_cmd += self.imu_corner_pid.control_effort
            rospy.loginfo("Crnr Steering Cmd:\t{}".format(steering_cmd))
        steering_cmd /= i

        rospy.loginfo("Steering Cmd:\t{}".format(steering_cmd))
        return steering_cmd

    def finish(self):
        if self.write_data:
            print "CLOSING CSV"
            self.csv_out.close()
