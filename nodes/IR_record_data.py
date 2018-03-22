#!/usr/bin/env python

import rospy
from time import sleep
from drivers import pololu
import csv
import math
import numpy as np

from std_msgs.msg import Float64
from std_srvs.srv import Empty
from advanced_robotics_team6.srv import PololuCmd

write_data = True
CENTER = 6000
MOTOR_SPEED = 6300

rospy.init_node('ir_test')

if write_data:
    csv_out = open("ir_course_data_doorwindow.csv", 'a')
    # csv_out = open("ir_course_data_doorway1.csv", 'a')
    writer = csv.writer(csv_out)

with pololu.Controller(0) as steering,  \
     pololu.Controller(1) as motor,     \
     pololu.Controller(2) as ir_bottom, \
     pololu.Controller(3) as ir_top:

    rate = rospy.Rate(50)

    distance_top = []
    distance_bottom = []
    for i in range(1,50):
        distance_top.append(ir_top.get_position())
        distance_bottom.append(ir_bottom.get_position())
        sleep(.01)

    # average of initial IR sensor data
    setpoint_top = int(sum(distance_top) / float(len(distance_top)))
    setpoint_bottom = int(sum(distance_bottom) / float(len(distance_bottom)))

    print "Top Setpoint: {} cm".format(setpoint_top)
    print "Bottom Setpoint: {} cm".format(setpoint_bottom)

    # Set zero intial velocity and steering
    #steering.set_target(CENTER)
    steering.set_target(CENTER-200)
    motor.set_target(CENTER)
    rospy.sleep(1)

    # Set forward speed
    motor.set_target(MOTOR_SPEED)
    print "MOTOR SPEED: ", MOTOR_SPEED

    start_time = rospy.Time.now()
    elapsed_time = rospy.Time.now() - start_time

    count = 1
    while not rospy.is_shutdown() and elapsed_time < rospy.Duration(3.5):
        elapsed_time = rospy.Time.now() - start_time
        steering.set_target(CENTER-200)

        if count == 1:
            data_top_prev = setpoint_top
            data_bottom_prev = setpoint_bottom
        else:
            data_top_prev = data_top
            data_bottom_prev = data_bottom

        data_top = ir_top.get_position()
        data_bottom = ir_bottom.get_position()

        ir_bottom_error = math.fabs(setpoint_bottom - data_bottom)
        ir_top_error = math.fabs(setpoint_top - data_top)
        ir_bottom_diff = math.fabs(data_bottom - data_bottom_prev)
        ir_top_diff = math.fabs(data_top - data_top_prev)

        #data[:,i] = [ir_bottom_error, ir_top_error, ir_bottom_diff, ir_top_diff]

        if write_data:
            # first number in list is distance [cm] to target
            writer.writerow([count, ir_bottom_error, ir_top_error, ir_bottom_diff, ir_top_diff])

        # print i,"\t",d,"\t",d1,"\t",d2,"\t",d3,"\t",d4
        #print "{}\t{}\t{}".format(i, data_bottom, data_top)
        # print i,"\t",data_bottom,"\t",data_top
        rate.sleep()
        count += 1

    # turn motor off
    motor.set_target(CENTER)
    print "DISABLING MOTORS after {} seconds".format(elapsed_time.secs)

if write_data:
    # np.savetxt("ir_course_data_doorway1.csv", data, delimiter=",")
    csv_out.close()
