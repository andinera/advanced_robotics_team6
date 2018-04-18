#!/usr/bin/env python

import rospy
from time import sleep
import csv
import math as m
from advanced_robotics_team6.drivers import *
from advanced_robotics_team6.srv import PololuCmd
from advanced_robotics_team6.drivers import pololu_driver as pololu

write_data = True

rospy.init_node('ir_test')

if write_data:
    csv_out = open("/home/odroid/ros_ws/src/advanced_robotics_team6/data/ir_test_041718.csv", 'a')
    writer = csv.writer(csv_out)

with pololu.Controller(2) as ir_bottom, pololu.Controller(3) as ir_top:

    rate = rospy.Rate(10)

    i = 0
    while not rospy.is_shutdown() and i < 50:
        data_top = ir_top.get_position()
        data_bottom = ir_bottom.get_position()
        pos = data_bottom

        # Note: Comment out polynomial fit in pololu.py for these to work properly
        # first data fit attempt
        d = 1000*pos - 1
        d = (0.1674*d**3 - 0.6824*d**2 + 1.8431*d - 0.3559)*100

        # second data fit attempt
        d1 = 113230*pos - 129.31
        d2 = 10308000*pos**2 + 54685*pos - 53.212
        d3 = 3212000000*pos**3 - 17229000*pos**2 + 129560*pos - 117.11
        d4 = -8648600000000*pos**4 + 102430000000*pos**3 - 430090000*pos**2 + 864970*pos - 588.4

        phi = 0.68      # [rad]
        # x = 1-(d1/d2 - m.cos(phi))**2
        # print x
        # D = d1*m.sqrt(1-(d1/d2 - m.cos(phi))**2)

        if write_data:
            # first number in list is distance [cm] to target
            writer.writerow([800, data_top, data_bottom])

        # print i,"\t",d,"\t",d1,"\t",d2,"\t",d3,"\t",d4
        print "{}\t{}\t{}".format(i, data_bottom, data_top)
        # print i,"\t",data_bottom,"\t",data_top
        rate.sleep()
        i += 1

if write_data:
    csv_out.close()
