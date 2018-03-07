#!/usr/bin/env python

import rospy
from time import sleep
from pololu import Controller
import csv
import math as m

write_data = True

rospy.init_node('ir_test')

if write_data:
    csv_out = open("IR_out.csv", 'a')
    writer = csv.writer(csv_out)

with Controller(2) as ir_bottom, Controller(3) as ir_top:
    rate = rospy.Rate(10)
    i = 0
    while not rospy.is_shutdown() and i < 50:
        data_top = ir_top.get_position()
        data_bottom = ir_bottom.get_position()

        d1 = data_bottom
        d2 = data_top

        phi = 0.68      # [rad]
        # x = 1-(d1/d2 - m.cos(phi))**2
        # print x
        # D = d1*m.sqrt(1-(d1/d2 - m.cos(phi))**2)

        if write_data:
            writer.writerow([350, data_top, data_bottom])

        print i,"\t",d1,"\t",d2
        rate.sleep()
        i += 1

if write_data:
    csv_out.close()
