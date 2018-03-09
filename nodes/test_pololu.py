#!/usr/bin/env python

import rospy
from time import sleep
from pololu import Controller
import csv

rospy.init_node('tester')

with Controller(0) as steering, Controller(1) as motor, Controller(2) as ir_one, \
	 Controller(3) as ir_two:
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		print ir_one.get_position()
		print ir_two.get_position()
		print
		rate.sleep()
