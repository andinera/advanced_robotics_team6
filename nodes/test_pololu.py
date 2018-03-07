#!/usr/bin/env python

import rospy
from time import sleep
from pololu import Controller
import csv


rospy.init_node('tester')

# csv_out = open("ir_output.csv", 'a')
# writer = csv.writer(csv_out)
with Controller(0) as steering, Controller(1) as motor, Controller(2) as ir_one:
	# rate = rospy.Rate(10)
	i = 0
	while not rospy.is_shutdown():
		# data = ir_one.get_position()
		# writer.writerow([data])
		print i
		# rate.sleep()
		i += 1
		# if i == 50:
			# break
# csv_out.close()
