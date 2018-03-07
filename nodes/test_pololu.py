#!/usr/bin/env python

import rospy
from time import sleep
# from pololu import Controller
import csv


def convert(x):
	result = 0.1674*x**3 - 0.6824*x**2 + 1.8431*x - 0.3559
	return result

rospy.init_node('tester')

csv_in = open("ir_output.csv", 'r')
reader = csv.reader(csv_in)
i = 1
for row in reader:
	print i
	i += 1
	print float(row[0])
	print convert(float(row[0]))
csv_in.close()

# with Controller(0) as steering, Controller(1) as motor, Controller(2) as ir_one:
	# rate = rospy.Rate(10)
	# i = 0
	# while not rospy.is_shutdown():
		# data = ir_one.get_position()
		# writer.writerow([data])
		# print i
		# rate.sleep()
		# i += 1
		# if i == 50:
			# break
# csv_out.close()
