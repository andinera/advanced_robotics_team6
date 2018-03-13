#!/usr/bin/env python


import rospy
from drivers import pololu

if __name__ == '__main__':
    with pololu.Controller(2) as ir_bottom, pololu.Controller(3) as ir_top:
        while not rospy.is_shutdown():
            measurements_bottom = []
            measurements_top = []
            for i in range(10):
                measurements_bottom.append(ir_bottom.get_position())
                measurements_top.append(ir_top.get_position())
                measurement_bottom = sum(measurements_bottom) / len(measurements_bottom)
                measurement_top = sum(measurements_top) / len(measurements_top)
                print "Bottom: {}".format(measurement_bottom)
                print "Top: {}".format(measurement_top)
                rospy.sleep(1)
