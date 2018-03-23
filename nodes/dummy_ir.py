#!/usr/bin/env python

import rospy
from random import uniform
import math
from std_msgs.msg import Float64


if __name__ == '__main__':
    rospy.init_node('dummy_ir', anonymous=True)
    ir_bottom_pub = rospy.Publisher('pololu/bottom_IR/data', Float64)
    ir_top_pub = rospy.Publisher('pololu/top_IR/data', Float64)
    rate = rospy.Rate(500)
    ir_state = Float64()

    while not rospy.is_shutdown():
        ir_state.data = uniform(100,200)
        ir_bottom_pub.publish(ir_state)

        ir_state.data = ir_state.data / math.cos(math.radians(39))
        ir_top_pub.publish(ir_state)
        print 'test1',ir_state.data
        rate.sleep()
