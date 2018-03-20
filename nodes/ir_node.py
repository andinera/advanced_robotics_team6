#!/usr/bin/env python

import rospy
from drivers import pololu
from std_msgs.msg import Float64

if __name__ == '__main__':
    rospy.init_node('ir_publisher', anonymous=True)
    ir_bottom_pub = rospy.Publisher('pololu/bottom_IR/data', Float64, queue_size=1)
    ir_top_pub = rospy.Publisher('pololu/top_IR/data', Float64, queue_size=1)
    rate = rospy.Rate(500)
    ir_state = Float64()

    with pololu.Controller(2) as ir_bottom, \
            pololu.Controller(3) as ir_top,
        while not rospy.is_shutdown():
            ir_state.data = ir_bottom.get_position()
            ir_bottom_pub.publish(ir_state)

            ir_state.data = ir_top.get_position()
            ir_top_pub.publish(ir_state)

            rospy.sleep()
