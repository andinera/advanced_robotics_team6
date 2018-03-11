#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Bool
# from pololu import Controller


def control_effort_callback(data):
    print data.data

if __name__ == '__main__':
    rospy.init_node('shane_pid_test')

    # with Controller(0) as steering, Controller(1) as motor, \
    #      Controller(2) as ir_bottom, Controller(3) as ir_top:

    rate = rospy.Rate(1)

    pid_enable = rospy.Publisher('pid_enable', Bool, latch=True, queue_size=1)
    pid_enable_data = Bool()
    pid_enable_data.data = True
    pid_enable.publish(pid_enable_data)

    setpoint = rospy.Publisher('setpoint', Float64, latch=True, queue_size=1)
    setpoint_data = Float64()
    setpoint_data.data = 1.0
    setpoint.publish(setpoint_data)

    state = rospy.Publisher('state', Float64, latch=True, queue_size=1)
    state_data = Float64()

    control_effort = rospy.Subscriber('control_effort', Float64, control_effort_callback)

    while not rospy.is_shutdown():
        state_data.data = 2
        state.publish(state_data)
        rate.sleep()
