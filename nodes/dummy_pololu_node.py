#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

from advanced_robotics_team6.srv import PololuCmd, PololuCmdResponse
from advanced_robotics_team6.drivers import dummy_pololu_driver as pololu


FREQUENCY = 500

def motor_handler(req):
    global motor_cmd
    motor_cmd = req.data
    return PololuCmdResponse()

def steering_handler(req):
    global steering_cmd
    steering_cmd = req.data
    return PololuCmdResponse()

if __name__ == '__main__':
    rospy.init_node('pololu_node', anonymous=True)

    bottom_ir_pub = rospy.Publisher('pololu/ir/bottom/data', Float64, queue_size=1)
    top_ir_pub = rospy.Publisher('pololu/ir/top/data', Float64, queue_size=1)
    rate = rospy.Rate(500)
    ir_state = Float64()

    motor_srv = rospy.Service('motor_cmd', PololuCmd, motor_handler)
    steering_srv = rospy.Service('steering_cmd',PololuCmd, steering_handler)

    with pololu.Controller(0) as steering,     \
            pololu.Controller(1) as motor,     \
            pololu.Controller(2) as bottom_ir, \
            pololu.Controller(3) as top_ir:

        motor_cmd = 0
        steering_cmd = 0
        timer = rospy.get_rostime() + rospy.Duration(1.0/FREQUENCY)

        while not rospy.is_shutdown():
            ir_state.data = bottom_ir.get_position()
            bottom_ir_pub.publish(ir_state)

            ir_state.data = top_ir.get_position()
            top_ir_pub.publish(ir_state)

            if motor_cmd != 0:
                motor.set_target(motor_cmd)
                motor_cmd = 0

            if steering_cmd != 0:
                steering.set_target(steering_cmd)
                steering_cmd = 0

            # Iterate at frequency of RATE
            while not rospy.is_shutdown() and timer > rospy.get_rostime():
                rospy.sleep(0.1/FREQUENCY)
            timer += rospy.Duration(1.0/FREQUENCY)
