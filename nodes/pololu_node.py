#!/usr/bin/env python

import rospy
from drivers import pololu
# from drivers import dummy_pololu as pololu
from std_msgs.msg import Float64
from advanced_robotics_team6.srv import PololuCmd, PololuCmdResponse

motor_cmd = 0
steering_cmd = 0

def motor_handler(req):
    global motor_cmd
    motor_cmd = req.data
    return PololuCmdResponse()

def steering_handler(req):
    global steering_cmd
    steering_cmd = req.data
    return PololuCmdResponse()

if __name__ == '__main__':
    rospy.init_node('ir_publisher', anonymous=True)
    ir_bottom_pub = rospy.Publisher('pololu/bottom_IR/data', Float64, queue_size=1)
    ir_top_pub = rospy.Publisher('pololu/top_IR/data', Float64, queue_size=1)
    rate = rospy.Rate(1000)
    ir_state = Float64()

    motor_srv = rospy.Service('motor_cmd', PololuCmd, motor_handler)
    steering_srv = rospy.Service('steering_cmd',PololuCmd, steering_handler)

    with pololu.Controller(0) as steering,     \
            pololu.Controller(1) as motor,     \
            pololu.Controller(2) as ir_bottom, \
            pololu.Controller(3) as ir_top:
        while not rospy.is_shutdown():
            ir_state.data = ir_bottom.get_position()
            ir_bottom_pub.publish(ir_state)

            ir_state.data = ir_top.get_position()
            ir_top_pub.publish(ir_state)

            if motor_cmd != 0:
                motor.set_target(motor_cmd)
                motor_cmd = 0

            if steering_cmd != 0:
                steering.set_target(steering_cmd)
                steering_cmd = 0

            rate.sleep()
