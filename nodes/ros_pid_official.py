#!/usr/bin/env python

import rospy

from drivers.pololu import Controller
# from drivers.dummy_pololu import Controller

# import message types
from std_msgs.msg import Float64
from std_msgs.msg import Bool

MIN = 4095
MAX = 7905
CENTER = 6000

# Callback from PID control effort subscriber
def pid_control_effort_callback(msg, controller):
    control_effort = int(msg.data)
    controller.pid_control_effort = control_effort

# Initialize PID communications
def pid_init(sensor, controller):

    # Enable PID controller
    pid_enable = "sensor/{}/pid/enable".format(sensor)
    enable_pub = rospy.Publisher(pid_enable,
                                 Bool,
                                 latch=True,
                                 queue_size=1)
    pid_enable_msg = Bool()
    pid_enable_msg.data = True
    enable_pub.publish(pid_enable_msg)

    # Define PID setpoint
    # Take average of multiple sensor readings
    pid_setpoint = "sensor/{}/pid/setpoint".format(sensor)
    setpoint_pub = rospy.Publisher(pid_setpoint,
                                   Float64,
                                   latch=True,
                                   queue_size=1)
    setpoint_msg = Float64()
    measurement = []
    for i in range(50):
        measurement.append(controller.get_position())
    setpoint = sum(measurement) / len(measurement)
    setpoint_msg.data = setpoint
    # setpoint_msg.data = 100					# False data for dummy testing
    setpoint_pub.publish(setpoint_msg)
    print "Setpoint for {} = {} cm".format(sensor, setpoint)

    # Initialize steering state publisher
    pid_state = "sensor/{}/pid/state".format(sensor)
    state_pub = rospy.Publisher(pid_state,
                                Float64,
                                latch=True,
                                queue_size=1)
    state_msg = Float64()

    # Initialize control effort subscriber
    pid_control_effort = "sensor/{}/pid/control_effort".format(sensor)
    control_effort_sub = rospy.Subscriber(pid_control_effort,
                                          Float64,
                                          pid_control_effort_callback,
                                          controller)

    return enable_pub,          \
           setpoint_pub,        \
           state_pub,           \
           state_msg,           \
           control_effort_sub   \

def odroid():

    # Initialize Pololu Controllers
    with Controller(0) as steering,  \
         Controller(1) as motor,     \
         Controller(2) as ir_bottom, \
         Controller(3) as ir_top:    \

        # Initialize publishers and subscriber for bottom IR sensor
        ir_bottom_enable_pub,   \
        ir_bottom_setpoint_pub, \
        ir_bottom_state_pub,    \
        ir_bottom_state_msg,    \
        ir_bottom_control_effort_sub = pid_init('bottom_IR', ir_bottom)

        # Initialize publishers and subscriber for top IR sensor
        ir_top_enable_pub,    \
        ir_top_setpoint_pub,  \
        ir_top_state_pub,     \
        ir_top_state_msg,     \
        ir_top_control_effort_sub = pid_init('top_IR', ir_top)

        # Set zero intial velocity and steering
        motor.set_target(CENTER)
        steering.set_target(CENTER)

        # Set forward speed
        offset = 300
        motor.set_target(CENTER + offset)

        # Iterate at 50 Hz
        # Rate based on Pololu documentation
        # States most servos are designed for 50 Hz operation
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():

            # Get measurement reading from sensor(s)
            ir_bottom_position = []
            ir_top_position = []
            for i in range(10):
                ir_bottom_position.append(ir_bottom.get_position())
                ir_top_position.append(ir_top.get_position())
            ir_bottom_position = sum(ir_bottom_position)            \
                                 / int(len(ir_bottom_position))
            rospy.loginfo("Bottom IR Position:\t%f", ir_bottom_position)
            ir_top_position = sum(ir_top_position)                  \
                              / int(len(ir_top_position))
            rospy.loginfo("Top IR Position:\t%f", ir_top_position)

            # TODO: use heading data to correct position measurement

            # Reset controller control efforts to 0
            ir_bottom.pid_control_effort = 0
            ir_top.pid_control_effort = 0

            # Publish sensor states to PIDs
            ir_bottom_state_msg.data = ir_bottom_position
            ir_bottom_state_pub.publish(ir_bottom_state_msg)
            ir_top_state_msg.data = ir_top_position
            ir_top_state_pub.publish(ir_top_state_msg)

            # Iterate at frequency of rate
            rate.sleep()

            # Send position command to steering servo
            steering_cmd = (ir_bottom.pid_control_effort            \
                            + ir_top.pid_control_effort) / 2
            steering_cmd += CENTER
            rospy.loginfo("Position Command:\t%d", steering_cmd)
            steering.set_target(steering_cmd)


if __name__ == '__main__':
    # Node init
    rospy.init_node('odroid_node', anonymous=True)

    try:
        odroid()
    except rospy.ROSInterruptException:
        pass