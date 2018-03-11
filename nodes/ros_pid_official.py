#!/usr/bin/env python

import rospy
import time
from pololu import Controller

# import message types
from std_msgs.msg import Float64
from std_msgs.msg import Bool

MIN = 4095
MAX = 7905
CENTER = 6000

def get_pid_control(ctrl_msg, steering):

    steering_cmd = int(ctrl_msg.data)
    steering_cmd += CENTER
    rospy.loginfo("Position Command:\t%d", steering_cmd)

    # send position command to steering servo
    # (don't think this if-else structure should be necessary because of
    # upper_limit and lower_limit params set in launch file)
    if steering_cmd > MAX:
        steering.set_target(MAX)
    elif steering_cmd < MIN:
        steering.set_target(MIN)
    else:
        steering.set_target(steering_cmd)

def pid_broadcaster():

    with Controller(0) as steering, Controller(1) as motor, \
         Controller(2) as ir_bottom, Controller(3) as ir_top:

        # Publisher init
        enable_pub = rospy.Publisher("robot/pid/steering/enable", Bool, queue_size=1)
        setpoint_pub = rospy.Publisher("robot/pid/steering/setpoint", Float64, queue_size=1)
        state_pub = rospy.Publisher("robot/pid/steering/state", Float64, queue_size=1)

        # enable PID controller
        pid_enable_msg = Bool()
        pid_enable_msg.data = True
        enable_pub.publish(pid_enable_msg)

        # set zero intial velocity
        motor.set_target(CENTER)
        steering.set_target(CENTER)
        time.sleep(2)

        # define setpoint and state messages
        setpoint_msg = Float64()
        state_msg = Float64()

        # define setpoint by averaging initial position data
        distance = []
        for i in range(50):
            distance.append(ir_bottom.get_position())

        # average of initial IR sensor data
        setpoint = sum(distance) / len(distance)
        setpoint_msg.data = setpoint
        setpoint_pub.publish(setpoint_msg)
        print "Setpoint = ",setpoint," cm"

        # set forward speed
        offset = 300
        motor.set_target(CENTER + offset)

        # Subscriber init
        controlEffort_sub = rospy.Subscriber("robot/pid/steering/control_effort", Float64, get_pid_control, steering)

        # Iterate at 100 Hz
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():

            # get position reading from IR sesnor(s)
            position = []
            for i in range(1,1):
                position.append(ir_bottom.get_position())
                time.sleep(0.01)
            
            position = sum(position) / int(len(distance))
            rospy.loginfo("IR Position:\t%f", position)

            # use heading data to correct position measurement

            state_msg.data = position
            state_pub.publish(state_msg)

            # Iterate at frequency of rate
            rate.sleep()

if __name__ == '__main__':
    # Node init
    rospy.init_node('pid_broadcaster', anonymous=True)

    try:
        pid_broadcaster()
    except rospy.ROSInterruptException:
        pass
