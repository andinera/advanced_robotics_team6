#!/usr/bin/env python

import rospy
from time import sleep
from pololu import Controller

# import message types
from std_msgs.msg import Float64
from std_msgs.msg import Bool

MIN = 4095
MAX = 7905
CENTER = 6000

def get_pid_control(ctrl_msg, ir_controller):

    steering_cmd = int(ctrl_msg.data)
    ir_controller.pid_control_effort = steering_cmd
    print 'ctrl msg {}'.format(ctrl_msg.data)

def ir_pid_init(ir, ir_controller):
	# enable PID controller
	ir_enable = "ir/{}/pid/steering/enable".format(ir)
	enable_pub = rospy.Publisher(ir_enable, Bool, queue_size=1)
	pid_enable_msg = Bool()
	pid_enable_msg.data = True
	enable_pub.publish(pid_enable_msg)

	# define setpoint by averaging initial position data
	ir_setpoint = "ir/{}/pid/steering/setpoint".format(ir)
	setpoint_pub = rospy.Publisher(ir_setpoint, Float64, queue_size=1)
	setpoint_msg = Float64()
	distance = []
	for i in range(50):
		distance.append(ir_controller.get_position())
	setpoint = sum(distance) / len(distance)
	setpoint_msg.data = setpoint
	setpoint_pub.publish(setpoint_msg)
	print "Setpoint for ir_{} = {} cm".format(ir, setpoint)
	
	# Initialize steering state publisher
	ir_state = "ir/{}/pid/steering/state".format(ir)
	state_pub = rospy.Publisher(ir_state, Float64, queue_size=1)
	state_msg = Float64()

	# Initialize control effort subscriber
	ir_control_effort = "ir/{}/pid/steering/control_effort".format(ir)
	control_effort_sub = rospy.Subscriber(ir_control_effort, Float64, get_pid_control, ir_controller)
	
	return state_pub, state_msg, control_effort_sub


def pid_odroid():

	# Initialize Pololu Controllers
    with Controller(0) as steering, Controller(1) as motor, \
         Controller(2) as ir_bottom, Controller(3) as ir_top:
			 
        ir_bottom_state_pub, ir_bottom_state_msg, ir_bottom_control_effort_sub = ir_pid_init('bottom', ir_bottom)
        ir_top_state_pub, ir_top_state_msg, ir_top_control_effort_sub = ir_pid_init('bottom', ir_top)

        # set zero intial velocity and steering
        motor.set_target(CENTER)
        steering.set_target(CENTER)
        
        # set forward speed
        offset = 300
        motor.set_target(CENTER + offset)

        # Iterate at 50 Hz
        # Rate based on Pololu documentation
        # States most servos are designed for 50 Hz operation
        rate = rospy.Rate(50)

        i = 1
        while not rospy.is_shutdown():

            # get position reading from IR sesnor(s)
            ir_bottom_position = []
            ir_top_position = []
            for i in range(10):
                ir_bottom_position.append(ir_bottom.get_position())
                ir_top_position.append(ir_top.get_position())
            ir_bottom_position = sum(ir_bottom_position) / int(len(ir_bottom_position))
            rospy.loginfo("Bottom IR Position:\t%f", ir_bottom_position)
            ir_top_position = sum(ir_top_position) / int(len(ir_top_position))
            rospy.loginfo("Top IR Position:\t%f", ir_top_position)

            # use heading data to correct position measurement
            
            # Reset IR controller control efforts to 0
            ir_bottom.pid_control_effort = 0
            ir_top.pid_control_effort = 0

            # Publish IR states to PIDs
            ir_bottom_state_msg.data = ir_bottom_position
            ir_bottom_state_pub.publish(ir_bottom_state_msg)
            ir_top_state_msg.data = ir_top_position
            ir_top_state_pub.publish(ir_top_state_msg)

            # send position command to steering servo
            #steering_cmd = (ir_bottom.pid_control_effort + ir_top.pid_control_effort) / 2
            #rospy.loginfo("Position Command:\t%d", steering_cmd)
            #steering.set_target(steering_cmd)

            # Iterate at frequency of rate
            rate.sleep()
            i += 1
            print 'test {}'.format(i)

if __name__ == '__main__':
    # Node init
    rospy.init_node('pid_odroid', anonymous=True)

    try:
        pid_odroid()
    except rospy.ROSInterruptException:
        pass
