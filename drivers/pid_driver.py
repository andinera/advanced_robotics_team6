#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Bool
import math
import numpy

class PID:
    # Initialize PID communications
    def __init__(self, sensor, num_states_stored):
        # Attributes passed during initialization
        self.sensor = sensor            # Name of sensor
        self.num_states_stored = num_states_stored        # Number of states to be saved
        # Attributes used for PID control
        self.control_effort = 0         # PID control
        # PID messages
        self.setpoint = Float64()
        self.state = Float64()
        # Attributes used for miscellaneous heuristics
        self.ignore = False             # Ignore sensor
        self.reported_states = []       # Last states sent to the PID

        # Enable PID controller
        pid_enable = "odroid/{}/pid/enable".format(self.sensor)
        self.enable_pub = rospy.Publisher(pid_enable,
                                     Bool,
                                     latch=True,
                                     queue_size=1)
        pid_enable_msg = Bool()
        pid_enable_msg.data = True
        self.enable_pub.publish(pid_enable_msg)

        # Define PID setpoint
        pid_setpoint = "odroid/{}/pid/setpoint".format(self.sensor)
        self.setpoint_pub = rospy.Publisher(pid_setpoint,
                                       Float64,
                                       latch=True,
                                       queue_size=1)

        # Initialize sensor state publisher
        pid_state = "odroid/{}/pid/state".format(self.sensor)
        self.state_pub = rospy.Publisher(pid_state,
                                    Float64,
                                    latch=True,
                                    queue_size=1)

        # Initialize control effort subscriber
        pid_control_effort = "odroid/{}/pid/control_effort".format(self.sensor)
        self.control_effort_sub = rospy.Subscriber(pid_control_effort,
                                              Float64,
                                              self.pid_control_effort_callback)

    # Required for using in conjunction with a "with" statement
    def __enter__(self):
        return self

    # Required for using in conjunction with a "with" statement
    def __exit__(self, type, value, traceback):
        pass

    def __del__(self):
        pass

    # Callback for receiving PID control effort
    def pid_control_effort_callback(self, data):
        if math.isnan(data.data):
            self.control_effort = 0
        else:
            self.control_effort = int(data.data)

    # Initialize IR setpoint
    def ir_setpoint(self, states=None, setpoint=None):
        if setpoint:
            self.setpoint.data = setpoint
        else:
            stts = states[:]
            std_dev = numpy.std(stts)
            mean = numpy.mean(stts)
            for state in stts[:]:
                if state < mean-std_dev or state > mean+std_dev:
                    stts.remove(state)
            self.setpoint.data = numpy.mean(stts)
        self.setpoint_pub.publish(self.setpoint)
        if len(self.reported_states) >= self.num_states_stored:
            del self.reported_states[0]
        self.reported_states.append(self.setpoint.data)
        self.state.data = self.setpoint.data

    # Initialize IMU setpoint
    def imu_setpoint(self, states=None, setpoint=None):
        if setpoint:
            self.setpoint.data = setpoint
        else:
            stts = states[:]
            y = 0
            x = 0
            for state in stts:
                y += math.sin(state)
                x += math.cos(state)
            heading = math.atan2(y, x)
            self.setpoint.data = heading
        #if self.setpoint.data > math.pi:
        #    self.setpoint.data -= 2*math.pi
       	# elif self.setpoint.data < -math.pi:
        #    self.setpoint.data += math.pi
        self.setpoint_pub.publish(self.setpoint)
        if len(self.reported_states) >= self.num_states_stored:
            del self.reported_states[0]
        self.reported_states.append(self.setpoint.data)
        self.state.data = self.setpoint.data


    # Publish IR sensor state
    def ir_publish_state(self, states=None, state=None):
        if state and states:
            self.state.data = state
            stts = states
            reported_states = numpy.mean(stts)
            if len(self.reported_states) >= self.num_states_stored:
                del self.reported_states[0]
            self.reported_states.append(reported_states)
            self.state_pub.publish(self.state)
	    elif state:
	        self.state.data = state
            if len(self.reported_states) >= self.num_states_stored:
                del self.reported_states[0]
            self.reported_states.append(self.state.data)
            self.state_pub.publish(self.state)
	    else:
            stts = states
            #std_dev = numpy.std(stts)
            #mean = numpy.mean(stts)
            #for state in stts[:]:
            #       if state < mean-std_dev or state > mean+std_dev:
            #          stts.remove(state)
            self.state.data = numpy.mean(stts)
            if len(self.reported_states) >= self.num_states_stored:
                del self.reported_states[0]
            self.reported_states.append(self.state.data)
            self.state_pub.publish(self.state)

    # Publish IMU state
    def imu_publish_state(self, states=None, state=None):
        if state:
            self.state.data = state
        else:
            #stts = states[-4:-1]
            #y = 0
            #x = 0
            #for state in stts:
               #y += math.sin(state)
               #x += math.cos(state)
            #self.state.data = math.atan2(y, x)
	    self.state.data = states[-1]
            if self.state.data < self.setpoint.data - math.pi:
                self.state.data += 2*math.pi
            elif self.state.data > self.setpoint.data + math.pi:
                self.state.data -= 2*math.pi
        if len(self.reported_states) >= self.num_states_stored:
            del self.reported_states[0]
        self.reported_states.append(self.state.data)
        self.state_pub.publish(self.state)
