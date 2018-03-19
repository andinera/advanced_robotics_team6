#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Bool
import math

class Driver:
    # Initialize PID communications
    def __init__(self, sensor, controller, active, num_states_stored):
        # Attributes passed during initialization
        self.sensor = sensor            # Name of sensor
        self.controller = controller    # Related Pololu Controller
        self.active = active            # Sensor is running
        self.num_states_stored = num_states_stored        # Number of states to be saved
        # Attributes used for PID control
        self.control_effort = 0         # PID control
        # PID messages
        self.setpoint = Float64()
        self.state = Float64()
        # Attributes used for miscellaneous heuristics
        self.ignore = False             # Ignore sensor
        self.turning = False            # Entering a corner
        self.reported_states = []       # Last states sent to the PID
        self.recorded_states = []       # Last states measured by IMU

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

    def pid_control_effort_callback(self, data):
        self.control_effort = int(data.data)

    # Initialize IR setpoint
    def ir_setpoint(self, pololu_connected, num_readings, dummy_ir_value):
        if pololu_connected:
            self.recorded_states = []
            for i in range(num_readings):
                self.recorded_states.append(self.controller.get_position())
            self.setpoint.data = sum(self.recorded_states) / float(len(self.recorded_states))
        else:
            self.setpoint.data = dummy_ir_value
        self.setpoint_pub.publish(self.setpoint)
        if len(self.reported_states) >= self.num_states_stored:
            del self.reported_states[0]
        self.reported_states.append(self.setpoint.data)
        self.state.data = self.setpoint.data
        print "Setpoint for {} = {} cm".format(self.sensor, self.setpoint.data)

    # Initialize IMU setpoint
    def imu_setpoint(self, setpoint=None):
        if setpoint:
            self.setpoint.data = setpoint
        else:
            y = 0
            x = 0
            for state in self.recorded_states:
                y += math.sin(state)
                x += math.cos(state)
            heading = math.atan2(y, x)
            self.setpoint.data = heading
        self.setpoint_pub.publish(self.setpoint)
        if len(self.reported_states) >= self.num_states_stored:
            del self.reported_states[0]
        self.reported_states.append(self.setpoint.data)
        self.state.data = self.setpoint.data
        print "Setpoint for {} = {} degrees".format(self.sensor, math.degrees(self.setpoint.data))

    # Publish sensor state
    def publish_state(self, state):
        self.state.data = state
        if len(self.reported_states) >= self.num_states_stored:
            del self.reported_states[0]
        self.reported_states.append(state)
        self.state_pub.publish(self.state)
