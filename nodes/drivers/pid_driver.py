#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Bool
import math

class Driver:
    # Initialize PID communications
    def __init__(self, sensor, controller, active, imu=None, num_states_stored=1):
        self.sensor = sensor            # Name of sensor
        self.controller = controller    # Related Pololu Controller
        self.imu = imu                  # Related IMU
        self.active = active            # Sensor is running
        self.ignore = False             # Ignore sensor
        self.setpoint = 0               # PID setpoint
        self.control_effort = 0         # PID control effort
        self.turning = False            # Entering a corner
        self.num_states_stored = num_states_stored        # Number of states to be saved
        self.reported_states = []       # Last n reported states
        self.recorded_states = []       # Last n recorded states

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
        self.state = Float64()

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
    def ir_setpoint(self, pololu_connected, num_readings, dummy_ir_value, ir_state):
        setpoint_msg = Float64()
        if pololu_connected:
            states = []
            for i in range(num_readings):
                states.append(self.controller.get_position())
            setpoint = sum(states) / float(len(states))
            setpoint_msg.data = setpoint
        else:
            setpoint_msg.data = dummy_ir_value
        if self.sensor == "ir_top":
            setpoint_msg.data = Driver.ir_top_conversion(setpoint_msg.data, ir_state)
        self.setpoint_pub.publish(setpoint_msg)
        self.setpoint = setpoint_msg.data
        self.state.data = setpoint_msg.data
        if len(self.reported_states) > self.num_states_stored:
            del self.reported_states[0]
        self.reported_states.append(setpoint_msg.data)
        print "Setpoint for {} = {} cm".format(self.sensor, setpoint_msg.data)

    # Initialize IMU setpoint
    def imu_setpoint(self, imu_connected, dummy_imu_value, setpoint=None):
        setpoint_msg = Float64()
        if setpoint:
            setpoint_msg.data = setpoint
        elif imu_connected:
            while not rospy.is_shutdown():
                states = self.recorded_states
                y = 0
                x = 0
                for state in states:
                    y += math.sin(state)
                    x += math.cos(state)
                heading = math.atan2(y, x)
                setpoint_msg.data = heading
                break
        else:
            setpoint_msg.data = dummy_imu_value
        self.setpoint_pub.publish(setpoint_msg)
        self.setpoint = setpoint_msg.data
        self.state.data = setpoint_msg.data
        if len(self.reported_states) > self.num_states_stored:
            del self.reported_states[0]
        self.reported_states.append(setpoint_msg.data)
        print "Setpoint for {} = {} degrees".format(self.sensor, setpoint_msg.data)

    # Publish sensor state
    def publish_state(self, state):
        self.state.data = state
        self.state_pub.publish(self.state)

    # Estimate for distance of car to wall based on measurement from top IR sensor
    # and IMU heading
    @staticmethod
    def ir_top_conversion(self, hypotenuse, ir_state):
        if self.active:
            states = self.imu.recorded_states
            y = 0
            x = 0
            for state in states:
                y += math.sin(state)
                x += math.cos(state)
            heading = math.atan2(y, x)
            offset = self.setpoint - heading
        else:
            offset = 0
        return hypotenuse * math.cos(ir_state - offset)
