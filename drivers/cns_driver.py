#!/usr/bin/env python

import rospy
from multiprocessing import Manager

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

from advanced_robotics_team6.srv import PololuCmd


NUM_READINGS = 151

class CNS:

    def __init__(self):
        # Setup data structures for saving sensor input data which can be shared
        # across processes
        self.bottom_ir_states = Manager().list()
        self.top_ir_states = Manager().list()
        self.imu_states = {'orientation': {'x': Manager().list(),
                                           'y': Manager().list(),
                                           'z': Manager().list()},
                           'angular_velocity': {'x': Manager().list(),
                                                'y': Manager().list(),
                                                'z': Manager().list()},
                           'linear_acceleration': {'x': Manager().list(),
                                                   'y': Manager().list(),
                                                   'z': Manager().list()}}
        self.imu_raw_states = {'orientation': {'x': Manager().list(),
                                           'y': Manager().list(),
                                           'z': Manager().list()},
                           'angular_velocity': {'x': Manager().list(),
                                                'y': Manager().list(),
                                                'z': Manager().list()},
                           'linear_acceleration': {'x': Manager().list(),
                                                   'y': Manager().list(),
                                                   'z': Manager().list()}}

        # Initialize services for sending motor and steering commands
        rospy.wait_for_service('motor_cmd')

        rospy.wait_for_service('steering_cmd')
        self.motor_srv = rospy.ServiceProxy('motor_cmd', PololuCmd)
        self.steering_srv = rospy.ServiceProxy('steering_cmd', PololuCmd)

        # Intialize subscriber for bottom IR sensor
        self.bottom_ir_sub = rospy.Subscriber("pololu/ir/bottom/data",
                                          Float64,
                                          self.bottom_ir_callback)

        # Initialize subscriber for top IR sensor
        self.top_ir_sub = rospy.Subscriber("pololu/ir/top/data",
                                      Float64,
                                      self.top_ir_callback)

        # Initialize subscriber for IMU
        self.imu_sub = rospy.Subscriber("imu/data",
                                        Imu,
                                        self.imu_callback)
        self.imu_raw_sub = rospy.Subscriber("imu/raw_data",
                                        Imu,
                                        self.imu_raw_callback)
        # Sleep to allow the sensor data structures to populate
        rospy.sleep(0.25)

    # Callbacks for recording data from top IR sensor
    def bottom_ir_callback(self, data):
        if len(self.bottom_ir_states) >= NUM_READINGS:
            del self.bottom_ir_states[0]
        self.bottom_ir_states.append(data.data)

    # Callbacks for recording data from top IR sensor
    def top_ir_callback(self, data):
        if len(self.top_ir_states) >= NUM_READINGS:
            del self.top_ir_states[0]
        self.top_ir_states.append(data.data)

    # Callbacks for recording data from IMU
    def imu_callback(self, data):
        x = data.orientation.x
        y = data.orientation.y
        z = data.orientation.z
        w = data.orientation.w
        angles = euler_from_quaternion([x, y, z, w])
        quat = quaternion_from_euler(angles[0],angles[1],angles[2])
        print "quaternion:", quat
        if len(self.imu_states['orientation']['x']) >= NUM_READINGS:
            del self.imu_states['orientation']['x'][0]
        self.imu_states['orientation']['x'].append(angles[0])
        if len(self.imu_states['orientation']['y']) >= NUM_READINGS:
            del self.imu_states['orientation']['y'][0]
        self.imu_states['orientation']['y'].append(angles[1])
        if len(self.imu_states['orientation']['z']) >= NUM_READINGS:
            del self.imu_states['orientation']['z'][0]
        self.imu_states['orientation']['z'].append(angles[2])

        if len(self.imu_states['angular_velocity']['x']) >= NUM_READINGS:
            del self.imu_states['angular_velocity']['x'][0]
        self.imu_states['angular_velocity']['x'].append(data.angular_velocity.x)
        if len(self.imu_states['angular_velocity']['y']) >= NUM_READINGS:
            del self.imu_states['angular_velocity']['y'][0]
        self.imu_states['angular_velocity']['y'].append(data.angular_velocity.y)
        if len(self.imu_states['angular_velocity']['z']) >= NUM_READINGS:
            del self.imu_states['angular_velocity']['z'][0]
        self.imu_states['angular_velocity']['z'].append(data.angular_velocity.z)

        if len(self.imu_states['linear_acceleration']['x']) >= NUM_READINGS:
            del self.imu_states['linear_acceleration']['x'][0]
        self.imu_states['linear_acceleration']['x'].append(data.linear_acceleration.x)
        if len(self.imu_states['linear_acceleration']['y']) >= NUM_READINGS:
            del self.imu_states['linear_acceleration']['y'][0]
        self.imu_states['linear_acceleration']['y'].append(data.linear_acceleration.x)
        if len(self.imu_states['linear_acceleration']['z']) >= NUM_READINGS:
            del self.imu_states['linear_acceleration']['z'][0]
        self.imu_states['linear_acceleration']['z'].append(data.linear_acceleration.x)

    def imu_raw_callback(self, data):

        if len(self.imu_raw_states['angular_velocity']['x']) >= NUM_READINGS:
            del self.imu_raw_states['angular_velocity']['x'][0]
        self.imu_raw_states['angular_velocity']['x'].append(data.angular_velocity.x)
        if len(self.imu_raw_states['angular_velocity']['y']) >= NUM_READINGS:
            del self.imu_raw_states['angular_velocity']['y'][0]
        self.imu_raw_states['angular_velocity']['y'].append(data.angular_velocity.y)
        if len(self.imu_raw_states['angular_velocity']['z']) >= NUM_READINGS:
            del self.imu_raw_states['angular_velocity']['z'][0]
        self.imu_raw_states['angular_velocity']['z'].append(data.angular_velocity.z)

        if len(self.imu_raw_states['linear_acceleration']['x']) >= NUM_READINGS:
            del self.imu_raw_states['linear_acceleration']['x'][0]
        self.imu_raw_states['linear_acceleration']['x'].append(data.linear_acceleration.x)
        if len(self.imu_raw_states['linear_acceleration']['y']) >= NUM_READINGS:
            del self.imu_raw_states['linear_acceleration']['y'][0]
        self.imu_raw_states['linear_acceleration']['y'].append(data.linear_acceleration.x)
        if len(self.imu_raw_states['linear_acceleration']['z']) >= NUM_READINGS:
            del self.imu_raw_states['linear_acceleration']['z'][0]
        self.imu_raw_states['linear_acceleration']['z'].append(data.linear_acceleration.x)
