#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

from advanced_robotics_team6.srv import PololuCmd


class CNS:

    def __init__(self):
        # Maximum length of each data list
        self.num_readings = 10
        # Setup data structures for saving sensor input data which can be shared
        # across processes
        self.ir_one_states = []
        self.ir_two_states = []
        self.imu_states = {'orientation': {'x': [],
                                           'y': [],
                                           'z': []},
                           'angular_velocity': {'x': [],
                                                'y': [],
                                                'z': []},
                           'linear_acceleration': {'x': [],
                                                   'y': [],
                                                   'z': []}}
        # Initialize services for sending motor and steering commands
        rospy.wait_for_service('motor_cmd')
        rospy.wait_for_service('steering_cmd')
        self.motor_srv = rospy.ServiceProxy('motor_cmd',
                                            PololuCmd)
        self.steering_srv = rospy.ServiceProxy('steering_cmd',
                                               PololuCmd)
        # Intialize subscriber for IR sensor data
        self.ir_one_sub = rospy.Subscriber("pololu/ir/one/data",
                                           Float64,
                                           self.ir_one_callback)
        self.ir_two_sub = rospy.Subscriber("pololu/ir/two/data",
                                           Float64,
                                           self.ir_two_callback)
        # Initialize subscriber for IMU data
        self.imu_sub = rospy.Subscriber("imu/data",
                                        Imu,
                                        self.imu_callback)
        # Sleep to allow the sensor data structures to populate
        rospy.sleep(0.25)

    # Callback for recording data from IR sensor one
    def ir_one_callback(self, data):
        if len(self.ir_one_states) >= self.num_readings:
            del self.ir_one_states[0]
        self.ir_one_states.append(data.data)

    # Callback for recording data from IR sensor two
    def ir_two_callback(self, data):
        if len(self.ir_two_states) >= self.num_readings:
            del self.ir_two_states[0]
        self.ir_two_states.append(data.data)

    # Callback for recording data from IMU
    def imu_callback(self, data):
        x = data.orientation.x
        y = data.orientation.y
        z = data.orientation.z
        w = data.orientation.w
        angles = euler_from_quaternion([x, y, z, w])
        #quat = quaternion_from_euler(angles[0],angles[1],angles[2])
        #print "z:", angles[2]
        if len(self.imu_states['orientation']['x']) >= self.num_readings:
            del self.imu_states['orientation']['x'][0]
        self.imu_states['orientation']['x'].append(angles[0])
        if len(self.imu_states['orientation']['y']) >= self.num_readings:
            del self.imu_states['orientation']['y'][0]
        self.imu_states['orientation']['y'].append(angles[1])
        if len(self.imu_states['orientation']['z']) >= self.num_readings:
            del self.imu_states['orientation']['z'][0]
        self.imu_states['orientation']['z'].append(angles[2])

        if len(self.imu_states['angular_velocity']['x']) >= self.num_readings:
            del self.imu_states['angular_velocity']['x'][0]
        self.imu_states['angular_velocity']['x'].append(data.angular_velocity.x)
        if len(self.imu_states['angular_velocity']['y']) >= self.num_readings:
            del self.imu_states['angular_velocity']['y'][0]
        self.imu_states['angular_velocity']['y'].append(data.angular_velocity.y)
        if len(self.imu_states['angular_velocity']['z']) >= self.num_readings:
            del self.imu_states['angular_velocity']['z'][0]
        self.imu_states['angular_velocity']['z'].append(data.angular_velocity.z)

        if len(self.imu_states['linear_acceleration']['x']) >= self.num_readings:
            del self.imu_states['linear_acceleration']['x'][0]
        self.imu_states['linear_acceleration']['x'].append(data.linear_acceleration.x)
        if len(self.imu_states['linear_acceleration']['y']) >= self.num_readings:
            del self.imu_states['linear_acceleration']['y'][0]
        self.imu_states['linear_acceleration']['y'].append(data.linear_acceleration.x)
        if len(self.imu_states['linear_acceleration']['z']) >= self.num_readings:
            del self.imu_states['linear_acceleration']['z'][0]
        self.imu_states['linear_acceleration']['z'].append(data.linear_acceleration.x)
