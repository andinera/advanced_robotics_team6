#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, MagneticField
from random import uniform


class DummyImu:

    def __init__(self):
        # Frequency for generating new IMU data
        self.frequency = 500
        # Initialize IMU data publishers
        self.pub_data = rospy.Publisher('imu/data_raw',
                                        Imu,
                                        latch=True,
                                        queue_size=1)
        self.pub_mag = rospy.Publisher('imu/mag',
                                       MagneticField,
                                       latch=True,
                                       queue_size=1)
        # Initialize message data types
        self.imu = Imu()
        self.mag = MagneticField()
        self.imu.header.frame_id = 'imu'
        # self.imu.orientation.x = 0
        # self.imu.orientation.y = 0
        # self.imu.orientation.z = 0
        # self.imu.orientation.w = 1
        self.imu.orientation_covariance = [-1,0,0,0,0,0,0,0,0]
        self.imu.angular_velocity.x = uniform(-1.0, 1.0)
        self.imu.angular_velocity.y = uniform(-1.0, 1.0)
        self.imu.angular_velocity.z = uniform(-1.0, 1.0)
        self.imu.angular_velocity_covariance = [0,0,0,0,0,0,0,0,0]
        self.imu.linear_acceleration.x = uniform(-1.0, 1.0)
        self.imu.linear_acceleration.y = uniform(-1.0, 1.0)
        self.imu.linear_acceleration.z = uniform(9.7, 9.9)
        self.imu.linear_acceleration_covariance = [0,0,0,0,0,0,0,0,0]
        # Transfer magnetic data
        self.mag.header.frame_id = 'imu'
        self.mag.header.stamp = rospy.Time.now()
        self.mag.magnetic_field.x = uniform(-1.0, 1.0)
        self.mag.magnetic_field.y = uniform(-1.0, 1.0)
        self.mag.magnetic_field.z = uniform(-1.0, 1.0)
        self.mag.magnetic_field_covariance = [0,0,0,0,0,0,0,0,0]
        # Initialize timer for managing iteration frequency
        self.timer = rospy.get_rostime()

    # Method for iteratively
    def iterate(self):
        while not rospy.is_shutdown():
            # Iterate at frequency of iteration
            while not rospy.is_shutdown() and self.timer > rospy.get_rostime():
                rospy.sleep(0.1/self.frequency)
            self.timer += rospy.Duration(1.0/self.frequency)
            # Transfer accelerometer and gyroscope data
            self.imu.header.stamp = rospy.Time.now()

            self.imu.angular_velocity.x += 0.1
            if self.imu.angular_velocity.x > 1.0:
                self.imu.angular_velocity.x = -1.0
            self.imu.angular_velocity.y += 0.1
            if self.imu.angular_velocity.y > 1.0:
                self.imu.angular_velocity.y = -1.0
            self.imu.angular_velocity.z += 0.1
            if self.imu.angular_velocity.z > 1.0:
                self.imu.angular_velocity.z = -1.0
            self.imu.linear_acceleration.x += 0.1
            if self.imu.linear_acceleration.x > 1.0:
                self.imu.linear_acceleration.x = -1.0
            self.imu.linear_acceleration.y += 0.1
            if self.imu.linear_acceleration.y > 1.0:
                self.imu.linear_acceleration.y = -1.0
            self.imu.linear_acceleration.z += 0.1
            if self.imu.linear_acceleration.z > 1.0:
                self.imu.linear_acceleration.z = -1.0

            # Transfer magnetic data
            self.mag.header.stamp = rospy.Time.now()
            self.mag.magnetic_field.x = uniform(-1.0, 1.0)
            if self.mag.magnetic_field.x > 1.0:
                self.mag.magnetic_field.x = -1.0
            self.mag.magnetic_field.y = uniform(-1.0, 1.0)
            if self.mag.magnetic_field.y > 1.0:
                self.mag.magnetic_field.y = -1.0
            self.mag.magnetic_field.z = uniform(-1.0, 1.0)
            if self.mag.magnetic_field.z > 1.0:
                self.mag.magnetic_field.z = -1.0
            # Publish data
            self.pub_data.publish(self.imu)
            self.pub_mag.publish(self.mag)

# Method for calling script directly
if __name__ == '__main__':
    rospy.init_node('dummy_imu', anonymous=True)
    di = DummyImu()
    di.iterate()
