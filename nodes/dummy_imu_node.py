#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, MagneticField
from random import uniform


FREQUENCY = 500

if __name__ == '__main__':
    rospy.init_node('dummy_imu', anonymous=True)

    pub_data = rospy.Publisher('imu/data_raw', Imu, latch=True, queue_size=1)
    pub_mag = rospy.Publisher('imu/mag', MagneticField, latch=True, queue_size=1)

    imu = Imu()
    mag = MagneticField()
    timer = rospy.get_rostime() + rospy.Duration(1.0/FREQUENCY)

    while not rospy.is_shutdown():
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = 'imu'
        # imu.orientation.x = 0
        # imu.orientation.y = 0
        # imu.orientation.z = 0
        # imu.orientation.w = 1
        imu.orientation_covariance = [-1,0,0,0,0,0,0,0,0]
        imu.angular_velocity.x = uniform(-1.0, 1.0)
        imu.angular_velocity.y = uniform(-1.0, 1.0)
        imu.angular_velocity.z = uniform(-1.0, 1.0)
        imu.angular_velocity_covariance = [0,0,0,0,0,0,0,0,0]
        imu.linear_acceleration.x = uniform(-1.0, 1.0)
        imu.linear_acceleration.y = uniform(-1.0, 1.0)
        imu.linear_acceleration.z = uniform(9.7, 9.9)
        imu.linear_acceleration_covariance = [0,0,0,0,0,0,0,0,0]

        mag.header.frame_id = 'imu'
        mag.header.stamp = rospy.Time.now()
        mag.magnetic_field.x = uniform(-1.0, 1.0)
        mag.magnetic_field.y = uniform(-1.0, 1.0)
        mag.magnetic_field.z = uniform(-1.0, 1.0)
        mag.magnetic_field_covariance = [0,0,0,0,0,0,0,0,0]

        pub_data.publish(imu)
        pub_mag.publish(mag)

        # Iterate at frequency of RATE
        while not rospy.is_shutdown() and timer > rospy.get_rostime():
            rospy.sleep(0.1/FREQUENCY)
        timer += rospy.Duration(1.0/FREQUENCY)
