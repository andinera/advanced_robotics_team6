#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class ImuOdometry:

    def __init__(self):
        # Publisher for publishing odometry data
        self.imu_pub = rospy.Publisher("imu/odometry",
                	              Odometry,
                        	      queue_size=1)
        # Subscriber for subscribing to IMU data
        self.imu_sub = rospy.Subscriber("imu/data",
                                   Imu,
                                   self.imu_callback)
        # Initialize message data type
        self.odometry = Odometry()

    # Callback for imu data
    def imu_callback(self, data):
        x = data.orientation.x
        y = data.orientation.y
        z = data.orientation.z
        w = data.orientation.w
        self.odometry.header = data.header
        self.odometry.pose.pose.orientation.x = x
        self.odometry.pose.pose.orientation.y = y
        self.odometry.pose.pose.orientation.z = z
        self.odometry.pose.pose.orientation.w = w
        self.imu_pub.publish(self.odometry)

# Method for calling script directly
if __name__ == '__main__':
    rospy.init_node('imu_odometry', anonymous=True)
    io = ImuOdometry()
    rospy.spin()
