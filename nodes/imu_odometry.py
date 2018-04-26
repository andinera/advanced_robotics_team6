#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

rospy.init_node('imu_odometry', anonymous=True)

def imu_callback(data, pub):
    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z
    w = data.orientation.w
    print "x,y,z,w: ",[x,y,z,w]
    odometry = Odometry()
    odometry.header = data.header
    odometry.pose.pose.orientation.x = x
    odometry.pose.pose.orientation.y = y
    odometry.pose.pose.orientation.z = z
    odometry.pose.pose.orientation.w = w
    pub.publish(odometry)

imu_pub = rospy.Publisher("imu/odometry",
        	                               Odometry,
                	                       queue_size=1)

imu_sub = rospy.Subscriber("imu/data",
                                        Imu,
                                        imu_callback,
										imu_pub)

rospy.spin()
