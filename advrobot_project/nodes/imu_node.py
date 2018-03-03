#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu


def callback(data):
    print data

if __name__ == '__main__':
    rospy.init_node('imu')
    sub = rospy.Subscriber('imu/data', Imu, callback)
    rospy.spin()
