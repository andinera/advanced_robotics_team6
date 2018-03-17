#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_srvs.srv import Empty


def callback(data, pub):
    tf_data = data
    tf_data.header = data.header
    tf_data.angular_velocity.x = data.angular_velocity.y
    tf_data.angular_velocity.y = data.angular_velocity.x
    tf_data.angular_velocity.z = data.angular_velocity.z
    tf_data.angular_velocity_covariance = data.angular_velocity_covariance
    tf_data.linear_acceleration.x = data.linear_acceleration.y
    tf_data.linear_acceleration.y = data.linear_acceleration.x
    tf_data.linear_acceleration.z = data.linear_acceleration.z
    tf_data.linear_acceleration_covariance = data.linear_acceleration_covariance
    pub.publish(tf_data)

def cal_callback(data):
    print "IMU calibrated"

if __name__ == '__main__':
    rospy.init_node('imu_listener', anonymous=True)
    pub = rospy.Publisher('imu/data_tf', Imu, queue_size=1)
    sub = rospy.Subscriber('imu/data_raw', Imu, callback, pub)

    try:
        rospy.spin()
    except rospy.ROSInterruptException, e:
        print e
