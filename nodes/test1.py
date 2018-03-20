#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool, SetBoolResponse


def handler(req):
    for i in range(10):
        print req.data
        rospy.sleep(1)
    return SetBoolResponse(True, 'hello test2 from test1')

if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=True)
    rate = rospy.Rate(1)
    s = rospy.Service('tester', SetBool, handler)
    while not rospy.is_shutdown():
        print 'this is test1'
        rate.sleep()
