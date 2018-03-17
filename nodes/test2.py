#!/usr/bin/env python

import rospy
from test_module import Test_module
import math
from std_srvs.srv import Empty, SetBool


def client():
    rospy.wait_for_service('test')
    try:
        test = rospy.ServiceProxy('test', Empty)
        print 'hello'
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    client()
