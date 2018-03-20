#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool, SetBoolResponse


if __name__ == "__main__":
    rospy.wait_for_service('tester')
    try:
        test = rospy.ServiceProxy('tester', SetBool)
        print 'this is test2'
        resp = test(True)
        print resp.success
        print resp.message
        print 'this is test2'
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
