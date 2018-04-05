#!/usr/bin/env python

import rospy

offline = rospy.get_param('offline')

import cns_driver
if offline:
    import dummy_pololu_driver
else:
    import pololu_driver
import pid_driver
