#!/usr/bin/env python

import rospy

try:
    offline = rospy.get_param('offline')
except KeyError:
    offline = True

import cns_driver
if offline:
    import dummy_pololu_driver
else:
    import pololu_driver
import pid_driver
