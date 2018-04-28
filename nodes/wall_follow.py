#!/usr/bin/env python

import rospy
import sys
import os
from threading import Thread, Event

from advanced_robotics_team6.scripts import *


FREQUENCY = 50

if __name__ == '__main__':
    # Initialize node
    rospy.init_node('wall_follow_node', anonymous=True)
    DEV = rospy.get_param('~dev')

    event = Event()

    # Initialize Wall_Follower
    try:
        wall_follower = globals()[DEV].wall_follower.Wall_Follower(event)
    except KeyError, e:
        print "Developer not specified:", e
        nodes = os.popen('rosnode list').readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")
        for node in nodes:
            os.system("rosnode kill "+ node)
        sys.exit()

    # Run state machine
    Thread(target=wall_follower.execute).start()
    # wall_follower.execute()
    # Initialize timer to iterate at set frequency
    timer = rospy.get_rostime() + rospy.Duration(1.0/FREQUENCY)
    # Begin iterations at set freuqency
    while not rospy.is_shutdown():
        # Allow processes waiting on event to run
        event.set()
        # Iterate at set frequency
        while not rospy.is_shutdown() and timer > rospy.get_rostime():
            rospy.sleep(0.1/FREQUENCY)
        # Reset freuqency timer
        timer += rospy.Duration(1.0/FREQUENCY)
    # Perform any necessary cleaning up before ending script
wall_follower.finish()
