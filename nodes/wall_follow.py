#!/usr/bin/env python

import rospy
import sys
import os
from threading import Thread, Event

from advanced_robotics_team6.scripts import *


class WallFollow:

    def __init__(self):
        # Retrieve dev parameter from parameter server
        self.dev = rospy.get_param('~dev')
        # Frequency for iteration
        self.frequency = 50
        # Event for synchronizing child scripts
        self.event = Event()
        # Initialize Wall_Follower script
        try:
            wall_follower = globals()[self.dev].wall_follower.Wall_Follower(event)
        except KeyError, e:
            print "Developer not specified:", e
            nodes = os.popen('rosnode list').readlines()
            for i in range(len(nodes)):
                nodes[i] = nodes[i].replace("\n","")
            for node in nodes:
                os.system("rosnode kill "+ node)
            sys.exit()
        # Initialize timer to iterate at set frequency
        self.timer = rospy.get_rostime()

    def iterate(self):
        # Run state machine
        Thread(target=wall_follower.execute).start()
        # Begin iterations at set freuqency
        while not rospy.is_shutdown():
            # Iterate at set frequency
            while not rospy.is_shutdown() and timer > rospy.get_rostime():
                rospy.sleep(0.1/self.frequency)
            # Reset freuqency timer
            self.timer += rospy.Duration(1.0/self.frequency)
            # Allow processes waiting on event to run
            self.event.set()
        # Perform any necessary cleaning up before ending script
        wall_follower.finish()

if __name__ == '__main__':
    rospy.init_node('wall_follow_node', anonymous=True)
    wf = WallFollow()
    wf.iterate()
