#!/usr/bin/env python

import rospy
from advanced_robotics_team6.scripts.shane import wall_follower

if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=True)
    print 'hello'
    # wall_follower.Wall_Follower()
