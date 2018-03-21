#!/usr/bin/env python

import rospy
from state_machines import example_state_machine

if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=True)
    print example_state_machine.main()
