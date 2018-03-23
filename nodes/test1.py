#!/usr/bin/env python

import rospy
from state_machines import example_state_machine
import threading
from random import uniform


if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=True)
