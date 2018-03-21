#!/usr/bin/env python

import rospy

if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=True)
    lst = [1, 2, 3, 4, 5, 4, 3, 2, 1]
    del_lst = lst[:]
    print lst
    for i in lst[:]:
        print i
        print lst
        lst.remove(i)
        print lst
        print
