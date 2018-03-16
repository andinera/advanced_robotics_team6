#!/usr/bin/env python

import rospy
# from drivers import pololu
from multiprocessing import Pool


def hello(x):
    print 'test2', x

if __name__ == '__main__':
    x = range(50)
    print x
    pool_one = Pool(1)
    pool_two = Pool(1)
    results_one=pool_one.map(hello, x)
    results_two=pool_two.map(hello, x)
    for i in range(50):
        print 'test1: ', i
    # pool_one.close()
    # pool_one.join()
    # pool_two.close()
    # pool_two.join()
