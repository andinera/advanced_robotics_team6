#!/usr/bin/env python

from drivers import dummy_pololu
from drivers import phidget
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

IR_ANGLE = quaternion_from_euler(0, 0, 1)[3]

if __name__ == '__main__':
    y = 0
    x = 0
    for i in range(0, 90):
        y += math.sin(math.radians(i))
        x += math.cos(math.radians(i))
    angle = math.atan2(y, x)
    print math.degrees(angle)
