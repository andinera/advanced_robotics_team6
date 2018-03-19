#!/usr/bin/env python

import time
from drivers import dummy_pololu
from drivers import pid_driver
import rospy

IMU_CORNER = None
NUM_STATES_STORED = None
IMU_WALL = None
BOTTOM_IR = None
TOP_IR = None


if __name__ == '__main__':
    rospy.init_node('test_node', anonymous=True)

    # Initialize Pololu Controllers
    with dummy_pololu.Controller(0) as steering,  \
         dummy_pololu.Controller(1) as motor,     \
         dummy_pololu.Controller(2) as ir_bottom, \
         dummy_pololu.Controller(3) as ir_top,      \
    # Initialize PID drivers
    pid_driver.Driver("IMU_CORNER",             \
                    None,
                    IMU_CORNER,
                    NUM_STATES_STORED) as imu_corner_pid,          \
    pid_driver.Driver("IMU_WALL",
                    None,
                    IMU_WALL,
                    NUM_STATES_STORED) as imu_wall_pid,            \
    pid_driver.Driver("bottom_IR",
                    ir_bottom,
                    BOTTOM_IR,
                    NUM_STATES_STORED,
                    imu_corner_pid) as ir_bottom_pid,              \
    pid_driver.Driver("top_IR",
                    ir_top,
                    TOP_IR,
                    NUM_STATES_STORED,
                    imu_corner_pid) as ir_top_pid:

        print 'hello'
