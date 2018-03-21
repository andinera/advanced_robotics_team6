#!/usr/bin/env python

import rospy
from drivers import pololu
import sys
import csv
import math

import thread

class _GetchUnix:
    def __init__(self):
        import tty, sys
        self.ch = None

    def __call__(self, cmd):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            cmd[0] = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

steering_cmd = 6000
motor_cmd = 6000

if __name__ == "__main__":
    rospy.init_node('manual_control', anonymous=True)
    with pololu.Controller(0) as steering,     \
            pololu.Controller(1) as motor,     \
            pololu.Controller(2) as ir_bottom, \
            pololu.Controller(3) as ir_top:

            cmd = [None]
            getch = _GetchUnix()
            thread.start_new_thread(getch, (cmd,))

            file_name = "IR_manual_032118:{}.csv".format(rospy.get_rostime())
            csv_out = open(file_name, 'a')
            writer = csv.writer(csv_out)
            writer.writerow(['bottom_error', 'top_error', 'ir_bottom_diff', 'ir_top_diff'])

            bottom_setpoint = ir_bottom.get_position()
            top_setpoint = ir_top.get_position()

            bottom_state = bottom_setpoint
            top_state = top_setpoint

            rate = rospy.Rate(50)

            while not rospy.is_shutdown():

                if cmd[0] != None:
                    if cmd[0] == 'w':
                        motor_cmd += 100
                        motor.set_target(motor_cmd)
                    if cmd[0] == 'a':
                        steering_cmd -= 100
                        steering.set_target(steering_cmd)
                    if cmd[0] == 's':
                        motor_cmd -= 100
                        motor.set_target(motor_cmd)
                    if cmd[0] == 'd':
                        steering_cmd += 100
                        steering.set_target(steering_cmd)
                    if cmd[0] ==  ' ':
                        motor.set_target(6000)
                        steering.set_target(6000)
                    if ord(cmd[0]) ==  127:
                        motor.set_target(6000)
                        steering.set_target(6000)
                        sys.exit(1)
                    cmd = [None]
                    thread.start_new_thread(getch, (cmd,))

                prev_bottom_state = bottom_state
                prev_top_state = top_state

                bottom_state = 0
                for _ in range(10):
                    bottom_state += ir_bottom.get_position()
                bottom_state /= 10

                top_state = 0
                for _ in range(10):
                    top_state += ir_top.get_position()
                top_state /= 10

                bottom_error = math.fabs(bottom_setpoint - bottom_state)     # [cm]
                top_error = math.fabs(top_setpoint - top_state)              # [cm]

                # finite differencing on state to estimate derivative (divide by timestep?)
                ir_bottom_diff = math.fabs(bottom_state - prev_bottom_state)    # [cm]
                ir_top_diff = math.fabs(top_state - prev_top_state)             # [cm]

                writer.writerow([bottom_error, top_error, ir_bottom_diff, ir_top_diff])

                rate.sleep()
