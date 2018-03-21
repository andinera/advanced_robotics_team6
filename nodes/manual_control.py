#!/usr/bin/env python

import rospy
from drivers import pololu
import sys

class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()

steering_cmd = 6000
motor_cmd = 6000

if __name__ == "__main__":
    rospy.init_node('manual_control', anonymous=True)
    with pololu.Controller(0) as steering,     \
            pololu.Controller(1) as motor,     \
            pololu.Controller(2) as ir_bottom, \
            pololu.Controller(3) as ir_top:

            getch = _Getch()

            rate = rospy.Rate(50)

            while not rospy.is_shutdown():

                cmd = getch()
                if cmd == 'w':
                    motor_cmd += 100
                    motor.set_target(motor_cmd)
                if cmd == 'a':
                    steering_cmd -= 100
                    steering.set_target(steering_cmd)
                if cmd == 's':
                    motor_cmd -= 100
                    motor.set_target(motor_cmd)
                if cmd == 'd':
                    steering_cmd += 100
                    steering.set_target(steering_cmd)
                if cmd ==  ' ':
                    motor.set_target(6000)
                    steering.set_target(6000)
                if ord(cmd) ==  127:
                    motor.set_target(6000)
                    steering.set_target(6000)
                    sys.exit(1)

            rate.sleep()
