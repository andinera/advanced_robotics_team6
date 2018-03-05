#!/usr/bin/env python

import rospy
from pololu import Controller


MIN = 4095
MAX = 7905
CENTER = 6000

def follow_wall_on_right():
    with Controller(0) as steering, Controller(1) as motor, \
         Controller(2) as ir_one:
        rate = rospy.Rate(100)
        prev_distance = 0
        cornering = False
        # motor.set_target(MAX)
        while not rospy.is_shutdown():
            steer = steering.target
            distance = ir_one.get_position()
            # If program just started, wait another round
            if prev_distance == 0:
                pass
            # If close to the wall
            elif distance > 550:
                # If going around a corner
                if cornering:
                    cornering = False
                # If moving away from the wall, steer straight
                if prev_distance > distance:
                    steering.set_target(CENTER)
                # If moving towards the wall, turn hard left
                else:
                    steering.set_target(MIN)
            # If going around a corner, turn hard right
            elif distance < 300 and not cornering:
                cornering = True
                steering.set_target(MAX)
            # If far from the wall
            elif distance < 450:
                # If going around a corner
                if cornering:
                    # If can't sense the wall, turn hard right
                    if distance < 300:
                        steering.set_target(MAX)
                    # If moving towards the wall, steer straight
                    elif prev_distance < distance:
                        steering.set_target(CENTER)
                    # If moving away from the wall, turn hard right
                    else:
                        steering.set_target(MAX)
                # If moving towards the wall, steer straight
                elif prev_distance < distance:
                    steering.set_target(CENTER)
                # If moving away from the wall, turn hard right
                else:
                    steering.set_target(MAX)
            # If in the ideal range from the wall, turn wheels appropriately to
            # maintain center line
            else:
                # If going around a corner
                if cornering:
                    cornering = False
                difference = (distance - 450) / 100
                steer = (MAX - MIN) * difference + MIN
                steering.set_target(steer)
            prev_distance = distance
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('odroid')
    follow_wall_on_right()
