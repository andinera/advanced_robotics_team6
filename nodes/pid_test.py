#!/usr/bin/env python

import rospy
import time
import PID
from pololu import Controller


MIN = 4095
MAX = 7905
CENTER = 6000

def PIDControl(P,I,D):
    with Controller(0) as steering, Controller(1) as motor, \
         Controller(2) as ir_bottom:

	motor.set_target(CENTER)
	time.sleep(2)

        pid = PID.PID(P, I, D)

        ir_bottom.get_position()
        distances = []
        for i in range(1,50):
            distances.append(ir_bottom.get_position())
            time.sleep(.01)
        start_distance = int(sum(distances) / float(len(distances)))
        pid.SetPoint = start_distance
        print start_distance #average of initial IR sensor data
        pid.setSampleTime(0.1)

        motor.set_target(CENTER + 300)

        while not rospy.is_shutdown():

            distance = ir_bottom.get_position()
            pid.update(distance)
            output = int(pid.output) + CENTER
            print output,"\t",distance
            if output > MAX:
                steering.set_target(MAX)
            elif output < MIN:
                steering.set_target(MIN)
            else:
                steering.set_target(output)

            time.sleep(.01)

if __name__ == '__main__':
    rospy.init_node('odroid')
    P = 3
    I = 1
    D = 1
    PIDControl(P,I,D)
