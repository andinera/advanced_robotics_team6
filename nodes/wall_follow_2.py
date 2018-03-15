#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import PID        # Dummy IMU value if IMU is not connected

MIN = 4095
MAX = 7905
CENTER = 6000
MOTOR_SPEED = 6300              # Motor input
RATE = 50                       # Iteration rate; 50 Hz based on Pololu documentation
NUM_READINGS = 10               # Number of sensor readings per iteration
IR_ANGLE = math.radians(39)     # Angle of top IR sensor counter-clockwise from x-axis
USE_ANGLE_RELATIONSHIP = True
DOOR_THRESHOLD = 50             # will need to tune
CORNER_THRESHOLD = 500          # will need to tune
STATES_STORED = 1            # not currently being used

WPID = [-10, -3,-3]
TPID = [-10, -3,-3]


# Estimate for distance of car to wall based on measurement from top IR sensor
# and IMU heading
def ir_top_conversion(hypotenuse, orientation):
    angles = orientation.angle
    y = math.sin(angle)
    x = math.cos(angle)
    heading = math.atan2(y, x)

    return x

class Odometry_Data:
    def angle(self):
        return self.angle
    def x(self):
        return self.x
    def dx(self):
        return self.dx
    def dy(self):
        return self.dy

# Callback from kalman filter subscriber
def kalman_filter_callback(data, odometry):
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    angles = euler_from_quaternion([x, y, z, w])
    odometry.angle =angles[2]
    odometry.x = data.pose.pose.position.x
    odometry.dx = data.twist.twist.linear.x
    odometry.dy = data.twist.twist.linear.y
# Main method
def odroid():

    # Initialize Pololu Controllers
    with pololu.Controller(0) as steering,  \
         pololu.Controller(1) as motor,     \
         pololu.Controller(2) as ir_bottom, \
         pololu.Controller(3) as ir_top,    \
         phidget.Controller() as imu:

            wall_pid = PID.PID(WPID[0],WPID[1],WPID[2])
            turn_pid = PID.PID(TPID[0],TPID[1],TPID[2])
            wall = True
            doorway = False
            turn = False
            use_top_ir = True
            use_bottom_ir = True
            odometry = Odometry_Data()
            # Initialize subscriber for IMU
            kf_sub = rospy.Subscriber("odometry/filtered",
                                       Odometry,
                                       kalman_filter_callback,
                                       odometry)
            #initialize publisher
            rospy.sleep(1)

            # Set setpoints for wall follow and turning PIDs
            target_distance = 150
            wall_pid.SetPoint = target_distance
            turn_pid.SetPoint = math.radians(90)
            wall_pid.setSampleTime(1.0/RATE)
            turn_pid.setSampleTime(1.0/RATE)
            # Set zero intial velocity and steering
            motor.set_target(CENTER)
            steering.set_target(CENTER)

            # Set forward speed
            motor.set_target(MOTOR_SPEED)

            # Iteration rate
            rate = rospy.Rate(RATE)

            while not rospy.is_shutdown():
                #get measurements
                bottom_ir_state = ir_bottom.get_position()
                top_ir_state = ir_top.get_position()

                #determine state we are in, wall, doorway, turn
                bottom_IR_error = math.fabs(bottom_ir_state - target_distance)
                top_IR_error = math.fabs(ir_top_conversion(top_ir_state, odometry.angle) - target_distance)
                # If steadily following the setpoints
                if bottom_IR_error < DOOR_THRESHOLD and top_IR_error < DOOR_THRESHOLD:
                    if turn :
                        kfangle = 0
                        #reset angle to zero

                    if not wall :
                        wall = True
                        doorway = False
                        turn = False
                # If crossing doorway
                elif bottom_IR_error < CORNER_THRESHOLD and top_IR_error < CORNER_THRESHOLD:
                    # Top IR sensor detects doorway, ignore top IR sensor
                    if not doorway :
                        wall = False
                        doorway = True
                        turn = False

                    if top_IR_error > DOOR_THRESHOLD:
                        use_top_ir = False
                    # Top IR sensor is past doorway
                    else:
                        use_top_ir = True
                    # Bottom IR sensor detects doorway, ignore bottom IR sensor
                    if bottom_IR_error > DOOR_THRESHOLD:
                        use_bottom_ir = False
                    else:
                        use_bottom_ir = True
                # If cornering, ignore IR sensors, start IMU turn
                else:
                    # If starting a turn
                    if not turn and bottom_ir_state > CORNER_THRESHOLD:
                        wall = False
                        doorway = False
                        turn = True


                #select which measurements to send
                if wall and USE_ANGLE_RELATIONSHIP:
                    if (bottom_ir_state/top_ir_state - math.cos(IR_ANGLE))**2 < 1:
                        x_dist = bottom_ir_state * math.sqrt(1-(bottom_ir_state/top_ir_state - math.cos(IR_ANGLE))**2)
                        angle = math.acos(-math.cos(IR_ANGLE) + bottom_ir_state/top_ir_state)
                        #publish to KF and need to see if angle is offset by 90 degrees

                    else :
                        x_dist = bottom_ir_state
                        #publish to KF
                if wall and not USE_ANGLE_RELATIONSHIP:
                    x_dist = bottom_IR
                if doorway :
                    if use_bottom_ir :
                        x_dist = bottom_ir_state
                        #publish to KF
                    elif use_top_ir :
                        x_dist = ir_top_conversion(top_ir_state,odometry)
                        #publish to KF
                    #else publish nothing and it goes off imu automatically


                # get odometry from KF, how long do we need to wait?
                rospy.sleep(.001)
                # determine control input
                if wall or doorway:
                    wall_pid.update(odometry.x)
                    control_effort = wall_pid.output + CENTER
                else :
                    turn_pid.update(odometry.angle)
                    control_effort = turn_pid.output + CENTER
                #set control on servos
                if control_effort > MAX:
                    steering.set_target(MAX)
                elif control_effort < MIN:
                    steering.set_target(MIN)
                else :
                    steering.set_target(control_effort)

                # Iterate at frequency of rate
                rate.sleep()

                # Set steering target

                rospy.loginfo("Steering Command:\t%d", control_effort)


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('odroid_node', anonymous=True)

    try:
        odroid()
    except rospy.ROSInterruptException:
        pass
