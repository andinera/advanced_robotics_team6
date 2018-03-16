#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import PID        # Dummy IMU value if IMU is not connected
import numpy as np
from drivers import pololu
from drivers import phidget
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
STOP_TURN_OFFSET = 50
WPID = [-10, -3,-3]
TPID = [-10, -3,-3]


# Estimate for distance of car to wall based on measurement from top IR sensor
# and IMU heading
def ir_top_conversion(hypotenuse, angle):
    
    x = math.cos(float(angle))

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
            odom0_pub = rospy.Publisher("robot/odom0/data",Odometry,queue_size=10)
            odom1_pub = rospy.Publisher("robot/odom1/data",Odometry,queue_size=10)
            reset_pub = rospy.Publisher("set_pose",PoseWithCovarianceStamped,queue_size=1)
            odom0 = Odometry()
            odom1 = Odometry()
            reset_msg = PoseWithCovarianceStamped()
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
                top_IR_error = math.fabs(ir_top_conversion(top_ir_state, odometry.angle()) - target_distance)
                #if in turn and not done
                if turn and turn_control_offset > STOP_TURN_OFFSET:
                    print "Continuing Turn"
                else:
                    # If steadily following the setpoints
                    if bottom_IR_error < DOOR_THRESHOLD and top_IR_error < DOOR_THRESHOLD:
                        if turn :
                            resetKF = true
                            print "reseting KF"
                        if not wall :
                            wall = True
                            doorway = False
                            turn = False
                            print "Starting Wall Follow"
                                # If crossing doorway
                    elif bottom_IR_error < CORNER_THRESHOLD and top_IR_error < CORNER_THRESHOLD:
                        # Top IR sensor detects doorway, ignore top IR sensor
                        if not doorway :
                            wall = False
                            doorway = True
                            turn = False
                            print "Starting doorway wall Follow"

                            if top_IR_error > DOOR_THRESHOLD:
                                use_top_ir = False
                                print "Not using top IR"
                                # Top IR sensor is past doorway
                        else:
                            use_top_ir = True
                            # Bottom IR sensor detects doorway, ignore bottom IR sensor
                        if bottom_IR_error > DOOR_THRESHOLD:
                            use_bottom_ir = False
                            print "Not using bottom IR"
                        else:
                            use_bottom_ir = True
                # If cornering, ignore IR sensors, start IMU turn
                    else:
                        # If starting a turn
                        if not turn and bottom_ir_state > CORNER_THRESHOLD:
                            wall = False
                            doorway = False
                            turn = True
                            print "Starting Turn"


                #select which measurements to send
                if wall and USE_ANGLE_RELATIONSHIP:
                    if (bottom_ir_state/top_ir_state - math.cos(IR_ANGLE))**2 < 1:
                        x_dist = bottom_ir_state * math.sqrt(1-(bottom_ir_state/top_ir_state - math.cos(IR_ANGLE))**2)
                        angle = math.acos(-math.cos(IR_ANGLE) + bottom_ir_state/top_ir_state) - math.pi
                        if resetKF:
                            reset_msg.header.stamp = rospy.Time.now()
                            reset_msg.header.frame_id = "base_link"
                            angles = quaternion_from_euler([0,0,angle])
                            reset_msg.pose.pose.orientaion = angles
                            reset_msg.pose.pose.position.x = x_dist
                            reset_pub.publish(reset_msg)

                        else:
                            odom0.header.stamp = rospy.Time.now()
                            odom0.header.frame_id = "base_link"
                            odom0.pose.pose.position.x = x_dist
                            angles = quaternion_from_euler([0,0,angle])
                            odom0.pose.pose.orientaion = angles
                            odom0_pub.publish(odom0)

                    else:
                        x_dist = bottom_ir_state
                        if resetKF:
                            reset_msg.header.stamp = rospy.Time.now()
                            reset_msg.header.frame_id = "base_link"
                            reset_msg.pose.pose.position.x = x_dist
                            reset_msg.pose.covariance = covariance
                            reset_pub.publish(reset_msg)
                        else :
                            odom1.header.stamp = rospy.Time.now()
                            odom1.header.frame_id = "base_link"
                            odom1.pose.pose.position.x = x_dist
                            odom1_pub.publish(odom1)
                        #publish to KF
                if wall and not USE_ANGLE_RELATIONSHIP:
                    x_dist = bottom_IR
                if doorway :
                    if use_bottom_ir :
                        x_dist = bottom_ir_state
                        odom1.header.stamp = rospy.Time.now()
                        odom1.header.frame_id = "base_link"
                        odom1.pose.pose.position.x = x_dist
                        odom1_pub.publish(odom1)
                        #publish to KF
                    elif use_top_ir :
                        x_dist = ir_top_conversion(top_ir_state,odometry)
                        #publish to KF
                        odom1.header.stamp = rospy.Time.now()
                        odom1.header.frame_id = "base_link"
                        odom1.pose.pose.position.x = x_dist
                        odom1_pub.publish(odom1)
                    #else publish nothing and it goes off imu automatically


                # get odometry from KF, how long do we need to wait?
                rospy.sleep(.001)
                # determine control input
                if wall or doorway:
                    wall_pid.update(odometry.x)
                    control_effort = wall_pid.output + CENTER
                else :
                    turn_pid.update(odometry.angle)
                    turn_control_offset = turn_pid.output
                    control_effort = turn_control_offset + CENTER
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
                rospy.loginfo("x distance:\t%d",odometry.pose.pose.position.x)
                rospy.loginfo("x distance:\t%d",odometry.twist.twist.linear.y)
                resetKF = False


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('odroid_node', anonymous=True)

    try:
        odroid()
    except rospy.ROSInterruptException:
        pass
