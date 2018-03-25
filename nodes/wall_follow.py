#!/usr/bin/env python

import rospy
import math
import csv
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from advanced_robotics_team6.srv import PololuCmd
from tf.transformations import euler_from_quaternion
from drivers import pid_driver

from carl import wall_follower as carl_wf
from kodie import wall_follower as kodie_wf
from wenjin import wall_follower as wenjin_wf
from shane import wall_follower as shane_wf


# Callbacks for recording data from IMU
def madgwick_callback(data, args):
    imu_wall_pid = args[0]
    imu_corner_pid = args[1]
    sensor_odometry_pub = args[2]
    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z
    w = data.orientation.w
    angles = euler_from_quaternion([x, y, z, w])
    if len(imu_wall_pid.recorded_states) >= NUM_READINGS:
        del imu_wall_pid.recorded_states[0]
    imu_wall_pid.recorded_states.append(angles[2])
    if len(imu_corner_pid.recorded_states) >= NUM_READINGS:
        del imu_corner_pid.recorded_states[0]
    imu_corner_pid.recorded_states.append(angles[2])

    odometry = Odometry()
    odometry.header = data.header
    odometry.pose.pose.orientation.x = x
    odometry.pose.pose.orientation.y = y
    odometry.pose.pose.orientation.z = z
    odometry.pose.pose.orientation.w = w
    #sensor_odometry_pub.publish(odometry)


# Callbacks for recording data from top IR sensor
def ir_top_callback(data, ir_top_pid):
    if len(ir_top_pid.recorded_states) >= NUM_READINGS:
        del ir_top_pid.recorded_states[0]
    ir_top_pid.recorded_states.append(data.data)


# Callbacks for recording data from top IR sensor
def ir_bottom_callback(data, ir_bottom_pid):
    if len(ir_bottom_pid.recorded_states) >= NUM_READINGS:
        del ir_bottom_pid.recorded_states[0]
    ir_bottom_pid.recorded_states.append(data.data)


# Main method
def odroid():

    # Initialize Pololu Controllers and PID drivers
    with pid_driver.Driver("bottom_ir",
                           NUM_STATES_STORED) as ir_bottom_pid,                \
         pid_driver.Driver("top_ir",
                           NUM_STATES_STORED) as ir_top_pid,                   \
         pid_driver.Driver("imu_wall",
                           NUM_STATES_STORED) as imu_wall_pid,                 \
         pid_driver.Driver("imu_corner",
                           NUM_STATES_STORED) as imu_corner_pid:

        rospy.wait_for_service('motor_cmd')
        rospy.wait_for_service('steering_cmd')
        motor_srv = rospy.ServiceProxy('motor_cmd', PololuCmd)
        steering_srv = rospy.ServiceProxy('steering_cmd', PololuCmd)

        # Intialize subscriber for bottom IR sensor
        ir_bottom_sub = rospy.Subscriber("pololu/bottom_ir/data",
                                          Float64,
                                          ir_bottom_callback,
                                          ir_bottom_pid)

        # Initialize subscriber for top IR sensor
        ir_top_sub = rospy.Subscriber("pololu/top_ir/data",
                                      Float64,
                                      ir_top_callback,
                                      ir_top_pid)

        sensor_odometry_pub = rospy.Publisher("imu/odometry",
                                              Odometry,
                                              queue_size=1)

        # Initialize subscriber for IMU
        madgwick_sub = rospy.Subscriber("imu/data",
                                        Imu,
                                        madgwick_callback,
                                        [imu_wall_pid, imu_corner_pid, sensor_odometry_pub])

        # Set zero intial velocity and steering
        motor_srv(MOTOR_CENTER)
        steering_srv(STEERING_CENTER)
        rospy.sleep(0.25)

        # Initialize Wall_Follower
        if DEV.lower() == "carl":
            wall_follower = carl_wf.Wall_Follower(ir_bottom_pid,
                                                  ir_top_pid,
                                                  imu_wall_pid,
                                                  imu_corner_pid,
                                                  motor_srv)
        elif DEV.lower() == "kodie":
            wall_follower = kodie_wf.Wall_Follower(ir_bottom_pid,
                                                   ir_top_pid,
                                                   imu_wall_pid,
                                                   imu_corner_pid,
                                                   motor_srv)
        elif DEV.lower() == "wenjin":
            wall_follower = wenjin_wf.Wall_Follower(ir_bottom_pid,
                                                    ir_top_pid,
                                                    imu_wall_pid,
                                                    imu_corner_pid,
                                                    motor_srv)
        elif DEV.lower() == "shane":
            wall_follower = shane_wf.Wall_Follower(ir_bottom_pid,
                                                   ir_top_pid,
                                                   imu_wall_pid,
                                                   imu_corner_pid,
                                                   motor_srv,
                                                   steering_srv)
        else:
            print "Developer not specified."
            return

        timer = rospy.get_rostime() + rospy.Duration(1.0/RATE)

        while not rospy.is_shutdown():

            while SMACH and wall_follower.sync == 1:
                pass
            if SMACH:
                wall_follower.sync = 1

            # Publish sensor states
            ir_bottom_pid.ir_publish_state()
            ir_top_pid.ir_publish_state()
            imu_wall_pid.imu_publish_state()
            imu_corner_pid.imu_publish_state(imu_wall_pid.state.data)

            if not SMACH:
                # Call state machine
                steering_cmd = wall_follower.execute()

                # Set steering target
                steering_cmd += STEERING_CENTER
                steering_srv(steering_cmd)

            # Iterate at frequency of RATE
            while timer > rospy.get_rostime():
                pass
            timer += rospy.Duration(1.0/RATE)
            print timer

        wall_follower.finish()


if __name__ == '__main__':

    # Initialize node
    rospy.init_node('odroid_node', anonymous=True)

    # Import param
    MIN = rospy.get_param('~min')
    MAX = rospy.get_param('~max')
    MOTOR_CENTER = rospy.get_param('~motor_center')
    STEERING_CENTER = rospy.get_param('~steering_center')
    RATE = rospy.get_param('~rate')
    NUM_READINGS = rospy.get_param('~num_readings')
    NUM_STATES_STORED = rospy.get_param('~num_states_stored')
    DEV = rospy.get_param('~developer')
    SMACH = rospy.get_param('~smach')

    try:
        odroid()
    except rospy.ROSInterruptException:
        pass
