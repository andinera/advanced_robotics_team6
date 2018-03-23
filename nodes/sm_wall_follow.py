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
from state_machines import wall_follow
import threading


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
    sensor_odometry_pub.publish(odometry)




# Callbacks for recording data from top IR sensor
def ir_bottom_callback(data, ir_bottom_pid):
    if len(ir_bottom_pid.recorded_states) >= NUM_READINGS:
        del ir_bottom_pid.recorded_states[0]
    ir_bottom_pid.recorded_states.append(data.data)

# Callbacks for recording data from top IR sensor
def ir_top_callback(data, ir_top_pid):
    if len(ir_top_pid.recorded_states) >= NUM_READINGS:
        del ir_top_pid.recorded_states[0]
    ir_top_pid.recorded_states.append(data.data)

# Main method
def odroid():

    # Initialize Pololu Controllers and PID drivers
    with pid_driver.Driver("IMU_CORNER",
                           NUM_STATES_STORED) as imu_corner_pid,          \
         pid_driver.Driver("IMU_WALL",
                           NUM_STATES_STORED) as imu_wall_pid,            \
         pid_driver.Driver("bottom_IR",
                           NUM_STATES_STORED) as ir_bottom_pid,              \
         pid_driver.Driver("top_IR",
                           NUM_STATES_STORED) as ir_top_pid:

        # Listed imus for printing
        pids = [ir_bottom_pid, ir_top_pid, imu_wall_pid, imu_corner_pid]

        rospy.wait_for_service('motor_cmd')
        rospy.wait_for_service('steering_cmd')
        motor_srv = rospy.ServiceProxy('motor_cmd', PololuCmd)
        steering_srv = rospy.ServiceProxy('steering_cmd', PololuCmd)

        # Calibrate IMU gyro biases
        if not DUMMY_MODE:
            rospy.wait_for_service('imu/calibrate')
            try:
                rospy.ServiceProxy('imu/calibrate', Empty)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        # Intialize subscriber for bottom IR sensor
        ir_bottom_sub = rospy.Subscriber("pololu/bottom_IR/data",
                                          Float64,
                                          ir_bottom_callback,
                                          ir_bottom_pid)

        # Initialize subscriber for top IR sensor
        ir_top_sub = rospy.Subscriber("pololu/top_IR/data",
                                      Float64,
                                      ir_top_callback,
                                      ir_top_pid)

        sensor_odometry_pub = rospy.Publisher("imu/odometry",
                                              Odometry,
                                              queue_size=1)

        # Initialize subscriber for IMU
        madgwick_sub = rospy.Subscriber("imu/data_madgwick",
                                        Imu,
                                        madgwick_callback,
                                        [imu_wall_pid,imu_corner_pid, sensor_odometry_pub])

        # Send setpoints to PIDs
        # Wait for recorded sensor data before publishing setpoint
        rospy.sleep(0.25)
        ir_bottom_pid.ir_setpoint()
        ir_top_pid.ir_setpoint()
        imu_corner_pid.imu_setpoint()
        imu_wall_pid.imu_setpoint(imu_corner_pid.setpoint.data)

        # Set zero intial velocity and steering
        motor_srv(MOTOR_CENTER)
        steering_srv(STEERING_CENTER)
        rospy.sleep(1)

        # Set forward speed
        motor_srv(MOTOR_SPEED)

        # Iteration rate
        rate = rospy.Rate(RATE)

        t = threading.Thread(target=wall_follow.main, args = (ir_bottom_pid,
                                                              ir_top_pid,
                                                              imu_wall_pid,
                                                              imu_corner_pid,
                                                              steering_srv))
        t.daemon = False
        t.start()

        while not rospy.is_shutdown():
            while not rospy.is_shutdown() and ir_bottom_pid.sync == 1:
                pass
            ir_bottom_pid.sync = 1

            # Publish sensor states
            ir_bottom_pid.ir_publish_state()
            ir_top_pid.ir_publish_state()
            imu_wall_pid.imu_publish_state()
            imu_corner_pid.imu_publish_state(imu_wall_pid.state.data)

            # Print statements
            for pid in pids:
                rospy.loginfo("Setpoint for {} = {}".format(pid.sensor, pid.setpoint.data))
            for pid in pids:
                rospy.loginfo("State for {} = {}".format(pid.sensor, pid.state.data))
            for pid in pids:
                rospy.loginfo("Control effort for {} = {}".format(pid.sensor, CENTER+pid.control_effort))
            print

            # Iterate at frequency of rate
            rate.sleep()


if __name__ == '__main__':

    # Initialize node
    rospy.init_node('odroid_node', anonymous=True)

    # Import param
    DUMMY_MODE = rospy.get_param('~dummy_mode')
    MIN = rospy.get_param('~min')
    MAX = rospy.get_param('~max')
    CENTER = rospy.get_param('~center')
    MOTOR_SPEED = rospy.get_param('~motor_speed')
    RATE = rospy.get_param('~rate')
    NUM_READINGS = rospy.get_param('~num_readings')
    TOP_IR_ANGLE = rospy.get_param('~top_ir_angle')
    IMU_THRESHOLD = math.radians(rospy.get_param('~imu_threshold'))
    DOOR_THRESHOLD = rospy.get_param('~door_threshold')
    CORNER_THRESHOLD = rospy.get_param('~corner_threshold')
    DOOR_ERROR_THRESHOLD = rospy.get_param('~door_error_threshold')
    CORNER_ERROR_THRESHOLD = rospy.get_param('~corner_error_threshold')
    IMU_RESET_THRESHOLD = rospy.get_param('~imu_reset_threshold')
    NUM_STATES_STORED = rospy.get_param('~num_states_stored')

    MOTOR_CENTER = rospy.get_param('~motor_center')
    STEERING_CENTER = rospy.get_param('~steering_center')

    # redefine DOOR and CORNER thresholds
    IMU_THRESHOLD = math.radians(15)

    try:
        odroid()
    except rospy.ROSInterruptException:
        pass
