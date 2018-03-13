#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

POLOLU_CONNECTED = False     # True if Pololu is connected
IMU_CONNECTED = False        # True if IMU is connected
DUMMY_IR_VALUE = 100        # Dummy IR sensor value if pololu is not connected
DUMMY_IMU_VALUE = 0         # Dummy IMU value if IMU is not connected

BOTTOM_IR = True
TOP_IR = True
IMU = True

if POLOLU_CONNECTED:
    from drivers import pololu
else:
    from drivers import dummy_pololu as pololu
from drivers import phidget
from drivers import pid_driver

MIN = 4095
MAX = 7905
CENTER = 6000
MOTOR_SPEED = 6300  # Motor input
RATE = 50           # Iteration rate; 50 Hz based on Pololu documentation
NUM_READINGS = 10   # Number of sensor readings per iteration
IR_ANGLE = math.radians(30)     # Angle of top IR sensor counter-clockwise from x-axis
IR_THRESHOLD = 10
IMU_THRESHOLD = 0.1


# Heuristic for not crashing
def heuristic(ir_bottom_pid, ir_top_pid, imu_pid, ir_bottom_state, ir_top_state, imu_state):
    # If a huge change on both IR sensors at the same time, assume cornering
    if (ir_bottom_pid.state > ir_bottom_state + IR_THRESHOLD or         \
            ir_bottom_pid.state < ir_bottom_state - IR_THRESHOLD) and   \
            (ir_top_pid.state > ir_top_state + IR_THRESHOLD or          \
            ir_top_pid.state < ir_top_state - IR_THRESHOLD):
        imu_pid.ignore = True
    # If no significant change
    elif ir_bottom_pid.state <= ir_bottom_state + IR_THRESHOLD and      \
            ir_bottom_pid.state >= ir_bottom_state - IR_THRESHOLD and   \
            ir_top_pid.state <= ir_top_state + IR_THRESHOLD and         \
            ir_top_pid.state >= ir_top_state - IR_THRESHOLD:
        if ir_bottom_pid.ignore:
            ir_bottom_pid.ir_setpoint(POLOLU_CONNECTED, NUM_READINGS, DUMMY_IR_VALUE, IR_ANGLE)
            ir_bottom_pid.ignore = False
        if ir_top_pid.ignore:
            ir_top_pid.ir_setpoint(POLOLU_CONNECTED, NUM_READINGS, DUMMY_IR_VALUE, IR_ANGLE)
            ir_top_pid.ignore = False
        if imu_pid.ignore:
            imu_pid.imu_setpoint(IMU_CONNECTED, DUMMY_IMU_VALUE)
            imu_pid.ignore = False
    # If huge change on only bottom IR sensor, assume door
    elif ir_bottom_pid.state <= ir_bottom_state + IR_THRESHOLD or      \
            ir_bottom_pid.state >= ir_bottom_state - IR_THRESHOLD:
        ir_top_pid.ignore = True
    # If huge change on only top IR sensor, assume door
    elif ir_top_pid.state <= ir_top_state + IR_THRESHOLD or            \
            ir_top_pid.state >= ir_top_state - IR_THRESHOLD:
        ir_bottom_pid.ignore = True
    # If all PIDs are ignored, unignore all PIDs
    if not ir_bottom_pid.ignore and not ir_top_pid.ignore and not imu_pid.ignore:
        ir_bottom_pid.ignore = False
        ir_top_pid.ignore = False
        imu_pid.ignore = False
    steering_cmd = 0
    # Take average of reliable sensors
    i = 0
    if not ir_bottom_pid.ignore:
        steering_cmd += ir_bottom_pid.control_effort
        i += 1
    if not ir_top_pid.ignore:
        steering_cmd += ir_top_pid.control_effort
        i += 1
    if not imu_pid.ignore:
        steering_cmd += imu_pid.control_effort
        i += 1
    steering_cmd /= i
    return steering_cmd

# Estimate for distance of car to wall based on measurement from top IR sensor
# and IMU heading
def ir_top_conversion(hypotenuse, imu):
    if IMU:
        angles = imu.angles
        y = 0
        x = 0
        for angle in angles:
            y += math.sin(angle)
            x += math.cos(angle)
        heading = math.atan2(y, x)
        offset = imu.setpoint - heading
    else:
        offset = 0
    return hypotenuse * math.cos(IR_ANGLE - offset)

# Callback from kalman filter subscriber
def kalman_filter_callback(data, controller):
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    angles = euler_from_quaternion([x, y, z, w])
    if len(controller.angles) < NUM_READINGS:
        controller.angles.append(angles[2])
    else:
        del controller.angles[0]
        controller.angles.append(angles[2])

# Main method
def odroid():

    # Initialize Pololu Controllers
    with pololu.Controller(0) as steering,  \
         pololu.Controller(1) as motor,     \
         pololu.Controller(2) as ir_bottom, \
         pololu.Controller(3) as ir_top,    \
         phidget.Controller() as imu:

         # Initialize PID drivers
         with pid_driver.Driver("bottom_IR", ir_bottom, BOTTOM_IR, imu) as ir_bottom_pid,    \
              pid_driver.Driver("top_IR", ir_top, TOP_IR, imu) as ir_top_pid,             \
              pid_driver.Driver("IMU", imu, IMU) as imu_pid:

            # Initialize subscriber for IMU
            kf_sub = rospy.Subscriber("odometry/filtered",
                                       Odometry,
                                       kalman_filter_callback,
                                       imu)

            # Send setpoints to PIDs
            if IMU:
                imu_pid.imu_setpoint(IMU_CONNECTED, DUMMY_IMU_VALUE)
            if BOTTOM_IR:
                ir_bottom_pid.ir_setpoint(POLOLU_CONNECTED, NUM_READINGS, DUMMY_IR_VALUE, IR_ANGLE)
            if TOP_IR:
                ir_top_pid.ir_setpoint(POLOLU_CONNECTED, NUM_READINGS, DUMMY_IR_VALUE, IR_ANGLE)

            # Set zero intial velocity and steering
            motor.set_target(CENTER)
            steering.set_target(CENTER)

            # Set forward speed
            motor.set_target(MOTOR_SPEED)

            # Iteration rate
            rate = rospy.Rate(RATE)

            while not rospy.is_shutdown():

                # Store states from previous round for heuristic
                temp_ir_bottom_state = ir_bottom_pid.state.data
                temp_ir_top_state = ir_top_pid.state.data
                temp_imu_state = imu_pid.state.data

                # Get measurement reading from sensor(s) and publish state
                if BOTTOM_IR:
                    ir_bottom_distance = []
                    for i in range(NUM_READINGS):
                        ir_bottom_distance.append(ir_bottom.get_position())
                    ir_bottom_distance = sum(ir_bottom_distance)                   \
                                         / float(len(ir_bottom_distance))
                    rospy.loginfo("Bottom IR Distance:\t%f", ir_bottom_distance)
                    ir_bottom_pid.publish_state(ir_bottom_distance)

                if TOP_IR:
                    ir_top_distance = []
                    for i in range(NUM_READINGS):
                        ir_top_distance.append(ir_top.get_position())
                    ir_top_distance = ir_top_conversion(sum(ir_top_distance), imu)      \
                                      / float(len(ir_top_distance))
                    rospy.loginfo("Top IR Distance:\t%f", ir_top_distance)
                    ir_top_pid.publish_state(ir_top_distance)

                if IMU:
                    angles = imu.angles
                    y = 0
                    x = 0
                    for angle in angles:
                        y += math.sin(angle)
                        x += math.cos(angle)
                    imu_heading = math.atan2(y, x)
                    rospy.loginfo("IMU Heading:\t%f", imu_heading)
                    imu_pid.publish_state(imu_heading)

                # Iterate at frequency of rate
                rate.sleep()



                if not IMU and not TOP_IR:
                    steering_cmd = ir_bottom_pid.control_effort
                elif not IMU and not BOTTOM_IR:
                    steering_cmd = ir_top_pid.control_effort
                elif not TOP_IR and not BOTTOM_IR:
                    steering_cmd = imu_pid.control_effort
                elif not IMU:
                    steering_cmd = (ir_bottom_pid.control_effort + \
                                   ir_top_pid.control_effort) / 2
                elif not TOP_IR:
                    steering_cmd = (ir_bottom_pid.control_effort + \
                                    imu_pid.control_effort) / 2
                elif not BOTTOM_IR:
                    steering_cmd = (ir_top_pid.control_effort + \
                                    imu_pid.control_effort) / 2
                else:
                    steering_cmd = heuristic(ir_bottom_pid,
                                             ir_top_pid,
                                             imu_pid,
                                             temp_ir_bottom_state,
                                             temp_ir_top_state,
                                             temp_imu_state)

                # Set steering target
                steering_cmd += CENTER
                rospy.loginfo("Position Command:\t%d", steering_cmd)
                steering.set_target(steering_cmd)
                print


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('odroid_node', anonymous=True)

    try:
        odroid()
    except rospy.ROSInterruptException:
        pass
