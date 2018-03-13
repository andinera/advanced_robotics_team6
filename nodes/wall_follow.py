#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

POLOLU_CONNECTED = True      # True if Pololu is connected
IMU_CONNECTED = True        # True if IMU is connected
DUMMY_IR_VALUE = 100        # Dummy IR sensor value if pololu is not connected
DUMMY_IMU_VALUE = 0         # Dummy IMU value if IMU is not connected

BOTTOM_IR = False
TOP_IR = False
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
IR_ANGLE = math.radians(39)     # Angle of top IR sensor counter-clockwise from x-axis
IR_THRESHOLD = 10
IMU_THRESHOLD = 0.1

DOOR_THRESHOLD = 50
CORNER_THRESHOLD = 500
STATES_STORED = 1

def heuristic3(ir_bottom_pid, ir_top_pid, imu_pid):
    bottom_IR_error = math.fabs(ir_bottom_pid.setpoint - ir_bottom_pid.state.data)
    top_IR_error = math.fabs(ir_top_pid.setpoint - ir_top_pid.state.data)
    imu_error = math.fabs(imu_pid.setpoint - imu_pid.state.data)
    
    # If steadily following the setpoints
    if (bottom_IR_error < DOOR_THRESHOLD and top_IR_error < DOOR_THRESHOLD) and not imu_pid.turning:
        ir_top_pid.ignore = False
        ir_bottom_pid.ignore = False
        imu_pid.turning = False
    
    # If crossing doorway
    elif (bottom_IR_error < CORNER_THRESHOLD and top_IR_error < CORNER_THRESHOLD) and not imu_pid.turning:
        # Top IR sensor detects doorway, ignore top IR sensor
        if top_IR_error > DOOR_THRESHOLD:
            ir_top_pid.ignore = True
        # Top IR sensor is past doorway
        else:
            ir_top_pid.ignore = False
        # Bottom IR sensor detects doorway, ignore bottom IR sensor
        if bottom_IR_error > DOOR_THRESHOLD:
            ir_bottom_pid.ignore = True
        else:
            ir_bottom_pid.ignore = False
    
    # If cornering, ignore IR sensors, start IMU turn
    else:
        # should run only at start of turn
        if not imu_pid.turning:
            print "ENTERING CORNER"
            imu_pid.turning = True
            ir_top_pid.ignore = True
            ir_bottom.ignore = True
            imu_pid.imu_setpoint(IMU_CONNECTED, DUMMY_VALUE, imu_pid.setpoint + math.radians(90))

        # should run during IMU turn
        elif imu_error < IMU_THRESHOLD:
            print "EXITING CORNER"
            imu_pid.turning = False
        else:
            print "CORNERING"

   
    # Set steering command
    i = 0
    steering_cmd = 0
    if not ir_top_pid.ignore:
        i += 1
        steering_cmd += ir_top_pid.control_effort
    if not ir_bottom_pid.ignore:
        i += 1
        steering_cmd += ir_bottom_pid.control_effort
    if not imu_pid.ignore:
        i += 1
        steering_cmd += imu_pid.control_effort
    steering_cmd /= i
    
    return steering_cmd


def heuristic2(ir_bottom_pid, ir_top_pid, imu_pid):
    bottom_IR_error = math.fabs(ir_bottom_pid.setpoint - ir_bottom_pid.state.data)
    top_IR_error = math.fabs(ir_top_pid.setpoint - ir_top_pid.state.data)
    imu_error = math.fabs(imu_pid.setpoint - imu_pid.state.data)
    # If steadily following the setpoints
    if bottom_IR_error < DOOR_THRESHOLD and top_IR_error < DOOR_THRESHOLD:
        ir_top_pid.ignore = False
        ir_bottom_pid.ignore = False
        imu_pid.turning = False
    # If crossing doorway
    elif bottom_IR_error < CORNER_THRESHOLD and top_IR_error < CORNER_THRESHOLD:
        # Top IR sensor detects doorway, ignore top IR sensor
        if top_IR_error > DOOR_THRESHOLD:
            ir_top_pid.ignore = True
        # Top IR sensor is past doorway
        else:
            ir_top_pid.ignore = False
        # Bottom IR sensor detects doorway, ignore bottom IR sensor
        if bottom_IR_error > DOOR_THRESHOLD:
            ir_bottom_pid.ignore = True
        else:
            ir_bottom_pid.ignore = False
    # If cornering, ignore IR sensors, start IMU turn
    else:
        # If starting a turn
        if not imu_pid.turning and bottom_IR_error > CORNER_THRESHOLD:
            ir_top_pid.ignore = True
            ir_bottom_pid.ignore = True
            imu_pid.imu_setpoint(IMU_CONNECTED, DUMMY_IMU_VALUE, imu_pid.setpoint + math.radians(90))
            imu_pid.turning = True
   
    # Set steering command
    i = 0
    steering_cmd = 0
    if not ir_top_pid.ignore:
        i += 1
        steering_cmd += ir_top_pid.control_effort
    if not ir_bottom_pid.ignore:
        i += 1
        steering_cmd += ir_bottom_pid.control_effort
    if not imu_pid.ignore:
        i += 1
        steering_cmd += imu_pid.control_effort
    steering_cmd /= i
    
    return steering_cmd

# Estimate for distance of car to wall based on measurement from top IR sensor
# and IMU heading
def ir_top_conversion(hypotenuse, imu):
    if IMU:
        angles = imu.controller.angles
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
def kalman_filter_callback(data, imu_pid):
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    angles = euler_from_quaternion([x, y, z, w])
    if len(imu_pid.angles) < NUM_READINGS:
        imu_pid.angles.append(angles[2])
    else:
        del imu_pid.angles[0]
        imu_pid.angles.append(angles[2])

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
                                       imu_pid)
            rospy.sleep(1)

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
                    ir_top_distance = ir_top_conversion(sum(ir_top_distance), imu_pid)      \
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
                    steering_cmd = heuristic3(ir_bottom_pid,
                                             ir_top_pid,
                                             imu_pid)
                                     

                # Set steering target
                steering_cmd += CENTER
                rospy.loginfo("Steering Command:\t%d", steering_cmd)
                steering.set_target(steering_cmd)
                print


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('odroid_node', anonymous=True)

    try:
        odroid()
    except rospy.ROSInterruptException:
        pass
