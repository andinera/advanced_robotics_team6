#!/usr/bin/env python


################################################################################
#                    HEURISTIC 3
################################################################################

def heuristic3(ir_bottom_pid, ir_top_pid, imu_pid):

    # General Questions:
    # 1. Is this heuristic better written as a state-machine rather than an
    #    if-elif-else structure? - Carl

    # Suspected Failure Modes:
    # 1. If ir_bottom_error OR ir_top_error is greater than DOOR_THRESHOLD at
    #    the end of the IMU turn, this state-machine will not return to the
    #    wall-following state, but instead will think that it has reached
    #    another doorway. I think that this failure mode can be eliminted by
    #    proper tuning of the IMU Cornering PID gains. - Carl

    # use setpoint error values for state switching logic
    ir_bottom_error = math.fabs(ir_bottom_pid.setpoint - ir_bottom_pid.state.data)
    ir_top_error = math.fabs(ir_top_pid.setpoint - ir_top_pid.state.data)
    imu_error = math.fabs(imu_pid.setpoint - imu_pid.state.data)

    # STATE: Wall-Following - If tracking wall distance setpoint
    if (ir_bottom_error < DOOR_THRESHOLD and ir_top_error < DOOR_THRESHOLD) and not imu_pid.turning:
        ir_top_pid.ignore = False
        ir_bottom_pid.ignore = False
        # imu_pid.turning = False

    # SATE: Doorway Crossing - If crossing doorway
    elif (ir_bottom_error < CORNER_THRESHOLD and ir_top_error < CORNER_THRESHOLD) and not imu_pid.turning:
        # Top IR sensor detects doorway, ignore top IR sensor
        if ir_top_error > DOOR_THRESHOLD:
            ir_top_pid.ignore = True
        # Top IR sensor is past doorway
        else:
            ir_top_pid.ignore = False
        # Bottom IR sensor detects doorway, ignore bottom IR sensor
        if ir_bottom_error > DOOR_THRESHOLD:
            ir_bottom_pid.ignore = True
        else:
            ir_bottom_pid.ignore = False

    # STATE: Cornering - If cornering, ignore IR sensors and start IMU turn
    else:
        # should execute only once at start of IMU turn
        if not imu_pid.turning:
            print "ENTERING CORNER"
            imu_pid.turning = True
            ir_top_pid.ignore = True
            ir_bottom.ignore = True
            imu_pid.imu_setpoint(IMU_CONNECTED, DUMMY_VALUE, imu_pid.setpoint + math.radians(90))

            # TODO: remap IMU PID gains to better suit cornering task (aka "Gain Scheduling")
            # Need 2 sets of IMU PID gains: wall-following & cornering

        # should execute when IMU turn is completed
        elif imu_error < IMU_THRESHOLD:
            print "EXITING CORNER"
            imu_pid.turning = False

            # TODO: set IMU PID gains back to wall-following gains

        # executes during IMU turn
        else:
            print "CORNERING:\t",math.degrees(imu_pid.state.data),"\t",math.degrees(imu_error)


    # Set steering command as average of steering commands that we want to use
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

################################################################################
#                    HEURISTIC 2
###############################################################################

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

################################################################################
#                    HEURISTIC 1
###############################################################################

def heuristic1(ir_bottom_pid, ir_top_pid, imu_pid):
    # For each sensor,
    # If 4 or more states are recorded
    # Then read second largest and second smallest
    # Then calculate the difference
    # If 3 or fewer states are recorded
    # Then read the largest and smallest
    # Then calculate the difference
    # The goal with the second largest and second smallest is to
    # eliminate possible outrageous readings
    if len(ir_bottom_pid.reported_states) >= 4:
        max_bottom_ir = ir_bottom_pid.reported_states.index(max(ir_bottom_pid.reported_states))
        ir_bottom_pid.reported_states.remove(max_bottom_ir)
        min_bottom_ir = ir_bottom_pid.reported_states.index(max(ir_bottom_pid.reported_states))
        ir_bottom_pid.reported_states.remove(min_bottom_ir)
    max_bottom_ir = max(ir_bottom_pid.reported_states)
    min_bottom_ir = min(ir_bottom_pid.reported_states)
    diff_bottom_ir = max_bottom_ir - min_bottom_ir
    if len(ir_top_pid.reported_states) >= 4:
        max_top_ir = ir_top_pid.reported_states.index(max(ir_top_pid.reported_states))
        ir_top_pid.reported_states.remove(max_top_ir)
        min_top_ir = ir_top_pid.reported_states.index(max(ir_top_pid.reported_states))
        ir_top_pid.reported_states.remove(min_top_ir)
    max_top_ir = max(ir_top_pid.reported_states)
    min_top_ir = min(ir_top_pid.reported_states)
    diff_top_ir = max_top_ir - min_top_ir
    if len(imu_pid.reported_states) >= 4:
        max_imu = imu_pid.reported_states.index(max(imu_pid.reported_states))
        imu_pid.reported_states.remove(max_imu)
        min_imu = imu_pid.reported_states.index(max(imu_pid.reported_states))
        imu_pid.reported_states.remove(min_imu)
    max_imu = max(imu_pid.reported_states)
    min_imu = min(imu_pid.reported_states)
    diff_imu = max_imu - min_imu

    # If following wall
    if diff_bottom_ir < DOOR_THRESHOLD and diff_top_ir < DOOR_THRESHOLD:
        ir_bottom_pid.ignore = False
        ir_top_pid.ignore = False
        # If the car is driving parallel to the wall within the imu reset threshold
        if diff_bottom_ir < IMU_RESET_THRESHOLD and diff_top_ir < IMU_RESET_THRESHOLD:
            states = imu_pid.recorded_states
            y = 0
            x = 0
            for state in states:
                y += math.sin(state)
                x += math.cos(state)
            imu_heading = math.atan2(y, x)
            rospy.wait_for_service('imu/calibrate')
            try:
                rospy.ServiceProxy('imu/calibrate', Empty)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            imu_pid.imu_setpoint(IMU_CONNECTED, DUMMY_IMU_VALUE, imu_heading)
    # If passing doorway
    elif diff_bottom_ir < CORNER_THRESHOLD and diff_top_ir < CORNER_THRESHOLD:
        if diff_bottom_ir >= DOOR_THRESHOLD:
            ir_bottom_pid.ignore = True
        else:
            ir_bottom_pid.ignore = False
        if diff_top_ir >= DOOR_THRESHOLD:
            ir_top_pid.ignore = True
        else:
            ir_top_pid.ignore = False
    # If entering corner
    else:
        if diff_bottom_ir < DOOR_THRESHOLD:
            ir_bottom_pid.ignore = False
        else:
            ir_bottom_pid.ignore = True
        if diff_top_ir < DOOR_THRESHOLD:
            ir_top_pid.ignore = False
        else:
            ir_top_pid.ignore = True
        # If fully in the corner
        # Then set imu setpoint 90 degrees clockwise
        if diff_bottom_ir.ignore == True and diff_top_ir.ignore == True:
            setpoint = imu_pid.setpoint - math.radians(90)
            if setpoint <= -math.pi:
                setpoint += 2*math.pi
            imu_pid.imu_setpoint(IMU_CONNECTED,
                                 DUMMY_IMU_VALUE,
                                 imu_pid.setpoint - math.radians(90))
    if ir_bottom_pid.ignore == True and ir_top_pid.ignore == True:
        imu_pid.ignore = False
    else:
        imu_pid.ignore = True

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
        states = imu.recorded_states
        y = 0
        x = 0
        for state in states:
            y += math.sin(state)
            x += math.cos(state)
        heading = math.atan2(y, x)
        offset = imu.setpoint - heading
    else:
        offset = 0
    return hypotenuse * math.cos(IR_STATE - offset)

# Callback from kalman filter subscriber
def kalman_filter_callback(data, imu_pid):
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    angles = euler_from_quaternion([x, y, z, w])
    if len(imu_pid.recorded_states) >= NUM_READINGS:
        del imu_pid.recorded_states[0]
    imu_pid.recorded_states.append(angles[2])


# Main method
def odroid():

    # Initialize Pololu Controllers
    with pololu.Controller(0) as steering,  \
         pololu.Controller(1) as motor,     \
         pololu.Controller(2) as ir_bottom, \
         pololu.Controller(3) as ir_top,    \
         phidget.Controller() as imu:

         # Initialize PID drivers
         with pid_driver.Driver("bottom_IR", ir_bottom, BOTTOM_IR, imu, STATES_STORED) as ir_bottom_pid,    \
              pid_driver.Driver("top_IR", ir_top, TOP_IR, imu, STATES_STORED) as ir_top_pid,             \
              pid_driver.Driver("IMU", imu, IMU, STATES_STORED) as imu_pid:

            # Initialize subscriber for IMU
            kf_sub = rospy.Subscriber("odometry/filtered",
                                       Odometry,
                                       kalman_filter_callback,
                                       imu_pid)

            # Send setpoints to PIDs
            if IMU:
                rospy.sleep(0.25)
                imu_pid.imu_setpoint(IMU_CONNECTED, DUMMY_IMU_VALUE)
            if BOTTOM_IR:
                ir_bottom_pid.ir_setpoint(POLOLU_CONNECTED, NUM_READINGS, DUMMY_IR_VALUE, IR_STATE)
            if TOP_IR:
                ir_top_pid.ir_setpoint(POLOLU_CONNECTED, NUM_READINGS, DUMMY_IR_VALUE, IR_STATE)

            # Set zero intial velocity and steering
            motor.set_target(CENTER)
            steering.set_target(CENTER)

            # Set forward speed
            motor.set_target(MOTOR_SPEED)

            # Iteration rate
            rate = rospy.Rate(RATE)

            turn_counter = 0
            while not rospy.is_shutdown():

                # Get measurement reading from sensor(s) and publish state
                if BOTTOM_IR:
                    ir_bottom_pid.recorded_states = []
                    for i in range(NUM_READINGS):
                        ir_bottom_pid.recorded_states.append(ir_bottom.get_position())
                    state = sum(ir_bottom_pid.recorded_states)                   \
                                         / float(len(ir_bottom_pid.recorded_states))
                    rospy.loginfo("Bottom IR Distance:\t%f", state)
                    ir_bottom_pid.publish_state(state)

                if TOP_IR:
                    ir_top_pid.recorded_states = []
                    for i in range(NUM_READINGS):
                        ir_top_pid.recorded_states.append(ir_top.get_position())
                    state = ir_top_conversion(sum(ir_top_pid.recorded_states), imu_pid)      \
                                      / float(len(ir_top_pid.recorded_states))
                    rospy.loginfo("Top IR Distance:\t%f", state)
                    ir_top_pid.publish_state(state)

                if IMU:
                    # if turn_counter == 150:
                    #     setpoint = imu_pid.setpoint - math.radians(90)
                    #     imu_pid.imu_setpoint(IMU_CONNECTED, DUMMY_IMU_VALUE, setpoint)
                    # turn_counter += 1
                    states = imu_pid.recorded_states
                    y = 0
                    x = 0
                    for state in states:
                        y += math.sin(state)
                        x += math.cos(state)
                    imu_heading = math.atan2(y, x)
                    if imu_heading < imu_pid.setpoint - math.pi:
                        imu_heading += 2*math.pi
                    elif imu_heading > imu_pid.setpoint + math.pi:
                        imu_heading -= 2*math.pi
                    rospy.loginfo("IMU Heading:\t%f", math.degrees(imu_heading))
                    imu_pid.publish_state(imu_heading)
                    rospy.loginfo("IMU setpoint:\t%f", math.degrees(imu_pid.setpoint))

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
                    steering_cmd = heuristic2(ir_bottom_pid,
                                             ir_top_pid,
                                             imu_pid)


                # Set steering target
                steering_cmd += CENTER
                rospy.loginfo("Steering Command:\t%d", steering_cmd)
                steering.set_target(steering_cmd)
                print


if __name__ == '__main__':
    import rospy

    # Initialize node
    rospy.init_node('odroid_node', anonymous=True)

    from nav_msgs.msg import Odometry
    from std_srvs.srv import Empty
    from tf.transformations import euler_from_quaternion
    import math

    POLOLU_CONNECTED = rospy.get_param('~pololu_connected')
    IMU_CONNECTED = rospy.get_param('~imu_connected')
    DUMMY_IR_VALUE = rospy.get_param('~dummy_ir_value')
    DUMMY_IMU_VALUE = rospy.get_param('~dummy_imu_value')
    BOTTOM_IR = rospy.get_param('~bottom_ir')
    TOP_IR = rospy.get_param('~top_ir')
    IMU = rospy.get_param('~imu')
    MIN = rospy.get_param('~min')
    MAX = rospy.get_param('~max')
    CENTER = rospy.get_param('~center')
    MOTOR_SPEED = rospy.get_param('~motor_speed')
    RATE = rospy.get_param('~rate')
    NUM_READINGS = rospy.get_param('~num_readings')
    IR_STATE = rospy.get_param('~ir_state')
    IMU_THRESHOLD = rospy.get_param('~imu_threshold')
    DOOR_THRESHOLD = rospy.get_param('~door_threshold')
    CORNER_THRESHOLD = rospy.get_param('~corner_threshold')
    IMU_RESET_THRESHOLD = rospy.get_param('~imu_reset_threshold')
    STATES_STORED = rospy.get_param('~states_stored')

    if POLOLU_CONNECTED:
        from drivers import pololu
    else:
        from drivers import dummy_pololu as pololu

    from drivers import phidget
    from drivers import pid_driver


    try:
        odroid()
    except rospy.ROSInterruptException:
        pass
