#!/usr/bin/env python


def test_imu(ir_bottom_pid, ir_top_pid, imu_wall_pid, imu_corner_pid):
    if ir_bottom_pid.recorded_states[-1] < CORNER_THRESHOLD and ir_top_pid.recorded_states[-1] < CORNER_THRESHOLD \
            and ir_bottom_pid.recorded_states[-1] > 0 and ir_top_pid.recorded_states[-1] > 0:
        print "WALL-FOLLOWING"
        return imu_wall_pid.control_effort
    elif imu_corner_pid.setpoint == imu_wall_pid.setpoint:
        setpoint = imu_corner_pid.setpoint.data - math.radians(90)
        if setpoint <= -math.pi:
            setpoint += 2*math.pi
        imu_corner_pid.imu_setpoint(setpoint)
        return imu_corner_pid.control_effort
    else:
        return imu_corner_pid.control_effort


def kodiesStateMachine1(robot,ir_bottom_pid,ir_top_pid,imu_wall_pid,imu_corner_pid):
    ir_top_pid.ignore = True
    if len(imu_corner_pid.reported_states) < 2:
        return 0

    # define setpoint error values for state switching logic
    ir_bottom_error = math.fabs(ir_bottom_pid.setpoint.data - ir_bottom_pid.state.data)

    imu_wall_error = math.fabs(imu_wall_pid.setpoint.data - imu_corner_pid.state.data)
    imu_corner_error = math.fabs(imu_corner_pid.setpoint.data - imu_corner_pid.state.data)

    # finite differencing on state to estimate derivative (divide by timestep?)
    ir_bottom_diff = math.fabs(ir_bottom_pid.state.data - ir_bottom_pid.reported_states[-2])

    imu_wall_diff = math.fabs(imu_wall_pid.state.data - imu_corner_pid.reported_states[-2])
    imu_corner_diff = math.fabs(imu_corner_pid.state.data - imu_corner_pid.reported_states[-2])

    #rospy.loginfo("Bottom IR Error:\t%f", ir_bottom_error)
    #rospy.loginfo("Top IR Error:\t%f", ir_top_error)
    #rospy.loginfo("IMU WALL Error:\t%f", imu_wall_error)
    #rospy.loginfo("IMU CORNER Error:\t%f", imu_corner_error)

    if robot["state"] == 'wall_follow':
        print "WALL-FOLLOW"
        rospy.loginfo("ir_bottom_diff:\t%f", ir_bottom_diff)
       # rospy.loginfo("ir_top_diff:\t%f", ir_top_diff)
        # either top or bottom IR has detected corner
        if ir_bottom_error > CORNER_ERROR_THRESHOLD:
            print "CORNER DETECTED"
            ir_bottom_pid.ignore = True
            imu_wall_pid.ignore = True      # don't know of any reason this should be False at this point
            # enable imu_corner_pid
            imu_corner_pid.ignore = False

            # reset IMU setpoint for cornering task
            imu_setpoint = imu_wall_pid.setpoint.data - math.radians(90)
            imu_wall_pid.imu_setpoint(imu_setpoint)
            imu_corner_pid.imu_setpoint(imu_setpoint)

            robot["state"] = 'corner'
            time_since_turn = time.time()
        # either top or bottom IR has detected doorway
        elif (ir_bottom_diff > DOOR_THRESHOLD and ir_bottom_diff < CORNER_THRESHOLD) and False:
            print "DOORWAY DETECTED"
            # reset IMU setpoint for cornering task
            imu_setpoint = imu_wall_pid.setpoint.data
            imu_wall_pid.imu_setpoint(imu_setpoint)
            # ignore IR sensor that has detected doorway
            if ir_bottom_diff > DOOR_THRESHOLD:
                ir_bottom_pid.ignore = True

            # use imu wall-following PID controller
            imu_wall_pid.ignore = False
            robot["state"] = 'doorway'


    elif robot["state"] == 'doorway':
        print "DOORWAY"
        rospy.loginfo("ir_bottom_diff:\t%f", ir_bottom_diff)
       # rospy.loginfo("ir_top_diff:\t%f", ir_top_diff)
        rospy.loginfo("ir_bottom_error:\t%f", ir_bottom_error)
       # rospy.loginfo("ir_top_error:\t%f", ir_top_error)

        if ir_bottom_error < CORNER_ERROR_THRESHOLD:
            ir_bottom_pid.ignore = False
            robot["state"] = 'wall_follow'

    elif robot["state"] == 'corner':
        print "CORNERING"
        if imu_corner_error < math.pi/18:
            print "REACHED IMU SETPOINT WITHIN IMU_THRESHOLD"

            time_diff = time.time() - time_since_turn

            # both IR errors are less than corner state
            if time_diff > 3 and (ir_bottom_error < CORNER_ERROR_THRESHOLD):
                # turn IR PID control back on
                ir_bottom_pid.ignore = False
                imu_wall_pid.ignore = True      # may not want to use imu_pid to do wall-following
                imu_corner_pid.ignore = True
                robot["state"] = 'wall_follow'

        else:
            # log imu_corner_pid state and setpoint error during turn
            rospy.loginfo("CORNERING:\t{}\t{}".format(math.degrees(imu_corner_pid.state.data), math.degrees(imu_corner_error)))

    else:
        print "Entered default case in state machine."

    # Set steering command as average of steering commands that we want to use
    i = 0
    steering_cmd = 0
    if not ir_top_pid.ignore and TOP_IR:
        i += 1
        steering_cmd += ir_top_pid.control_effort
    if not ir_bottom_pid.ignore and BOTTOM_IR:
        i += 1
        steering_cmd += ir_bottom_pid.control_effort
    if not imu_wall_pid.ignore and IMU_WALL:
        i += 1
        steering_cmd += imu_wall_pid.control_effort
    if not imu_corner_pid.ignore and IMU_CORNER:
        i += 1
        steering_cmd += imu_corner_pid.control_effort
    steering_cmd /= i

    return steering_cmd

def kodiesStateMachine(robot,ir_bottom_pid,ir_top_pid,imu_wall_pid,imu_corner_pid):

    if len(imu_corner_pid.reported_states) < 2:
        return 0

    # define setpoint error values for state switching logic
    ir_bottom_error = math.fabs(ir_bottom_pid.setpoint.data - ir_bottom_pid.state.data)
    ir_top_error = math.fabs(ir_top_pid.setpoint.data - ir_top_pid.state.data)
    imu_wall_error = math.fabs(imu_wall_pid.setpoint.data - imu_corner_pid.state.data)
    imu_corner_error = math.fabs(imu_corner_pid.setpoint.data - imu_corner_pid.state.data)

    # finite differencing on state to estimate derivative (divide by timestep?)
    ir_bottom_diff = math.fabs(ir_bottom_pid.state.data - ir_bottom_pid.reported_states[-2])
    ir_top_diff = math.fabs(ir_top_pid.state.data - ir_top_pid.reported_states[-2])
    imu_wall_diff = math.fabs(imu_wall_pid.state.data - imu_corner_pid.reported_states[-2])
    imu_corner_diff = math.fabs(imu_corner_pid.state.data - imu_corner_pid.reported_states[-2])

    #rospy.loginfo("Bottom IR Error:\t%f", ir_bottom_error)
    #rospy.loginfo("Top IR Error:\t%f", ir_top_error)
    #rospy.loginfo("IMU WALL Error:\t%f", imu_wall_error)
    #rospy.loginfo("IMU CORNER Error:\t%f", imu_corner_error)

    if robot["state"] == 'wall_follow':
        print "WALL-FOLLOW"
        rospy.loginfo("ir_bottom_diff:\t%f", ir_bottom_diff)
        rospy.loginfo("ir_top_diff:\t%f", ir_top_diff)
        # either top or bottom IR has detected corner
        if ir_bottom_error > CORNER_ERROR_THRESHOLD and ir_top_error > CORNER_ERROR_THRESHOLD:
            print "CORNER DETECTED"
            ir_bottom_pid.ignore = True
            ir_top_pid.ignore = True
            imu_wall_pid.ignore = True      # don't know of any reason this should be False at this point

            # enable imu_corner_pid
            imu_corner_pid.ignore = False

            # reset IMU setpoint for cornering task
            imu_setpoint = imu_wall_pid.setpoint.data - math.radians(80)
            imu_wall_pid.imu_setpoint(imu_setpoint)
            imu_corner_pid.imu_setpoint(imu_setpoint)
            robot["state"] = 'corner'
        # either top or bottom IR has detected doorway
    #    elif (ir_bottom_diff > DOOR_THRESHOLD and ir_bottom_diff < CORNER_THRESHOLD) or \
     #       (ir_top_diff > DOOR_THRESHOLD and ir_top_diff < CORNER_THRESHOLD):
        elif False:
            print "DOORWAY DETECTED"
            # reset IMU setpoint for cornering task
            imu_setpoint = imu_wall_pid.setpoint.data
            imu_wall_pid.imu_setpoint(imu_setpoint)
            # ignore IR sensor that has detected doorway
            if ir_bottom_diff > DOOR_THRESHOLD:
                ir_bottom_pid.ignore = True
            if ir_top_diff > DOOR_THRESHOLD:
                ir_top_pid.ignore = True

            # use imu wall-following PID controller
            imu_wall_pid.ignore = False
            robot["state"] = 'doorway'



        else:
            #protect against entering or exiting a corner
            if ir_bottom_error < CORNER_ERROR_THRESHOLD and ir_top_error < CORNER_ERROR_THRESHOLD:
                ir_bottom_pid.ignore = False
                ir_top_pid.ignore = False
            elif ir_bottom_error > CORNER_ERROR_THRESHOLD:
                ir_bottom_pid.ignore = True
            elif ir_top_error > CORNER_ERROR_THRESHOLD:
                ir_top_pid.ignore = True



    elif robot["state"] == 'doorway':
        print "DOORWAY"
        rospy.loginfo("ir_bottom_diff:\t%f", ir_bottom_diff)
        rospy.loginfo("ir_top_diff:\t%f", ir_top_diff)
        rospy.loginfo("ir_bottom_error:\t%f", ir_bottom_error)
        rospy.loginfo("ir_top_error:\t%f", ir_top_error)

        if ir_bottom_error > CORNER_ERROR_THRESHOLD:
            ir_bottom_pid.ignore = True
            robot["state"] = 'wall_follow'
        elif ir_top_error > CORNER_ERROR_THRESHOLD:
            ir_top_pid.ignore = True
            robot["state"] = 'wall_follow'
        else:
            if ir_bottom_error > DOOR_THRESHOLD:
                ir_bottom_pid.ignore = True
            if ir_top_error > DOOR_THRESHOLD:
                ir_top_pid.ignore = True

            # only switch back to wall-following after both sensors have cleared the doorway. This will prevent
            # the 'doorway' state from triggering again once the bottom IR sensor passes the doorway since
            # switching to the 'doorway' state is currently based on the abs value of the error derivative
            if ir_bottom_error < DOOR_THRESHOLD and ir_top_error < DOOR_THRESHOLD:
                ir_bottom_pid.ignore = False
                ir_top_pid.ignore = False
                imu_wall_pid.ignore = True

                robot["state"] = 'wall_follow'

    elif robot["state"] == 'corner':
        print "CORNERING"
        if imu_corner_error < IMU_THRESHOLD:
            print "REACHED IMU SETPOINT WITHIN IMU_THRESHOLD"

            # both IR errors are less than corner state
            if ir_bottom_error < CORNER_ERROR_THRESHOLD and ir_top_error < CORNER_ERROR_THRESHOLD:
                # turn IR PID control back on
                ir_bottom_pid.ignore = False
                ir_top_pid.ignore = False
                imu_wall_pid.ignore = True      # may not want to use imu_pid to do wall-following
                imu_corner_pid.ignore = True

                robot["state"] = 'wall_follow'

        else:
            # log imu_corner_pid state and setpoint error during turn
            rospy.loginfo("CORNERING:\t{}\t{}".format(math.degrees(imu_corner_pid.state.data), math.degrees(imu_corner_error)))

    else:
        print "Entered default case in state machine."

    # Set steering command as average of steering commands that we want to use
    i = 0
    steering_cmd = 0
    if not ir_top_pid.ignore and TOP_IR:
        i += 1
        steering_cmd += ir_top_pid.control_effort
    if not ir_bottom_pid.ignore and BOTTOM_IR:
        i += 1
        steering_cmd += ir_bottom_pid.control_effort
    if not imu_wall_pid.ignore and IMU_WALL:
        i += 1
        steering_cmd += imu_wall_pid.control_effort
    if not imu_corner_pid.ignore and IMU_CORNER:
        i += 1
        steering_cmd += imu_corner_pid.control_effort
    steering_cmd /= i

    return steering_cmd



def stateMachine(robot,ir_bottom_pid,ir_top_pid,imu_wall_pid,imu_corner_pid):

    if len(imu_corner_pid.reported_states) < 2:
        return 0

    # define setpoint error values for state switching logic
    ir_bottom_error = math.fabs(ir_bottom_pid.setpoint.data - ir_bottom_pid.state.data)
    ir_top_error = math.fabs(ir_top_pid.setpoint.data - ir_top_pid.state.data)
    imu_wall_error = math.fabs(imu_wall_pid.setpoint.data - imu_corner_pid.state.data)
    imu_corner_error = math.fabs(imu_corner_pid.setpoint.data - imu_corner_pid.state.data)

    # finite differencing on state to estimate derivative (divide by timestep?)
    ir_bottom_diff = math.fabs(ir_bottom_pid.state.data - ir_bottom_pid.reported_states[-2])
    ir_top_diff = math.fabs(ir_top_pid.state.data - ir_top_pid.reported_states[-2])
    imu_wall_diff = math.fabs(imu_wall_pid.state.data - imu_corner_pid.reported_states[-2])
    imu_corner_diff = math.fabs(imu_corner_pid.state.data - imu_corner_pid.reported_states[-2])

    #rospy.loginfo("Bottom IR Error:\t%f", ir_bottom_error)
    #rospy.loginfo("Top IR Error:\t%f", ir_top_error)
    #rospy.loginfo("IMU WALL Error:\t%f", imu_wall_error)
    #rospy.loginfo("IMU CORNER Error:\t%f", imu_corner_error)

    if robot["state"] == 'wall_follow':
        print "WALL-FOLLOW"
        rospy.loginfo("ir_bottom_diff:\t%f", ir_bottom_diff)
        rospy.loginfo("ir_top_diff:\t%f", ir_top_diff)

        # either top or bottom IR has detected doorway
        if (ir_bottom_diff > DOOR_THRESHOLD and ir_bottom_diff < CORNER_THRESHOLD) or \
            (ir_top_diff > DOOR_THRESHOLD and ir_top_diff < CORNER_THRESHOLD):
            print "DOORWAY DETECTED"

            # ignore IR sensor that has detected doorway
            if ir_bottom_diff > DOOR_THRESHOLD:
                ir_bottom_pid.ignore = True
            if ir_top_diff > DOOR_THRESHOLD:
                ir_top_pid.ignore = True

            # use imu wall-following PID controller
            imu_wall_pid.ignore = False
            robot["state"] = 'doorway'

        # either top or bottom IR has detected corner
        elif ir_bottom_diff > CORNER_THRESHOLD or ir_top_diff > CORNER_THRESHOLD:
            print "CORNER DETECTED"
            ir_bottom_pid.ignore = True
            ir_top_pid.ignore = True
            imu_wall_pid.ignore = True      # don't know of any reason this should be False at this point

            # enable imu_corner_pid
            imu_corner_pid.ignore = False

            # reset IMU setpoint for cornering task
            imu_setpoint = imu_wall_pid.setpoint.data - math.radians(90)
            imu_wall_pid.imu_setpoint(imu_setpoint)
            imu_corner_pid.imu_setpoint(imu_setpoint)
            robot["state"] = 'corner'

        else:
            pass
            # do nothing: continue wall-following

    elif robot["state"] == 'doorway':
        print "DOORWAY"
        rospy.loginfo("ir_bottom_diff:\t%f", ir_bottom_diff)
        rospy.loginfo("ir_top_diff:\t%f", ir_top_diff)
        rospy.loginfo("ir_bottom_error:\t%f", ir_bottom_error)
        rospy.loginfo("ir_top_error:\t%f", ir_top_error)

        if ir_bottom_error > DOOR_THRESHOLD:
            ir_bottom_pid.ignore = True
        if ir_top_error > DOOR_THRESHOLD:
            ir_top_pid.ignore = True

        # only switch back to wall-following after both sensors have cleared the doorway. This will prevent
        # the 'doorway' state from triggering again once the bottom IR sensor passes the doorway since
        # switching to the 'doorway' state is currently based on the abs value of the error derivative
        if ir_bottom_error < DOOR_THRESHOLD and ir_top_error < DOOR_THRESHOLD:
            ir_bottom_pid.ignore = False
            ir_top_pid.ignore = False
            imu_wall_pid.ignore = True

            robot["state"] = 'wall_follow'

    elif robot["state"] == 'corner':
        print "CORNERING"
        if imu_corner_error < IMU_THRESHOLD:
            print "REACHED IMU SETPOINT WITHIN IMU_THRESHOLD"

            # both IR derivatives have stabilized (states not necessarily within DOOR_THRESHOLD)
            if ir_bottom_diff < DOOR_THRESHOLD and ir_top_diff < DOOR_THRESHOLD:
                # turn IR PID control back on
                ir_bottom_pid.ignore = False
                ir_top_pid.ignore = False
                imu_wall_pid.ignore = True      # may not want to use imu_pid to do wall-following
                imu_corner_pid.ignore = True

                robot["state"] = 'wall_follow'

        else:
            # log imu_corner_pid state and setpoint error during turn
            rospy.loginfo("CORNERING:\t{}\t{}".format(math.degrees(imu_pid.state.data), math.degrees(imu_error)))

    else:
        print "Entered default case in state machine."

    # Set steering command as average of steering commands that we want to use
    i = 0
    steering_cmd = 0
    if not ir_top_pid.ignore and TOP_IR:
        i += 1
        steering_cmd += ir_top_pid.control_effort
    if not ir_bottom_pid.ignore and BOTTOM_IR:
        i += 1
        steering_cmd += ir_bottom_pid.control_effort
    if not imu_wall_pid.ignore and IMU_WALL:
        i += 1
        steering_cmd += imu_wall_pid.control_effort
    if not imu_corner_pid.ignore and IMU_CORNER:
        i += 1
        steering_cmd += imu_corner_pid.control_effort
    steering_cmd /= i

    return steering_cmd

def heuristic4(ir_bottom_pid,ir_top_pid,imu_pid,ir_bottom_state,ir_top_state,imu_state):

    # General Questions:
    # 1. Is this heuristic better written as a state-machine rather than an
    #    if-elif-else structure? - Carl

    # define setpoint error values for state switching logic
    ir_bottom_error = math.fabs(ir_bottom_pid.setpoint - ir_bottom_pid.state.data)
    ir_top_error = math.fabs(ir_top_pid.setpoint - ir_top_pid.state.data)
    imu_error = math.fabs(imu_pid.setpoint - imu_pid.state.data)

    # finite differencing on state to estimate derivative (divide by timestep?)
    ir_bottom_diff = math.fabs(ir_bottom_pid.state.data - ir_bottom_state)
    ir_top_diff = math.fabs(ir_top_pid.state.data - ir_top_state)
    imu_diff = math.fabs(imu_pid.state.data - imu_state)

    rospy.loginfo("Bottom IR Error:\t%f", ir_bottom_error)
    rospy.loginfo("Top IR Error:\t%f", ir_top_error)
    rospy.loginfo("IMU Error:\t%f", imu_error)

    # STATE: Wall-Following - If tracking wall distance setpoint
    if (ir_bottom_diff < DOOR_THRESHOLD and ir_top_diff < DOOR_THRESHOLD) and not imu_pid.turning:
        print"WALL FOLLOWING"
        ir_top_pid.ignore = False
        ir_bottom_pid.ignore = False
        # imu_pid.turning = False

    # SATE: Doorway Crossing - If crossing doorway
    elif (ir_bottom_diff < CORNER_THRESHOLD and ir_top_diff < CORNER_THRESHOLD) and not imu_pid.turning:
        print "PASSING DOORWAY"

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
            ir_bottom_pid.ignore = True
            imu_pid.imu_setpoint(IMU_CONNECTED, DUMMY_IMU_VALUE, imu_pid.setpoint - math.radians(90))

            # TODO: remap IMU PID gains to better suit cornering task (aka "Gain Scheduling")
            # Need 2 sets of IMU PID gains: wall-following & cornering

        # should execute when IMU turn is completed
        elif imu_error < IMU_THRESHOLD and ir_bottom_diff < DOOR_THRESHOLD and ir_top_diff < DOOR_THRESHOLD:
            print "EXITING CORNER"
            imu_pid.turning = False

            # TODO: set IMU PID gains back to wall-following gains

        # executes during IMU turn
        else:
            rospy.loginfo("CORNERING:\t{}\t{}".format(math.degrees(imu_pid.state.data), math.degrees(imu_error)))


    # Set steering command as average of steering commands that we want to use
    i = 0
    steering_cmd = 0
    if not ir_top_pid.ignore and TOP_IR:
        i += 1
        steering_cmd += ir_top_pid.control_effort
    if not ir_bottom_pid.ignore and BOTTOM_IR:
        i += 1
        steering_cmd += ir_bottom_pid.control_effort
    if not imu_wall_pid.ignore and IMU_WALL_PID:
        i += 1
        steering_cmd += imu_pid.control_effort
    if not imu_corner_pid.ignore and IMU_CORNER_PID:
        i += 1
        steering_cmd += imu_pid.control_effort
    steering_cmd /= i

    return steering_cmd


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

    rospy.loginfo("Bottom IR Error:\t%f", ir_bottom_error)
    rospy.loginfo("Top IR Error:\t%f", ir_top_error)
    rospy.loginfo("IMU Error:\t%f", imu_error)

    # STATE: Wall-Following - If tracking wall distance setpoint
    if (ir_bottom_error < DOOR_THRESHOLD and ir_top_error < DOOR_THRESHOLD) and not imu_pid.turning:
        ir_top_pid.ignore = False
        ir_bottom_pid.ignore = False
        # imu_pid.turning = False

    # SATE: Doorway Crossing - If crossing doorway
    elif (ir_bottom_error < CORNER_THRESHOLD and ir_top_error < CORNER_THRESHOLD) and not imu_pid.turning:
        print "PASSING DOORWAY"

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
            ir_bottom_pid.ignore = True
            imu_pid.imu_setpoint(IMU_CONNECTED, DUMMY_IMU_VALUE, imu_pid.setpoint - math.radians(90))

            # TODO: remap IMU PID gains to better suit cornering task (aka "Gain Scheduling")
            # Need 2 sets of IMU PID gains: wall-following & cornering

        # should execute when IMU turn is completed
        elif imu_error < IMU_THRESHOLD:
            print "EXITING CORNER"

            if ir_bottom_error > DOOR_THRESHOLD or ir_top_error > DOOR_THRESHOLD:
                print "EXITING CORNER: ir_error > DOOR_THRESHOLD"
                # Do nothing - remain in IMU turn

                # Ideally, this is not what I'd like to do. Once imu_error < IMU_THRESHOLD, I want to
                # be able to swtich back over to incorproating the distance tracking PID controller,
                # and use that controller to drive out distance error back to zero after the turn has
                # completed and we've returned to wall following.

                # This will require that I track the derivative of the IR distance errors as opposed to
                # just the current values of ir_bottom_error and ir_top_error.

            else:
                print "EXITING CORNER: ir_error < DOOR_THRESHOLD"
                imu_pid.turning = False

            # TODO: set IMU PID gains back to wall-following gains

        # executes during IMU turn
        else:
            print "CORNERING:\t",math.degrees(imu_pid.state.data),"\t",math.degrees(imu_error)


    # Set steering command as average of steering commands that we want to use
    i = 0
    steering_cmd = 0
    if not ir_top_pid.ignore and TOP_IR:
        i += 1
        steering_cmd += ir_top_pid.control_effort
    if not ir_bottom_pid.ignore and BOTTOM_IR:
        i += 1
        steering_cmd += ir_bottom_pid.control_effort
    if not imu_wall_pid.ignore and IMU_WALL_PID:
        i += 1
        steering_cmd += imu_pid.control_effort
    if not imu_corner_pid.ignore and IMU_PID:
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

    # Set steering command as average of steering commands that we want to use
    i = 0
    steering_cmd = 0
    if not ir_top_pid.ignore and TOP_IR:
        i += 1
        steering_cmd += ir_top_pid.control_effort
    if not ir_bottom_pid.ignore and BOTTOM_IR:
        i += 1
        steering_cmd += ir_bottom_pid.control_effort
    if not imu_wall_pid.ignore and IMU_WALL_PID:
        i += 1
        steering_cmd += imu_pid.control_effort
    if not imu_corner_pid.ignore and IMU_PID:
        i += 1
        steering_cmd += imu_pid.control_effort
    steering_cmd /= i

    return steering_cmd


# Callback from kalman filter subscriber
def kalman_filter_callback(data, args):
    imu_wall_pid = args[0]
    imu_corner_pid = args[1]
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    angles = euler_from_quaternion([x, y, z, w])
    if len(imu_wall_pid.recorded_states) >= NUM_READINGS:
        del imu_wall_pid.recorded_states[0]
    imu_wall_pid.recorded_states.append(angles[2])
    if len(imu_corner_pid.recorded_states) >= NUM_READINGS:
        del imu_corner_pid.recorded_states[0]
    imu_corner_pid.recorded_states.append(angles[2])


# Main method
def odroid():

    # Initialize Pololu Controllers and PID drivers
    with pololu.Controller(0) as steering,  \
         pololu.Controller(1) as motor,     \
         pololu.Controller(2) as ir_bottom, \
         pololu.Controller(3) as ir_top,    \
         pid_driver.Driver("IMU_CORNER",
                           None,
                           IMU_CORNER,
                           NUM_STATES_STORED) as imu_corner_pid,          \
         pid_driver.Driver("IMU_WALL",
                           None,
                           IMU_WALL,
                           NUM_STATES_STORED) as imu_wall_pid,            \
         pid_driver.Driver("bottom_IR",
                           ir_bottom,
                           BOTTOM_IR,
                           NUM_STATES_STORE) as ir_bottom_pid,              \
         pid_driver.Driver("top_IR",
                           ir_top,
                           TOP_IR,
                           NUM_STATES_STORED) as ir_top_pid:

        # Initialize subscriber for IMU
        kf_sub = rospy.Subscriber("odometry/filtered",
                                   Odometry,
                                   kalman_filter_callback,
                                   [imu_wall_pid,imu_corner_pid])

        # Calibrate IMU gyro biases
        if IMU_CONNECTED:
            rospy.wait_for_service('imu/calibrate')
            try:
                rospy.ServiceProxy('imu/calibrate', Empty)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        # Send setpoints to PIDs
        # Wait for recorded IMU data before publishing setpoint
        rospy.sleep(0.25)
        imu_corner_pid.imu_setpoint()
        imu_wall_pid.imu_setpoint(imu_corner_pid.setpoint.data)
        ir_bottom_pid.ir_setpoint(POLOLU_CONNECTED, NUM_READINGS, DUMMY_IR_VALUE)
        ir_top_pid.ir_setpoint(POLOLU_CONNECTED, NUM_READINGS, DUMMY_IR_VALUE)

        # Set zero intial velocity and steering
        motor.set_target(CENTER)
        steering.set_target(CENTER)
        rospy.sleep(1)

        # Set forward speed
        motor.set_target(MOTOR_SPEED)

        # Iteration rate
        rate = rospy.Rate(RATE)

        # Initialize stateMachine()
        robot = {"state": "wall_follow"}
        imu_wall_pid.ignore = True
        imu_corner_pid.ignore = True

        # Count iterations
        # Can be used for debugging or any miscellaneous needs
        count = 0
        while not rospy.is_shutdown():

            if POLOLU_CONNECTED:
                # Get measurement reading from bottom IR sensor and publish state
                ir_bottom_pid.recorded_states = []
                for i in range(NUM_READINGS):
                    ir_bottom_pid.recorded_states.append(ir_bottom.get_position())
                state = sum(ir_bottom_pid.recorded_states)                   \
                                     / float(len(ir_bottom_pid.recorded_states))
            else:
                state = DUMMY_IR_VALUE
            rospy.loginfo("Bottom IR Distance:\t%f", state)
            ir_bottom_pid.publish_state(state)

            if POLOLU_CONNECTED:
                # Get measurement reading from top IR sensor and publish state
                ir_top_pid.recorded_states = []
                for i in range(NUM_READINGS):
                    ir_top_pid.recorded_states.append(ir_top.get_position())
                state = sum(ir_top_pid.recorded_states)                   \
                                     / float(len(ir_top_pid.recorded_states))
            else:
                state = DUMMY_IR_VALUE
            rospy.loginfo("Top IR Distance:\t%f", state)
            ir_top_pid.publish_state(state)

            # Block for shifting IMU setpoint right 90 degrees
            #if count == 150:
                #setpoint = imu_pid.setpoint - math.radians(90)
                #imu_pid.imu_setpoint(IMU_CONNECTED, DUMMY_IMU_VALUE, setpoint)

            # Get measurement reading from top IR sensor and publish state
            y = 0
            x = 0
            for state in imu_corner_pid.recorded_states:
                y += math.sin(state)
                x += math.cos(state)
            imu_heading = math.atan2(y, x)
            if imu_heading < imu_corner_pid.setpoint.data - math.pi:
                imu_heading += 2*math.pi
            elif imu_heading > imu_corner_pid.setpoint.data + math.pi:
                imu_heading -= 2*math.pi
            rospy.loginfo("IMU Heading:\t%f", math.degrees(imu_heading))
            imu_wall_pid.publish_state(imu_heading)
            imu_corner_pid.publish_state(imu_heading)
            rospy.loginfo("IR top setpoint:\t%f", ir_top_pid.setpoint.data)
            rospy.loginfo("IR bottom setpoint:\t%f", ir_bottom_pid.setpoint.data)
            rospy.loginfo("IMU wall setpoint:\t%f", math.degrees(imu_wall_pid.setpoint.data))
            rospy.loginfo("IMU corner setpoint:\t%f", math.degrees(imu_corner_pid.setpoint.data))

            # Iterate at frequency of rate
            rate.sleep()

            # Heuristics
            # steering_cmd = test_imu(ir_bottom_pid, ir_top_pid, imu_wall_pid, imu_corner_pid)
            # steering_cmd = heuristic3(ir_bottom_pid,ir_top_pid,imu_pid)
            # steering_cmd = heuristic4(ir_bottom_pid,ir_top_pid,imu_pid, \
            #                           ir_bottom_state,ir_top_state,imu_state)
            # steering_cmd = stateMachine(robot, ir_bottom_pid, ir_top_pid, imu_wall_pid, imu_corner_pid)
            steering_cmd = kodiesStateMachine1(robot, ir_bottom_pid, ir_top_pid, imu_wall_pid, imu_corner_pid)

            # Set steering target
            steering_cmd += CENTER
            rospy.loginfo("Steering Command:\t%d", steering_cmd)
            steering.set_target(steering_cmd)
            print

            # Kill command
            #if math.fabs(ir_top_pid.recorded_states[-1]) <= 65:
            #    break

            count += 1


if __name__ == '__main__':
    import rospy

    # Initialize node
    rospy.init_node('odroid_node', anonymous=True)

    import time
    time_since_turn = time.time()

    from nav_msgs.msg import Odometry
    from std_srvs.srv import Empty
    from tf.transformations import euler_from_quaternion
    import math

    POLOLU_CONNECTED = rospy.get_param('~pololu_connected')
    IMU_CONNECTED = rospy.get_param('~imu_connected')
    DUMMY_IR_VALUE = rospy.get_param('~dummy_ir_value')
    BOTTOM_IR = rospy.get_param('~bottom_ir')
    TOP_IR = rospy.get_param('~top_ir')
    IMU_WALL = rospy.get_param('~imu_wall')
    IMU_CORNER = rospy.get_param('~imu_corner')
    MIN = rospy.get_param('~min')
    MAX = rospy.get_param('~max')
    CENTER = rospy.get_param('~center')
    MOTOR_SPEED = rospy.get_param('~motor_speed')
    RATE = rospy.get_param('~rate')
    NUM_READINGS = rospy.get_param('~num_readings')
    TOP_IR_ANGLE = rospy.get_param('~top_ir_angle')
    IMU_THRESHOLD = rospy.get_param('~imu_threshold')
    DOOR_THRESHOLD = rospy.get_param('~door_threshold')
    CORNER_THRESHOLD = rospy.get_param('~corner_threshold')
    DOOR_ERROR_THRESHOLD = rospy.get_param('~door_error_threshold')
    CORNER_ERROR_THRESHOLD = rospy.get_param('~corner_error_threshold')
    IMU_RESET_THRESHOLD = rospy.get_param('~imu_reset_threshold')
    NUM_STATES_STORED = rospy.get_param('~num_states_stored')

    if POLOLU_CONNECTED:
        from drivers import pololu
    else:
        from drivers import dummy_pololu as pololu
    from drivers import pid_driver


    try:
        odroid()
    except rospy.ROSInterruptException:
        pass
