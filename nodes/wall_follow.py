#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from advanced_robotics_team6.srv import PololuCmd
from tf.transformations import euler_from_quaternion
import math
from drivers import pid_driver


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
    if not ir_top_pid.ignore:
        i += 1
        steering_cmd += ir_top_pid.control_effort
    if not ir_bottom_pid.ignore:
        i += 1
        steering_cmd += ir_bottom_pid.control_effort
    if not imu_wall_pid.ignore:
        i += 1
        steering_cmd += imu_wall_pid.control_effort
    if not imu_corner_pid.ignore:
        i += 1
        steering_cmd += imu_corner_pid.control_effort
    steering_cmd /= i

    return steering_cmd

def kodiesStateMachine(robot,ir_bottom_pid,ir_top_pid,imu_wall_pid,imu_corner_pid,time_since_turn):

    if len(imu_corner_pid.reported_states) < 9:
        return 0

    # define setpoint error values for state switching logic
    ir_bottom_error = math.fabs(ir_bottom_pid.setpoint.data - ir_bottom_pid.state.data)
    ir_top_error = math.fabs(ir_top_pid.setpoint.data - ir_top_pid.state.data)
    imu_wall_error = math.fabs(imu_wall_pid.setpoint.data - imu_corner_pid.state.data)
    imu_corner_error = math.fabs(imu_corner_pid.setpoint.data - imu_corner_pid.state.data)

    # finite differencing on state to estimate derivative (divide by timestep?)
    ir_bottom_diff = math.fabs(ir_bottom_pid.state.data - ir_bottom_pid.reported_states[-9])
    ir_top_diff = math.fabs(ir_top_pid.state.data - ir_top_pid.reported_states[-9])
    imu_wall_diff = math.fabs(imu_wall_pid.state.data - imu_corner_pid.reported_states[-9])
    imu_corner_diff = math.fabs(imu_corner_pid.state.data - imu_corner_pid.reported_states[-9])

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
            imu_setpoint = imu_wall_pid.setpoint.data - math.radians(90)
            imu_wall_pid.imu_setpoint(imu_setpoint)
            imu_corner_pid.imu_setpoint(imu_setpoint)
            robot["state"] = 'corner'
        # either top or bottom IR has detected doorway
        elif (ir_bottom_diff > DOOR_THRESHOLD and ir_bottom_diff < CORNER_THRESHOLD \
        and ir_bottom_error > DOOR_THRESHOLD and ir_bottom_error < CORNER_ERROR_THRESHOLD) or \
           (ir_top_diff > DOOR_THRESHOLD and ir_top_diff < CORNER_THRESHOLD \
           and ir_top_error > DOOR_THRESHOLD and ir_top_error < CORNER_ERROR_THRESHOLD):

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
        if imu_corner_error < math.pi/9:
            print "REACHED IMU SETPOINT WITHIN IMU_THRESHOLD"

            # both IR errors are less than corner state

            if ir_top_error < CORNER_ERROR_THRESHOLD and ir_bottom_error < CORNER_ERROR_THRESHOLD \
            and ir_bottom_diff < DOOR_THRESHOLD:
                # turn top and bottom IR PID control back on
                ir_bottom_pid.ignore = False
                ir_top_pid.ignore = False
                imu_wall_pid.ignore = True
                imu_corner_pid.ignore = True

                robot["state"] = 'wall_follow'

            elif ir_top_error < CORNER_ERROR_THRESHOLD:
                print "only using top ir sensor"
                # turn top IR PID control back on
                ir_bottom_pid.ignore = True
                ir_top_pid.ignore = False
                imu_wall_pid.ignore = True     # may not want to use imu_pid to do wall-following
                imu_corner_pid.ignore = True

                robot["state"] = 'corner'
        else:
            # log imu_corner_pid state and setpoint error during turn
            rospy.loginfo("CORNERING:\t{}\t{}".format(math.degrees(imu_corner_pid.state.data), math.degrees(imu_corner_error)))

    else:
        print "Entered default case in state machine."

    # Set steering command as average of steering commands that we want to use
    i = 0
    steering_cmd = 0
    if not ir_top_pid.ignore:
        i += 1
        steering_cmd += ir_top_pid.control_effort
    if not ir_bottom_pid.ignore:
        i += 1
        steering_cmd += ir_bottom_pid.control_effort
    if not imu_wall_pid.ignore:
        i += 1
        steering_cmd += imu_wall_pid.control_effort
    if not imu_corner_pid.ignore:
        i += 1
        steering_cmd += imu_corner_pid.control_effort
    steering_cmd /= i

    return steering_cmd

def stateMachine_ccs(robot,ir_bottom_pid,ir_top_pid,imu_wall_pid,imu_corner_pid):

    if len(imu_corner_pid.reported_states) < 2:
        return 0

    # Q: How to differentiate between a window and a corner?
    # I believe that in both cases ir_top_diff and ir_top_error will spike
    # beyond the corner threshold. In the case of the window: top_ir will return
    # to a low error value soon after crossing the window, while in the case of
    # a corner, top_ir error will not return a low error value. Additionally, in
    # the case of the window, bottom_ir will

    # My belief is that for the window case, both ir_top_error and ir_bottom_error
    # will not exceed the corner threshold at the same time, whereas, with the
    # corner, both the bottom and top_ir errors will exceed the corner threshold

    # define setpoint error values for state switching logic
    ir_bottom_error = math.fabs(ir_bottom_pid.setpoint.data - ir_bottom_pid.state.data)     # [cm]
    ir_top_error = math.fabs(ir_top_pid.setpoint.data - ir_top_pid.state.data)              # [cm]
    imu_wall_error = math.fabs(imu_wall_pid.setpoint.data - imu_corner_pid.state.data)      # [rad]
    imu_corner_error = math.fabs(imu_corner_pid.setpoint.data - imu_corner_pid.state.data)  # [rad]

    # finite differencing on state to estimate derivative (divide by timestep?)
    ir_bottom_diff = math.fabs(ir_bottom_pid.state.data - ir_bottom_pid.reported_states[-2])    # [cm]
    ir_top_diff = math.fabs(ir_top_pid.state.data - ir_top_pid.reported_states[-2])             # [cm]
    imu_wall_diff = math.fabs(imu_wall_pid.state.data - imu_corner_pid.reported_states[-2])     # [rad]
    imu_corner_diff = math.fabs(imu_corner_pid.state.data - imu_corner_pid.reported_states[-2]) # [rad]

    rospy.loginfo("ir_bottom_diff:\t%f", ir_bottom_diff)
    rospy.loginfo("ir_top_diff:\t%f", ir_top_diff)
    rospy.loginfo("ir_bottom_error:\t%f", ir_bottom_error)
    rospy.loginfo("ir_top_error:\t%f", ir_top_error)

    if robot["state"] == 'wall_follow':
        print "WALL-FOLLOW"

        # doorway detected
        if ir_bottom_error > 700 and ir_top_error < 200:
            print "DOORWAY DETECTED"
            robot["state"] = 'doorway'

            # ignore both IR sensores and switch to IMU PID
            ir_bottom_pid.ignore = True
            ir_top_pid.ignore = True

            # use imu wall-following PID controller and (maybe?) reset IMU setpoints
            imu_wall_pid.ignore = False
            imu_setpoint = imu_wall_pid.state.data
            imu_wall_pid.imu_setpoint(imu_setpoint)
            imu_corner_pid.imu_setpoint(imu_setpoint)

        # corner detected - these values can be decreased
    elif ir_top_error > 10000 or ir_top_diff > 10000:
            print "CORNER DETECTED"
            robot["state"] = 'corner'

            # ignore both IR sensores and switch to IMU PID
            ir_bottom_pid.ignore = True
            ir_top_pid.ignore = True
            imu_wall_pid.ignore = True      # don't know of any reason this should be False at this point

            # enable imu_corner_pid
            imu_corner_pid.ignore = False

            # reset IMU setpoint for cornering task relative to current heading
            # (this is to account for IMU heading drift)
            imu_setpoint = imu_wall_pid.state.data - math.radians(90)
            imu_wall_pid.imu_setpoint(imu_setpoint)
            imu_corner_pid.imu_setpoint(imu_setpoint)

        # continue wall-following
        else:
            # ignore IR top or bottom steering commands for semi-large derivative spikes
            if ir_top_diff > 50 and not ir_top_pid.ignore:
                print "DISABLING TOP IR IN DEFAULT CASE"
                ir_top_pid.ignore = True
            else:
                print "RE-ENABLE TOP IR IN DEFAULT CASE"
                ir_top_pid.ignore = False

            if ir_bottom_diff > 50 and not ir_bottom_pid.ignore:
                print "DISABLING BOTTOM IR IN DEFAULT CASE"
                ir_bottom_pid.ignore = True
            else:
                print "RE-ENABLE BOTTOM IR IN DEFAULT CASE"
                ir_bottom_pid.ignore = False
            # do nothing: continue wall-following

    elif robot["state"] == 'doorway':
        print "DOORWAY"

        # exit doorway andd switch back to wall-following
        if ir_bottom_diff < 75 and ir_top_diff < 75:
            print "EXITING DOORWAY: RETURNING TO WALL-FOLLOW"
            robot["state"] = 'wall_follow'

            ir_bottom_pid.ignore = False
            ir_top_pid.ignore = False
            imu_wall_pid.ignore = True

        # doorway mistaken for corner - needs further tuning (could decrease these values)
        elif ir_top_diff > 10000 or ir_bottom_error > 10000:
            print "CORNER MISTAKEN FOR DOORWAY: ENTERING CORNER"
            robot["state"] = 'corner'

            ir_bottom_pid.ignore = True
            ir_top_pid.ignore = True
            imu_wall_pid.ignore = True      # don't know of any reason this should be False at this point

            # enable imu_corner_pid
            imu_corner_pid.ignore = False

            # reset IMU setpoint for cornering task relative to current heading
            # (this is to account for IMU heading drift)
            imu_setpoint = imu_wall_pid.state.data - math.radians(90)
            imu_wall_pid.imu_setpoint(imu_setpoint)
            imu_corner_pid.imu_setpoint(imu_setpoint)

    elif robot["state"] == 'corner':
        print "CORNERING"
        if imu_corner_error < IMU_THRESHOLD:    # note: make sure IMU_THRESHOLD is in radians
            print "REACHED IMU SETPOINT WITHIN IMU_THRESHOLD"

            # the following if statement will never be true if the imu_corner_error
            # grows large again after it has reached the IMU_THRESHOLD, even if
            # ir derivates have stabilized. This will likely cause some problems.
            # Need to think of another way to handle this...

            # both IR derivatives have stabilized (states not necessarily within DOOR_THRESHOLD)
            if ir_bottom_diff < 100 and ir_top_diff < 100:  # will want to decrease these as much as possible
                print "EXITING CORNER: IR STATE DERIVATIVES HAVE STABILIZED"
                robot["state"] = 'wall_follow'

                # turn IR PID control back on
                ir_bottom_pid.ignore = False
                ir_top_pid.ignore = False
                imu_wall_pid.ignore = True      # may not want to use imu_pid to do wall-following
                imu_corner_pid.ignore = True

        else:
            # log imu_corner_pid state and setpoint error during turn
            rospy.loginfo("CORNERING:\t{}\t{}\t{}".format(math.degrees(imu_corner_pid.state.data), \
        math.degrees(imu_corner_error), math.degrees(imu_corner_pid.setpoint.data)))

    else:
        print "FAULT: Entered default case in state machine."

    # Set steering command as average of steering commands that we want to use
    i = 0
    steering_cmd = 0
    if not ir_top_pid.ignore:
        i += 1
        steering_cmd += ir_top_pid.control_effort
    if not ir_bottom_pid.ignore:
        i += 1
        steering_cmd += ir_bottom_pid.control_effort
    if not imu_wall_pid.ignore:
        i += 1
        steering_cmd += imu_wall_pid.control_effort
    if not imu_corner_pid.ignore:
        i += 1
        steering_cmd += imu_corner_pid.control_effort
    steering_cmd /= i

    return steering_cmd

def stateMachine(robot,ir_bottom_pid,ir_top_pid,imu_wall_pid,imu_corner_pid):

    if len(imu_corner_pid.reported_states) < 2:
        return 0

    # Q: How to differentiate between a window and a corner?
    # I believe that in both cases ir_top_diff and ir_top_error will spike
    # beyond the corner threshold. In the case of the window: top_ir will return
    # to a low error value soon after crossing the window, while in the case of
    # a corner, top_ir error will not return a low error value. Additionally, in
    # the case of the window, bottom_ir will

    # My belief is that for the window case, both ir_top_error and ir_bottom_error
    # will not exceed the corner threshold at the same time, whereas, with the
    # corner, both the bottom and top_ir errors will exceed the corner threshold

    # define setpoint error values for state switching logic
    ir_bottom_error = math.fabs(ir_bottom_pid.setpoint.data - ir_bottom_pid.state.data)     # [cm]
    ir_top_error = math.fabs(ir_top_pid.setpoint.data - ir_top_pid.state.data)              # [cm]
    imu_wall_error = math.fabs(imu_wall_pid.setpoint.data - imu_corner_pid.state.data)      # [rad]
    imu_corner_error = math.fabs(imu_corner_pid.setpoint.data - imu_corner_pid.state.data)  # [rad]

    # finite differencing on state to estimate derivative (divide by timestep?)
    ir_bottom_diff = math.fabs(ir_bottom_pid.state.data - ir_bottom_pid.reported_states[-2])    # [cm]
    ir_top_diff = math.fabs(ir_top_pid.state.data - ir_top_pid.reported_states[-2])             # [cm]
    imu_wall_diff = math.fabs(imu_wall_pid.state.data - imu_corner_pid.reported_states[-2])     # [rad]
    imu_corner_diff = math.fabs(imu_corner_pid.state.data - imu_corner_pid.reported_states[-2]) # [rad]

    rospy.loginfo("ir_bottom_diff:\t%f", ir_bottom_diff)
    rospy.loginfo("ir_top_diff:\t%f", ir_top_diff)
    rospy.loginfo("ir_bottom_error:\t%f", ir_bottom_error)
    rospy.loginfo("ir_top_error:\t%f", ir_top_error)

    if robot["state"] == 'wall_follow':
        print "WALL-FOLLOW"

        # either top or bottom IR has detected doorway (works well!)
        if (ir_bottom_diff > DOOR_THRESHOLD and ir_bottom_diff < CORNER_THRESHOLD) or \
            (ir_top_diff > DOOR_THRESHOLD and ir_top_diff < CORNER_THRESHOLD):
        #if ir_top_diff > DOOR_THRESHOLD and ir_top_diff < CORNER_THRESHOLD:
            print "DOORWAY DETECTED"

            # ignore IR sensor that has detected doorway
            if ir_bottom_diff > DOOR_THRESHOLD:
                print "DISABLE TOP IR PID"
                ir_bottom_pid.ignore = True
            if ir_top_diff > DOOR_THRESHOLD:
                # should not execute
                ir_top_pid.ignore = True

            # use imu wall-following PID controller and (maybe?) reset IMU setpoints
            imu_wall_pid.ignore = False
            imu_setpoint = imu_wall_pid.state.data
            imu_wall_pid.imu_setpoint(imu_setpoint)
            imu_corner_pid.imu_setpoint(imu_setpoint)

            robot["state"] = 'doorway'

        # passing window
        elif (ir_top_diff > CORNER_THRESHOLD and not ir_bottom_diff > CORNER_THRESHOLD) or \
             (ir_bottom_diff > CORNER_THRESHOLD and not ir_bottom_diff > CORNER_THRESHOLD):
            if ir_top_diff > CORNER_THRESHOLD:
                print "TOP IR PASSING WINDOW"
                # may want to disable top IR as it passes window
            if ir_bottom_diff > CORNER_THRESHOLD:
                print "BOTTOM IR PASSING WINDOW"
                # may want to disable bottom IR as it passes window

        # will corner only when both IR sensor errors exceed the corner threshold.
        # The intention is to prevent a window from being misinterpreted as a corner...
        # There may be a better way to make this distinction.
        # elif ir_bottom_diff > CORNER_THRESHOLD or ir_top_diff > CORNER_THRESHOLD:
        elif ir_bottom_error > CORNER_THRESHOLD and ir_top_error > 95:
            print "CORNER DETECTED"
            ir_bottom_pid.ignore = True
            ir_top_pid.ignore = True
            imu_wall_pid.ignore = True      # don't know of any reason this should be False at this point

            # enable imu_corner_pid
            imu_corner_pid.ignore = False

            # reset IMU setpoint for cornering task relative to current heading
            # (this is to account for IMU heading drift)
            imu_setpoint = imu_wall_pid.state.data - math.radians(90)
            imu_wall_pid.imu_setpoint(imu_setpoint)
            imu_corner_pid.imu_setpoint(imu_setpoint)
            robot["state"] = 'corner'

        else:
            if ir_top_error > 110 and not ir_top_pid.ignore:
                print "DISABLING TOP IR IN DEFAULT CASE"
                ir_top_pid.ignore = True
            else:
                ir_top_pid.ignore = False
            # do nothing: continue wall-following

    elif robot["state"] == 'doorway':
        print "DOORWAY"

        if ir_bottom_error > DOOR_THRESHOLD and not ir_bottom_pid.ignore:
            print "DISABLE BOTTOM IR PID"
            ir_bottom_pid.ignore = True
        if ir_top_error > DOOR_THRESHOLD and not ir_top_pid.ignore:
            print "SHOULD NOT PRINT: DISABLE TOP IR PID"
            ir_top_pid.ignore = True

        # only switch back to wall-following after both sensors have cleared the doorway. This will prevent
        # the 'doorway' state from triggering again once the bottom IR sensor passes the doorway since
        # switching to the 'doorway' state is currently based on the abs value of the error derivative
        #if (ir_bottom_error < DOOR_THRESHOLD and ir_top_error < DOOR_THRESHOLD) and \
        #   (ir_bottom_diff < DOOR_THRESHOLD and ir_top_diff < DOOR_THRESHOLD):
        if (ir_bottom_error < 50 and ir_top_error < 50) and \
           (ir_bottom_diff < 30 and ir_top_diff < 30):

            print "EXITING DOORWAY: RETURNING TO WALL-FOLLOW"
            ir_bottom_pid.ignore = False
            ir_top_pid.ignore = False
            imu_wall_pid.ignore = True

            robot["state"] = 'wall_follow'

        # needs further tuning
        elif ir_bottom_error > 600 and ir_top_error > 100:
            print "CORNER MISTAKEN FOR DOORWAY: ENTERING CORNER"

            ir_bottom_pid.ignore = True
            ir_top_pid.ignore = True
            imu_wall_pid.ignore = True      # don't know of any reason this should be False at this point

            # enable imu_corner_pid
            imu_corner_pid.ignore = False

            # reset IMU setpoint for cornering task relative to current heading
            # (this is to account for IMU heading drift)
            imu_setpoint = imu_wall_pid.state.data - math.radians(90)
            imu_wall_pid.imu_setpoint(imu_setpoint)
            imu_corner_pid.imu_setpoint(imu_setpoint)
            robot["state"] = 'corner'


    elif robot["state"] == 'corner':
        print "CORNERING"
        if imu_corner_error < IMU_THRESHOLD:    # note: make sure IMU_THRESHOLD is in radians
            print "REACHED IMU SETPOINT WITHIN IMU_THRESHOLD"

            # the following if statement will never be true if the imu_corner_error
            # grows large again after it has reached the IMU_THRESHOLD, even if
            # ir derivates have stabilized. This will likely cause some problems.
            # Need to think of another way to handle this...

            # both IR derivatives have stabilized (states not necessarily within DOOR_THRESHOLD)
            if ir_bottom_diff < DOOR_THRESHOLD and ir_top_diff < DOOR_THRESHOLD:
                print "EXITING CORNER: IR STATE DERIVATIVES HAVE STABILIZED"
                # turn IR PID control back on
                ir_bottom_pid.ignore = False
                ir_top_pid.ignore = False
                imu_wall_pid.ignore = True      # may not want to use imu_pid to do wall-following
                imu_corner_pid.ignore = True

                robot["state"] = 'wall_follow'

        else:
            # log imu_corner_pid state and setpoint error during turn
            rospy.loginfo("CORNERING:\t{}\t{}\t{}".format(math.degrees(imu_corner_pid.state.data), \
        math.degrees(imu_corner_error), math.degrees(imu_corner_pid.setpoint.data)))

    else:
        print "FAULT: Entered default case in state machine."

    # Set steering command as average of steering commands that we want to use
    i = 0
    steering_cmd = 0
    if not ir_top_pid.ignore:
        i += 1
        steering_cmd += ir_top_pid.control_effort
    if not ir_bottom_pid.ignore:
        i += 1
        steering_cmd += ir_bottom_pid.control_effort
    if not imu_wall_pid.ignore:
        i += 1
        steering_cmd += imu_wall_pid.control_effort
    if not imu_corner_pid.ignore:
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
    if not ir_top_pid.ignore:
        i += 1
        steering_cmd += ir_top_pid.control_effort
    if not ir_bottom_pid.ignore:
        i += 1
        steering_cmd += ir_bottom_pid.control_effort
    if not imu_wall_pid.ignore:
        i += 1
        steering_cmd += imu_wall_pid.control_effort
    if not imu_corner_pid.ignore:
        i += 1
        steering_cmd += imu_corner_pid.control_effort
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
    if not ir_top_pid.ignore:
        i += 1
        steering_cmd += ir_top_pid.control_effort
    if not ir_bottom_pid.ignore:
        i += 1
        steering_cmd += ir_bottom_pid.control_effort
    if not imu_wall_pid.ignore:
        i += 1
        steering_cmd += imu_wall_pid.control_effort
    if not imu_corner_pid.ignore:
        i += 1
        steering_cmd += imu_corner_pid.control_effort
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
    if not ir_top_pid.ignore:
        i += 1
        steering_cmd += ir_top_pid.control_effort
    if not ir_bottom_pid.ignore:
        i += 1
        steering_cmd += ir_bottom_pid.control_effort
    if not imu_wall_pid.ignore:
        i += 1
        steering_cmd += imu_wall_pid.control_effort
    if not imu_corner_pid.ignore:
        i += 1
        steering_cmd += imu_corner_pid.control_effort
    steering_cmd /= i

    return steering_cmd


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
        imu_corner_pid.imu_setpoint()
        imu_wall_pid.imu_setpoint(imu_corner_pid.setpoint.data)
        ir_bottom_pid.ir_setpoint()
        ir_top_pid.ir_setpoint()

        # Set zero intial velocity and steering
        motor_srv(MOTOR_CENTER)
        steering_srv(STEERING_CENTER)
        rospy.sleep(1)

        # Set forward speed
        motor_srv(MOTOR_SPEED)
        print "MOTOR SPEED: ", MOTOR_SPEED
        #motor.set_target(MOTOR_SPEED)

        # Iteration rate
        rate = rospy.Rate(RATE)

        # Initialize stateMachine()
        robot = {"state": "wall_follow"}
        imu_wall_pid.ignore = True
        imu_corner_pid.ignore = True

        time_since_turn = rospy.get_time()

        # Count iterations
        # Can be used for debugging or other miscellaneous needs
        count = 0
        while not rospy.is_shutdown():

            # Publish sensor states
            ir_bottom_pid.ir_publish_state()
            ir_top_pid.ir_publish_state()
            imu_wall_pid.imu_publish_state()
            imu_corner_pid.imu_publish_state(imu_wall_pid.state.data)

            # Heuristics
            # steering_cmd = test_imu(ir_bottom_pid, ir_top_pid, imu_wall_pid, imu_corner_pid)
            # steering_cmd = heuristic3(ir_bottom_pid,ir_top_pid,imu_pid)
            # steering_cmd = heuristic4(ir_bottom_pid,ir_top_pid,imu_pid, \
            #                           ir_bottom_state,ir_top_state,imu_state)
            steering_cmd = stateMachine(robot, ir_bottom_pid, ir_top_pid, imu_wall_pid, imu_corner_pid)
            #steering_cmd = kodiesStateMachine1(robot, ir_bottom_pid, ir_top_pid, imu_wall_pid, imu_corner_pid)

            # Set steering target
            steering_cmd += STEERING_CENTER
            #steering.set_target(steering_cmd)
            steering_srv(steering_cmd)

            # Print statements
            #for pid in pids:
            #    rospy.loginfo("Setpoint for {} = {}".format(pid.sensor, pid.setpoint.data))
            #for pid in pids:
            #    rospy.loginfo("State for {} = {}".format(pid.sensor, pid.state.data))
            #for pid in pids:
            #    rospy.loginfo("Control effort for {} = {}".format(pid.sensor, CENTER+pid.control_effort))
            #rospy.loginfo("Steering Command:\t%d", steering_cmd)
            print

            # Count variable for debugging and miscellaneous uses
            count += 1

            # Iterate at frequency of rate
            rate.sleep()


if __name__ == '__main__':

    # Initialize node
    rospy.init_node('odroid_node', anonymous=True)

    # Import param
    DUMMY_MODE = rospy.get_param('~dummy_mode')
    MIN = rospy.get_param('~min')
    MAX = rospy.get_param('~max')
    #CENTER = rospy.get_param('~center')
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
    DOOR_THRESHOLD = 150
    CORNER_THRESHOLD = 600
    IMU_THRESHOLD = math.radians(15)

    try:
        odroid()
    except rospy.ROSInterruptException:
        pass
