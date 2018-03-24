#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math

wf = None
STEERING_CENTER = 5800
rtrn = 'outcome1'

# define state Foo
class Wall(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1','outcome2', 'outcome3'])

    def execute(self, userdata):

        while not rospy.is_shutdown() and len(wf.imu_corner_pid.reported_states) < 9:
            wf.sync = 0

        while not rospy.is_shutdown() and wf.sync == 0:
            pass
        wf.sync = 0

        global rtrn

        # define setpoint error values for state switching logic
        ir_bottom_error = math.fabs(wf.ir_bottom_pid.setpoint.data - wf.ir_bottom_pid.state.data)     # [cm]
        ir_top_error = math.fabs(wf.ir_top_pid.setpoint.data - wf.ir_top_pid.state.data)              # [cm]
        imu_wall_error = math.fabs(wf.imu_wall_pid.setpoint.data - wf.imu_corner_pid.state.data)      # [rad]
        imu_corner_error = math.fabs(wf.imu_corner_pid.setpoint.data - wf.imu_corner_pid.state.data)  # [rad]

        # finite differencing on state to estimate derivative (divide by timestep?)
        ir_bottom_diff = math.fabs(wf.ir_bottom_pid.state.data - wf.ir_bottom_pid.reported_states[-9])    # [cm]
        ir_top_diff = math.fabs(wf.ir_top_pid.state.data - wf.ir_top_pid.reported_states[-9])             # [cm]
        imu_wall_diff = math.fabs(wf.imu_wall_pid.state.data - wf.imu_corner_pid.reported_states[-9])     # [rad]
        imu_corner_diff = math.fabs(wf.imu_corner_pid.state.data - wf.imu_corner_pid.reported_states[-9]) # [rad]
        corner_count = 0

        print "WALL-FOLLOW"
        wf.motor_srv(6250)
        rospy.loginfo("ir_bottom_diff:\t%f", ir_bottom_diff)
        rospy.loginfo("ir_top_diff:\t%f", ir_top_diff)
        rospy.loginfo("ir_bottom_error:\t%f",ir_bottom_error)
        rospy.loginfo("ir_top_error:\t%f",ir_top_error)
        # either top or bottom IR has detected corner
        if ir_bottom_error > 1000 and ir_bottom_diff > 1000 and corner_count < 3:
            print "CORNER DETECTED"
            rtrn = 'outcome3'
            wf.motor_srv(6150)
            corner_count += 1
            wf.ir_bottom_pid.ignore = True
            wf.ir_top_pid.ignore = True
            wf.imu_wall_pid.ignore = True      # don't know of any reason this should be False at this point

            # enable imu_corner_pid
            wf.imu_corner_pid.ignore = False

            # reset IMU setpoint for cornering task
            imu_setpoint = wf.imu_corner_pid.state.data - math.radians(90)
            wf.imu_wall_pid.imu_setpoint(imu_setpoint)
            wf.imu_corner_pid.imu_setpoint(imu_setpoint)

        # either top or bottom IR has detected doorway
        elif ir_top_error > 500 and ir_top_error < 5000 and \
                    ir_top_diff > 500 and ir_top_diff < 5000:
            print "DOORWAY DETECTED"
            rtrn = 'outcome1'

            # reset IMU setpoint for cornering task
            imu_setpoint = wf.imu_wall_pid.state.data
            wf.imu_wall_pid.imu_setpoint(imu_setpoint)

            wf.ir_bottom_pid.ignore = True
            wf.ir_top_pid.ignore = True
            # use imu wall-following PID controller
            wf.imu_wall_pid.ignore = False

        else:
            #protect against entering or exiting a corner
            if ir_bottom_error < wf.corner_error_threshold and ir_top_error < wf.corner_error_threshold:
                wf.ir_bottom_pid.ignore = False
                wf.ir_top_pid.ignore = False
            elif ir_bottom_error > wf.corner_error_threshold:
                wf.ir_bottom_pid.ignore = True
            elif ir_top_error > wf.corner_error_threshold:
                wf.ir_top_pid.ignore = True

        # Set steering command as average of steering commands that we want to use
        i = 0
        steering_cmd = 0
        if not wf.ir_top_pid.ignore:
            i += 1
            steering_cmd += wf.ir_top_pid.control_effort
            #rospy.loginfo("steering_cmd_top:\t{}".format(ir_top_pid.control_effort))
        if not wf.ir_bottom_pid.ignore:
            i += 1
            steering_cmd += wf.ir_bottom_pid.control_effort
            #rospy.loginfo("steering_cmd_bottom:\t{}".format(ir_bottom_pid.control_effort))

        if not wf.imu_wall_pid.ignore:
            i += 1
            steering_cmd += wf.imu_wall_pid.control_effort
            #rospy.loginfo("steering_cmd_wall:\t{}".format(imu_wall_pid.control_effort))

        if not wf.imu_corner_pid.ignore:
            i += 1
            steering_cmd += wf.imu_corner_pid.control_effort
            #rospy.loginfo("steering_cmd_corner:\t{}".format(imu_corner_pid.control_effort))

        steering_cmd /= i
        rospy.loginfo("steering_cmd:\t{}".format(steering_cmd))

        wf.steering_srv(STEERING_CENTER + steering_cmd)

        return rtrn

class Doorway(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1', 'outcome2', 'outcome3'])

    def execute(self, userdata):

        while not rospy.is_shutdown() and wf.sync == 0:
            pass
        wf.sync = 0

        global rtrn

        # define setpoint error values for state switching logic
        ir_bottom_error = math.fabs(wf.ir_bottom_pid.setpoint.data - wf.ir_bottom_pid.state.data)     # [cm]
        ir_top_error = math.fabs(wf.ir_top_pid.setpoint.data - wf.ir_top_pid.state.data)              # [cm]
        imu_wall_error = math.fabs(wf.imu_wall_pid.setpoint.data - wf.imu_corner_pid.state.data)      # [rad]
        imu_corner_error = math.fabs(wf.imu_corner_pid.setpoint.data - wf.imu_corner_pid.state.data)  # [rad]

        # finite differencing on state to estimate derivative (divide by timestep?)
        ir_bottom_diff = math.fabs(ir_bottom_pid.state.data - ir_bottom_pid.reported_states[-2])    # [cm]
        ir_top_diff = math.fabs(ir_top_pid.state.data - ir_top_pid.reported_states[-2])             # [cm]
        imu_wall_diff = math.fabs(imu_wall_pid.state.data - imu_corner_pid.reported_states[-2])     # [rad]
        imu_corner_diff = math.fabs(imu_corner_pid.state.data - imu_corner_pid.reported_states[-2]) # [rad]
        corner_count = 0

        print "Entered default case in state machine."

        rtrn = 'outcome2'

        # Set steering command as average of steering commands that we want to use
        i = 0
        steering_cmd = 0
        if not wf.ir_top_pid.ignore:
            i += 1
            steering_cmd += wf.ir_top_pid.control_effort
            #rospy.loginfo("steering_cmd_top:\t{}".format(ir_top_pid.control_effort))
        if not wf.ir_bottom_pid.ignore:
            i += 1
            steering_cmd += wf.ir_bottom_pid.control_effort
            #rospy.loginfo("steering_cmd_bottom:\t{}".format(ir_bottom_pid.control_effort))

        if not wf.imu_wall_pid.ignore:
            i += 1
            steering_cmd += wf.imu_wall_pid.control_effort
            #rospy.loginfo("steering_cmd_wall:\t{}".format(imu_wall_pid.control_effort))

        if not wf.imu_corner_pid.ignore:
            i += 1
            steering_cmd += wf.imu_corner_pid.control_effort
            #rospy.loginfo("steering_cmd_corner:\t{}".format(imu_corner_pid.control_effort))

        steering_cmd /= i
        rospy.loginfo("steering_cmd:\t{}".format(steering_cmd))

        wf.steering_srv(STEERING_CENTER + steering_cmd)

        return rtrn


# define state Bar
class Corner(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1', 'outcome2', 'outcome3'])

    def execute(self, userdata):

        while not rospy.is_shutdown() and wf.sync == 0:
            pass
        wf.sync = 0

        global rtrn

        # define setpoint error values for state switching logic
        ir_bottom_error = math.fabs(wf.wf.ir_bottom_pid.setpoint.data - wf.ir_bottom_pid.state.data)     # [cm]
        ir_top_error = math.fabs(wf.ir_top_pid.setpoint.data - wf.ir_top_pid.state.data)              # [cm]
        imu_wall_error = math.fabs(wf.imu_wall_pid.setpoint.data - wf.imu_corner_pid.state.data)      # [rad]
        imu_corner_error = math.fabs(wf.imu_corner_pid.setpoint.data - wf.imu_corner_pid.state.data)  # [rad]

        # finite differencing on state to estimate derivative (divide by timestep?)
        ir_bottom_diff = math.fabs(wf.ir_bottom_pid.state.data - wf.ir_bottom_pid.reported_states[-2])    # [cm]
        ir_top_diff = math.fabs(wf.ir_top_pid.state.data - wf.ir_top_pid.reported_states[-2])             # [cm]
        imu_wall_diff = math.fabs(wf.imu_wall_pid.state.data - wf.imu_corner_pid.reported_states[-2])     # [rad]
        imu_corner_diff = math.fabs(wf.imu_corner_pid.state.data - wf.imu_corner_pid.reported_states[-2]) # [rad]

        print "CORNERING"
        rospy.loginfo("CORNERING:\t{}".format(imu_corner_pid))
        if imu_corner_error < math.pi/4.5:
            print "REACHED IMU SETPOINT WITHIN IMU_THRESHOLD"

            # both IR errors are less than corner state

            if ir_top_error < 100 and ir_bottom_error < 100:
                # turn top and bottom IR PID control back on
                wf.ir_bottom_pid.ignore = False
                wf.ir_top_pid.ignore = False
                wf.imu_wall_pid.ignore = True
                wf.imu_corner_pid.ignore = True

                rtrn = 'outcome1'

            elif ir_top_error < 100 :
                # turn top IR PID control back on
                wf.ir_bottom_pid.ignore = True
                wf.ir_top_pid.ignore = False
                wf.imu_wall_pid.ignore = True     # may not want to use imu_pid to do wall-following
                wf.imu_corner_pid.ignore = True

                rtrn = 'outcome3'
                print "Using top ir sensor for wall follow"
        else:
            # log imu_corner_pid state and setpoint error during turn
            rospy.loginfo("CORNERING:\t{}\t{}".format(math.degrees(imu_corner_pid.state.data), math.degrees(imu_corner_error)))

        # Set steering command as average of steering commands that we want to use
        i = 0
        steering_cmd = 0
        if not wf.ir_top_pid.ignore:
            i += 1
            steering_cmd += wf.ir_top_pid.control_effort
            #rospy.loginfo("steering_cmd_top:\t{}".format(ir_top_pid.control_effort))
        if not wf.ir_bottom_pid.ignore:
            i += 1
            steering_cmd += wf.ir_bottom_pid.control_effort
            #rospy.loginfo("steering_cmd_bottom:\t{}".format(ir_bottom_pid.control_effort))

        if not wf.imu_wall_pid.ignore:
            i += 1
            steering_cmd += wf.imu_wall_pid.control_effort
            #rospy.loginfo("steering_cmd_wall:\t{}".format(imu_wall_pid.control_effort))

        if not wf.imu_corner_pid.ignore:
            i += 1
            steering_cmd += wf.imu_corner_pid.control_effort
            #rospy.loginfo("steering_cmd_corner:\t{}".format(imu_corner_pid.control_effort))

        steering_cmd /= i
        rospy.loginfo("steering_cmd:\t{}".format(steering_cmd))

        wf.steering_srv(STEERING_CENTER + steering_cmd)

        return rtrn


def main(wall_follower):

    global wf
    wf = wall_follower

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Wall', Wall(),
                               transitions={'outcome1':'Wall',
                                            'outcome2':'Doorway',
                                            'outcome3':'Corner'},
                               remapping={})
        smach.StateMachine.add('Corner', Corner(),
                               transitions={'outcome1':'Wall',
                                            'outcome2':'Doorway',
                                            'outcome3':'Corner'},
                               remapping={})
        smach.StateMachine.add('Doorway', Doorway(),
                                transitions={'outcome1':'Wall',
                                             'outcome2':'Doorway',
                                             'outcome3':'Corner'},
                                remapping={})


    # Execute SMACH plan
    return sm.execute()
