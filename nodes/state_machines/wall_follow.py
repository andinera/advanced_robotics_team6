#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math

ir_bottom_pid = None
ir_top_pid = None
imu_wall_pid = None
imu_corner_pid = None
steering_srv = None
STEERING_CENTER = 5800
rtrn = 'outcome1'

# define state Foo
class Wall_Follow(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1','outcome2', 'outcome3'])

    def execute(self, userdata):


        while not rospy.is_shutdown() and ir_bottom_pid.sync == 0:
            pass
        while len(imu_corner_pid.reported_states) < 9:
            ir_bottom_pid.sync = 0
            pass
        ir_bottom_pid.sync = 0
        global rtrn
        # define setpoint error values for state switching logic
        ir_bottom_error = math.fabs(ir_bottom_pid.setpoint.data - ir_bottom_pid.state.data)     # [cm]
        ir_top_error = math.fabs(ir_top_pid.setpoint.data - ir_top_pid.state.data)              # [cm]
        imu_wall_error = math.fabs(imu_wall_pid.setpoint.data - imu_corner_pid.state.data)      # [rad]
        imu_corner_error = math.fabs(imu_corner_pid.setpoint.data - imu_corner_pid.state.data)  # [rad]

        # finite differencing on state to estimate derivative (divide by timestep?)
        ir_bottom_diff = math.fabs(ir_bottom_pid.state.data - ir_bottom_pid.reported_states[-9])    # [cm]
        ir_top_diff = math.fabs(ir_top_pid.state.data - ir_top_pid.reported_states[-9])             # [cm]
        imu_wall_diff = math.fabs(imu_wall_pid.state.data - imu_corner_pid.reported_states[-9])     # [rad]
        imu_corner_diff = math.fabs(imu_corner_pid.state.data - imu_corner_pid.reported_states[-9]) # [rad]

        print "WALL-FOLLOW"
        rospy.loginfo("ir_bottom_diff:\t%f", ir_bottom_diff)
        rospy.loginfo("ir_top_diff:\t%f", ir_top_diff)
        rospy.loginfo("ir_bottom_error:\t%f",ir_bottom_error)
        rospy.loginfo("ir_top_error:\t%f",ir_top_error)
        # either top or bottom IR has detected corner
        if ir_top_error > 14000 or ir_bottom_error > 14000:
            print "CORNER DETECTED"
            rtrn = 'outcome3'

            ir_bottom_pid.ignore = True
            ir_top_pid.ignore = True
            imu_wall_pid.ignore = True      # don't know of any reason this should be False at this point

            # enable imu_corner_pid
            imu_corner_pid.ignore = False

            # reset IMU setpoint for cornering task
            imu_setpoint = imu_corner_pid.state.data - math.radians(90)
            imu_wall_pid.imu_setpoint(imu_setpoint)
            imu_corner_pid.imu_setpoint(imu_setpoint)

        else:
            #protect against entering or exiting a corner
            if ir_bottom_error < 14000 and ir_top_error < 14000:
                ir_bottom_pid.ignore = False
                ir_top_pid.ignore = False
                print "wall_follow_1"

            elif ir_bottom_error > 14000:
                ir_bottom_pid.ignore = True
                imu_corner_pid.ignore = False
                ir_top_pid.ignore = False
                print "cornering_1"
                rtrn = 'outcome3'

            elif ir_top_error > 14000:
                ir_top_pid.ignore = True
                ir_bottom_pid.ignore = False
                imu_corner_pid.ignore = False
                print "cornering_2"
                rtrn = 'outcome3'

        # Set steering command as average of steering commands that we want to use
        i = 0
        steering_cmd = 0
        if not ir_top_pid.ignore:
            i += 1
            steering_cmd += ir_top_pid.control_effort
            #rospy.loginfo("steering_cmd_top:\t{}".format(ir_top_pid.control_effort))
        if not ir_bottom_pid.ignore:
            i += 1
            steering_cmd += ir_bottom_pid.control_effort
            #rospy.loginfo("steering_cmd_bottom:\t{}".format(ir_bottom_pid.control_effort))

        if not imu_wall_pid.ignore:
            i += 1
            steering_cmd += imu_wall_pid.control_effort
            #rospy.loginfo("steering_cmd_wall:\t{}".format(imu_wall_pid.control_effort))

        if not imu_corner_pid.ignore:
            i += 1
            steering_cmd += imu_corner_pid.control_effort
            #rospy.loginfo("steering_cmd_corner:\t{}".format(imu_corner_pid.control_effort))

        steering_cmd /= i
        rospy.loginfo("steering_cmd:\t{}".format(steering_cmd))

        steering_srv(STEERING_CENTER + steering_cmd)

        return rtrn

class Doorway(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1', 'outcome2', 'outcome3'])

    def execute(self, userdata):
        while not rospy.is_shutdown() and ir_bottom_pid.sync == 0:
            pass
        ir_bottom_pid.sync = 0
        global rtrn
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

        print "Entered default case in state machine."

        rtrn = 'outcom2'

        return rtrn


# define state Bar
class Corner(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1', 'outcome2', 'outcome3'])

    def execute(self, userdata):
        while not rospy.is_shutdown() and ir_bottom_pid.sync == 0:
            pass
        ir_bottom_pid.sync = 0
        global rtrn
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

        print "CORNERING"
        if imu_corner_error < math.pi/4.5:
            print "REACHED IMU SETPOINT WITHIN IMU_THRESHOLD"

            # both IR errors are less than corner state
            if ir_top_error < 14000 and ir_bottom_error < 14000:
                # turn top and bottom IR PID control back on
                ir_bottom_pid.ignore = False
                ir_top_pid.ignore = False
                imu_wall_pid.ignore = True
                imu_corner_pid.ignore = True
                rtrn = 'outcome1'

                print "wall_follow_2"

        else:
            # log imu_corner_pid state and setpoint error during turn
            rospy.loginfo("CORNERING:\t{}\t{}".format(math.degrees(imu_corner_pid.state.data), math.degrees(imu_corner_error)))

        return rtrn


def main(ir_bottom, ir_top, imu_wall, imu_corner, steering):

    global ir_bottom_pid, ir_top_pid, imu_wall_pid, imu_corner_pid, steering_srv
    ir_bottom_pid = ir_bottom
    ir_top_pid = ir_top
    imu_wall_pid = imu_wall
    imu_corner_pid = imu_corner
    steering_srv = steering


    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    print ir_bottom_pid.reported_states

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Wall_Follow', Wall_Follow(),
                               transitions={'outcome1':'Wall_Follow',
                                            'outcome2':'Doorway',
                                            'outcome3':'Corner'},
                               remapping={})
        smach.StateMachine.add('Corner', Corner(),
                               transitions={'outcome1':'Wall_Follow',
                                            'outcome2':'Doorway',
                                            'outcome3':'Corner'},
                               remapping={})
        smach.StateMachine.add('Doorway', Doorway(),
                                transitions={'outcome1':'Wall_Follow',
                                             'outcome2':'Doorway',
                                             'outcome3':'Corner'},
                                remapping={})


    # Execute SMACH plan
    return sm.execute()
