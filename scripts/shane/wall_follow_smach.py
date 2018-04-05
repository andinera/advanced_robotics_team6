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

        while not rospy.is_shutdown() and len(wf.cns.imu_states['orientation']['z']) < 9:
            wf.event.wait()
            wf.event.clear()
            wf.publish_states()

        wf.event.wait()
        wf.event.clear()
        global rtrn

        # define setpoint error values for state switching logic
        ir_bottom_error = math.fabs(wf.bottom_ir_pid.setpoint.data - wf.bottom_ir_pid.state.data)     # [cm]
        ir_top_error = math.fabs(wf.top_ir_pid.setpoint.data - wf.top_ir_pid.state.data)              # [cm]
        imu_wall_error = math.fabs(wf.wall_imu_pid.setpoint.data - wf.wall_imu_pid.state.data)      # [rad]
        imu_corner_error = math.fabs(wf.corner_imu_pid.setpoint.data - wf.corner_imu_pid.state.data)  # [rad]

        # finite differencing on state to estimate derivative (divide by timestep?)
        ir_bottom_diff = math.fabs(wf.bottom_ir_pid.state.data - wf.cns.bottom_ir_states[-9])    # [cm]
        ir_top_diff = math.fabs(wf.top_ir_pid.state.data - wf.cns.top_ir_states[-9])             # [cm]
        imu_wall_diff = math.fabs(wf.wall_imu_pid.state.data - wf.cns.imu_states['orientation']['z'][-9])     # [rad]
        imu_corner_diff = math.fabs(wf.corner_imu_pid.state.data - wf.cns.imu_states['orientation']['z'][-9]) # [rad]
        corner_count = 0

        wf.motor_srv(6250)
        # either top or bottom IR has detected corner
        if ir_bottom_error > 1000 and ir_bottom_diff > 1000 and corner_count < 3:
            rtrn = 'outcome3'
            wf.motor_srv(6150)
            corner_count += 1
            wf.bottom_ir_pid.ignore = True
            wf.top_ir_pid.ignore = True
            wf.wall_imu_pid.ignore = True      # don't know of any reason this should be False at this point

            # enable corner_imu_pid
            wf.corner_imu_pid.ignore = False

            # reset IMU setpoint for cornering task
            imu_setpoint = wf.corner_imu_pid.state.data - math.radians(90)
            wf.wall_imu_pid.imu_setpoint(setpoint=imu_setpoint)
            wf.corner_imu_pid.imu_setpoint(setpoint=imu_setpoint)

        # either top or bottom IR has detected doorway
        elif ir_top_error > 500 and ir_top_error < 5000 and \
                    ir_top_diff > 500 and ir_top_diff < 5000:
            rtrn = 'outcome1'

            # reset IMU setpoint for cornering task
            imu_setpoint = wf.wall_imu_pid.state.data
            wf.wall_imu_pid.imu_setpoint(setpoint=imu_setpoint)

            wf.bottom_ir_pid.ignore = True
            wf.top_ir_pid.ignore = True
            # use imu wall-following PID controller
            wf.wall_imu_pid.ignore = False

        else:
            #protect against entering or exiting a corner
            if ir_bottom_error < wf.corner_error_threshold and ir_top_error < wf.corner_error_threshold:
                wf.bottom_ir_pid.ignore = False
                wf.top_ir_pid.ignore = False
            elif ir_bottom_error > wf.corner_error_threshold:
                wf.bottom_ir_pid.ignore = True
            elif ir_top_error > wf.corner_error_threshold:
                wf.top_ir_pid.ignore = True

        wf.publish_steering_cmd()
        wf.publish_states()

        return rtrn

class Doorway(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1', 'outcome2', 'outcome3'])

    def execute(self, userdata):

        wf.event.wait()
        wf.event.clear()

        global rtrn

        # define setpoint error values for state switching logic
        ir_bottom_error = math.fabs(wf.bottom_ir_pid.setpoint.data - wf.bottom_ir_pid.state.data)     # [cm]
        ir_top_error = math.fabs(wf.top_ir_pid.setpoint.data - wf.top_ir_pid.state.data)              # [cm]
        imu_wall_error = math.fabs(wf.wall_imu_pid.setpoint.data - wf.corner_imu_pid.state.data)      # [rad]
        imu_corner_error = math.fabs(wf.corner_imu_pid.setpoint.data - wf.corner_imu_pid.state.data)  # [rad]

        # finite differencing on state to estimate derivative (divide by timestep?)
        ir_bottom_diff = math.fabs(wf.bottom_ir_pid.state.data - wf.cns.bottom_ir_states[-9])    # [cm]
        ir_top_diff = math.fabs(wf.top_ir_pid.state.data - wf.cns.top_ir_states[-9])             # [cm]
        imu_wall_diff = math.fabs(wf.wall_imu_pid.state.data - wf.cns.imu_states['orientation']['z'][-9])     # [rad]
        imu_corner_diff = math.fabs(wf.corner_imu_pid.state.data - wf.cns.imu_states['orientation']['z'][-9]) # [rad]
        corner_count = 0

        rtrn = 'outcome2'

        wf.publish_steering_cmd()
        wf.publish_states()

        return rtrn


# define state Bar
class Corner(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1', 'outcome2', 'outcome3'])

    def execute(self, userdata):

        wf.event.wait()
        wf.event.clear()

        global rtrn

        # define setpoint error values for state switching logic
        ir_bottom_error = math.fabs(wf.wf.bottom_ir_pid.setpoint.data - wf.bottom_ir_pid.state.data)     # [cm]
        ir_top_error = math.fabs(wf.top_ir_pid.setpoint.data - wf.top_ir_pid.state.data)              # [cm]
        imu_wall_error = math.fabs(wf.wall_imu_pid.setpoint.data - wf.corner_imu_pid.state.data)      # [rad]
        imu_corner_error = math.fabs(wf.corner_imu_pid.setpoint.data - wf.corner_imu_pid.state.data)  # [rad]

        # finite differencing on state to estimate derivative (divide by timestep?)
        ir_bottom_diff = math.fabs(wf.bottom_ir_pid.state.data - wf.cns.bottom_ir_states[-9])    # [cm]
        ir_top_diff = math.fabs(wf.top_ir_pid.state.data - wf.cns.top_ir_states[-9])             # [cm]
        imu_wall_diff = math.fabs(wf.wall_imu_pid.state.data - wf.cns.imu_states['orientation']['z'][-9])     # [rad]
        imu_corner_diff = math.fabs(wf.corner_imu_pid.state.data - wf.cns.imu_states['orientation']['z'][-9]) # [rad]
        corner_count = 0

        if imu_corner_error < math.pi/4.5:

            # both IR errors are less than corner state

            if ir_top_error < 100 and ir_bottom_error < 100:
                # turn top and bottom IR PID control back on
                wf.bottom_ir_pid.ignore = False
                wf.top_ir_pid.ignore = False
                wf.wall_imu_pid.ignore = True
                wf.corner_imu_pid.ignore = True

                rtrn = 'outcome1'

            elif ir_top_error < 100 :
                # turn top IR PID control back on
                wf.bottom_ir_pid.ignore = True
                wf.top_ir_pid.ignore = False
                wf.wall_imu_pid.ignore = True     # may not want to use imu_pid to do wall-following
                wf.corner_imu_pid.ignore = True

                rtrn = 'outcome3'
        else:
            pass
            # log corner_imu_pid state and setpoint error during turn

        wf.publish_steering_cmd()
        wf.publish_states()

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
