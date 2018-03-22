#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from advanced_robotics_team6.srv import PololuCmd
from tf.transformations import euler_from_quaternion
import math
from drivers import pid_driver

WRITE_DATA = True
if WRITEDATA:
    print "OPENING CSV"
    csv_out = open("/home/odroid/ros_ws/src/advanced_robotics_team6/data/ir_course_data.csv", "a")
    writer = csv.writer(csv_out)

def DOPEStateMachine(robot,ir_bottom_pid,ir_top_pid,imu_wall_pid,imu_corner_pid):

    if len(imu_corner_pid.reported_states) < 4:
        return 0

    # define setpoint error values for state switching logic
    ir_bottom_error = math.fabs(ir_bottom_pid.setpoint.data - ir_bottom_pid.state.data)
    ir_top_error = math.fabs(ir_top_pid.setpoint.data - ir_top_pid.state.data)
    imu_wall_error = math.fabs(imu_wall_pid.setpoint.data - imu_corner_pid.state.data)
    imu_corner_error = math.fabs(imu_corner_pid.setpoint.data - imu_corner_pid.state.data)

    # finite differencing on state to estimate derivative (divide by timestep?)

    ir_bottom_diff = math.fabs(ir_bottom_pid.state.data - ir_bottom_pid.reported_states[-2])
    ir_top_diff = math.fabs(ir_top_pid.state.data - ir_top_pid.reported_states[-2])
    ir_top_difference = ir_top_pid.state.data - ir_top_pid.reported_states[-2]
    imu_wall_diff = math.fabs(imu_wall_pid.state.data - imu_corner_pid.reported_states[-2])
    imu_corner_diff = math.fabs(imu_corner_pid.state.data - imu_corner_pid.reported_states[-2])

    ir_bottom_average_error = math.fabs(ir_bottom_pid.setpoint.data - (ir_bottom_pid.reported_states[-1] + ir_bottom_pid.reported_states[-2] + ir_bottom_pid.reported_states[-3])/3)

    if WRITE_DATA:
        print "WRITING DATA"
        writer.writerow([ir_bottom_error, ir_top_error, ir_bottom_diff, ir_top_diff])

    if robot["state"] == 'wall_follow':
        print "WALL-FOLLOW"
        rospy.loginfo("ir_bottom_diff:\t%f", ir_bottom_diff)
        rospy.loginfo("ir_top_diff:\t%f", ir_top_diff)
        rospy.loginfo("ir_bottom_error:\t%f",ir_bottom_error)
        rospy.loginfo("ir_top_error:\t%f",ir_top_error)
        # either top or bottom IR has detected corner
        if ir_bottom_error > BOTTOM_C_MIN and ir_top_error > TOP_C_MIN and ir_top_error < TOP_C_MAX and imu_corner_pid.turns_completed < 2:
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
        elif ir_top_error > TOP_D_MIN or (ir_bottom_error > BOTTOM_D_MIN and ir_bottom_error < BOTTOM_D_MAX):

            print "DOORWAY DETECTED"

            # ignore IR sensor that has detected doorway
            ir_bottom_pid.ignore = True
            ir_top_pid.ignore = True

            # use imu wall-following PID controller
            imu_wall_pid.ignore = False
            robot["state"] = 'doorway'

        else:
            #protect against entering or exiting a corner
	    if ir_bottom_error < 5 and ir_top_error < 5:
		# reset IMU setpoint for cornering task
            	imu_setpoint = 0
            	headings = imu_wall_pid.recorded_states[:]
            	for i in range(-1,-9,-1):
                    imu_setpoint = imu_setpoint + headings[i]/8

            	imu_wall_pid.imu_setpoint(imu_setpoint)
		imu_corner_pid.imu_setpoint(imu_setpoint)

            if ir_bottom_error < BOTTOM_C_MIN and ir_top_error < TOP_C_MIN:
                ir_bottom_pid.ignore = False
                ir_top_pid.ignore = False
            elif ir_bottom_error > BOTTOM_C_MIN:
                ir_bottom_pid.ignore = True
                print "ignoring bottom IR while wall following"
            elif ir_top_error > TOP_C_MIN:
                ir_top_pid.ignore = True
                print "ignoring top IR while wall following"


    elif robot["state"] == 'doorway':
        print "DOORWAY"
        rospy.loginfo("ir_bottom_diff:\t%f", ir_bottom_diff)
        rospy.loginfo("ir_top_diff:\t%f", ir_top_diff)
        rospy.loginfo("ir_bottom_error:\t%f", ir_bottom_error)
        rospy.loginfo("ir_top_error:\t%f", ir_top_error)

        #if ir_top_error > CORNER_THRESHOLD:
            #ir_top_pid.ignore = True
            #robot["state"] = 'wall_follow'
            #print "exit becasue top corner threshold"
        if ir_bottom_error > BOTTOM_C_MIN and ir_top_error > TOP_C_MIN and ir_top_error < TOP_C_MAX and ir_top_diff < 100:
            #ir_bottom_pid.ignore = True
            #robot["state"] = 'corner'
            #imu_setpoint = imu_wall_pid.recorded_states[-1] - math.radians(90)
            #imu_wall_pid.imu_setpoint(imu_setpoint)
            #imu_corner_pid.imu_setpoint(imu_setpoint)
    	    robot["state"] = 'wall_follow'
            print "exit to wall_follow because bottom corner threshold"

        elif ir_bottom_error < 100 and ir_top_error < 100 and ir_bottom_diff < 30 and ir_top_diff < 30:
            ir_bottom_pid.ignore = False
            ir_top_pid.ignore = False
            imu_wall_pid.ignore = True

            robot["state"] = 'wall_follow'
            print "Exited Doorway with standard method"

    elif robot["state"] == 'corner':
        print "CORNERING"
        rospy.loginfo("ir_bottom_state:\t%f", ir_bottom_pid.state.data)
        rospy.loginfo("ir_top_state:\t%f", ir_top_pid.state.data)
        rospy.loginfo("ir_bottom_setpoint:\t%f", ir_bottom_pid.setpoint.data)
        rospy.loginfo("ir_top_setpoint:\t%f", ir_top_pid.setpoint.data)
        rospy.loginfo("CORNERING:\t{}\t{}".format(math.degrees(imu_corner_pid.state.data), math.degrees(imu_corner_error)))
        #if imu_corner_error > math.radians(85) and ir_top_difference < 0 and ir_top_pid.state.data < ir_top_pid.setpoint.data :
        #ir_bottom_pid.ignore = False
        #    ir_top_pid.ignore = False
        #    imu_wall_pid.ignore = True
        #    imu_corner_pid.ignore = True
        #    robot["state"] = 'wall_follow'
        #    print "exited turn due to top IR getting closer"
        if imu_corner_error < math.pi/9:
            print "REACHED IMU SETPOINT WITHIN IMU_THRESHOLD"

            # both IR errors are less than corner state

            if ir_top_error < 100 and ir_bottom_error < 100:


                # turn top and bottom IR PID control back on
                ir_bottom_pid.ignore = False
                ir_top_pid.ignore = False
                imu_wall_pid.ignore = True
                imu_corner_pid.ignore = True

                robot["state"] = 'wall_follow'
		imu_corner_pid.turns_completed += 1

            elif ir_top_error < TOP_C_MIN:
                # turn top IR PID control back on
                ir_bottom_pid.ignore = True
                ir_top_pid.ignore = False
                imu_wall_pid.ignore = True     # may not want to use imu_pid to do wall-following
                imu_corner_pid.ignore = True
                print "Using top ir sensor for wall follow"

    else:
        print "Entered default case in state machine."

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

    return steering_cmd

# Callbacks for recording data from IMU
def madgwick_callback(data, args):
    imu_wall_pid = args[0]
    imu_corner_pid = args[1]
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

        #for _ in range(NUM_READINGS):
        #    ir_bottom_callback(ir_bottom.get_position(), ir_bottom_pid)
        #    ir_top_callback(ir_top.get_position(), ir_top_pid)

        # Initialize subscriber for IMU
        madgwick_sub = rospy.Subscriber("imu/data_madgwick",
                                        Imu,
                                        madgwick_callback,
                                        [imu_wall_pid,imu_corner_pid])

        # Send setpoints to PIDs
        # Wait for recorded IMU data before publishing setpoint
        rospy.sleep(0.25)
        imu_corner_pid.imu_setpoint()
        imu_wall_pid.imu_setpoint(imu_corner_pid.setpoint.data)
        ir_bottom_pid.ir_setpoint()
        ir_top_pid.ir_setpoint()

        # Set zero intial velocity and steering
        motor_srv(CENTER)
        steering_srv(CENTER)

        #motor.set_target(CENTER)
        #steering.set_target(CENTER)
        rospy.sleep(1)

        #motor_srv(6400)
        #rospy.sleep(1)

        # Set forward speed
        motor_srv(6250)
        print "MOTOR SPEED: ", MOTOR_SPEED
        #motor.set_target(MOTOR_SPEED)

        # Iteration rate
        rate = rospy.Rate(RATE)

        # Initialize stateMachine()
        robot = {"state": "wall_follow"}
        imu_wall_pid.ignore = True
        imu_corner_pid.ignore = True
	imu_corner_pid.turns_completed = 0
        time_since_turn = rospy.get_time()

        # Count iterations: can be used for debugging or other miscellaneous needs
        count = 0
        while not rospy.is_shutdown():

            #for _ in range(NUM_READINGS):
            #    ir_bottom_callback(ir_bottom.get_position(), ir_bottom_pid)
            #    ir_top_callback(ir_top.get_position(), ir_top_pid)

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
            steering_cmd = DOPEStateMachine(robot, ir_bottom_pid, ir_top_pid, imu_wall_pid, imu_corner_pid)
            #steering_cmd = kodiesStateMachine1(robot, ir_bottom_pid, ir_top_pid, imu_wall_pid, imu_corner_pid)

            # Set steering target
            steering_cmd += CENTER
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
    rospy.init_node('odroid_node1', anonymous=True)

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
    TOP_CORNER_ERROR_THRESHOLD = rospy.get_param('~top_corner_error_threshold')
    TOP_C_MIN = 100
    TOP_C_MAX = 500
    BOTTOM_C_MIN = 700
    TOP_D_MIN = 500
    BOTTOM_D_MIN = 90
    BOTTOM_D_MAX = 700



    # redefine DOOR and CORNER thresholds
    #DOOR_THRESHOLD = 150
    #CORNER_THRESHOLD = 600
    #IMU_THRESHOLD = math.radians(20)

    try:
        odroid()
    except rospy.ROSInterruptException:
        pass
