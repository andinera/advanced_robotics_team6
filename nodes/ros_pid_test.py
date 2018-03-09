#!/usr/bin/env python

import rospy
import time
from pololu import Controller

from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Bool

# additional imports

def get_pid_control(ctrl_msg):

    pos_cmd = ctrl_msg.my_data

    # send position command to sterring servo



def get_pose(odom_msg):

    pos = odom_msg.pose.pose.position
    pos_list = [pos.x, pos.y, pos.z]

    quaternion = odom_msg.pose.pose.orientation
    quat_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

    euler_angles = np.asarray(euler_from_quaternion(quat_list))     # [radians]
    # print type(euler_angles)
    euler_angles.shape = (3,1)
    euler_list = [euler_angles[0,0], euler_angles[1,0], euler_angles[2,0]]

    # rospy.loginfo("Position: [%f %f %f]" % pos_list)    # doesn't work
    # rospy.loginfo("Position: [%f %f %f]" % (pos.x, pos.y, pos.z))
    # rospy.loginfo("Quaternion: [%f %f %f %f]" % (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    # rospy.loginfo("Euler Angles: [%f %f %f]" % (euler_angles[0,0], euler_angles[1,0], euler_angles[2,0]))


def pid_broadcaster():

    # Publisher init
    setpoint_pub = rospy.Publisher("robot/pid/steering/setpoint", Float64, queue_size=10)
    state_pub = rospy.Publisher("robot/pid/steering/state", Float64, queue_size=10)
    pidEnable_pub = rospy.Publisher,"robot/pid/sterring/enable", Bool, queue_size=10

    # Subscriber init
    controlEffort_sub = rospy.Subscriber("robot/pid/steering/control_effort", Float64, get_pid_control)

    # Node init
    rospy.init_node('broadcast', anonymous=True)

    with Controller(0) as steering, Controller(1) as motor, \
         Controller(2) as ir_bottom:

         # set zero intial velocity
         motor.set_target(CENTER)
         time.sleep(2)

         # define setpoint by averaging initial position data
         setpoint_msg = Float64()

         distance = []
         for i in range(1,50):
             distance.append(ir_bottom.get_position())
             time.sleep(.01)

         # average of initial IR sensor data
         setpoint = int(sum(distance) / float(len(distance)))
         setpoint_msg.data = setpoint
         setpoint_pub.Publish(setpoint_msg)
         print "Start Distance: ",setpoint," cm"

         # set forward speed
         motor.set_target(CENTER + 300)

        count = 0
        while not rospy.is_shutdown():

            #define state message
            state_msg = Float64()

            state_msg.data = pos


            imu_pub.publish(imu_msg)

            count += 1

            # what is this doing?
            rate.sleep()

if __name__ == '__main__':
    try:

        pid_broadcaster()

    except rospy.ROSInterruptException:
        pass
