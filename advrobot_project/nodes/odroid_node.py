#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64
import sys
import numpy

# Motor and servo
##########################
# Input range [4095, 7905]
##########################

############### Objects #####################

class SteeringController:

    def __init__(self):
        self.pub_position = 6000
        self.sub_position = 6000
        self.pub = rospy.Publisher('steer/set_position', Int64, queue_size=1)
        # self.sub = rospy.Subscriber('steer/get_position', Int64, SteeringController.steering_position, self)
        self.rate = rospy.Rate(1)

    # @staticmethod
    # def steering_position(data, sC):
    #     sC.sub_position = data.data
    #     sC.rate.sleep()

    def set_position(self, position):
        self.pub.publish(position)
        self.pub_position = position

class MotorController:

    def __init__(self):
        self.pub_speed = 6000
        self.sub_speed = 6000
        self.pub = rospy.Publisher('motor/set_speed', Int64, queue_size=1)
        # self.sub = rospy.Subscriber('motor/get_speed', Int64, MotorController.motor_speed, self)
        self.rate = rospy.Rate(1)

    # @staticmethod
    # def motor_speed(data, mC):
    #     mC.sub_speed = data.data
    #     mC.rate.sleep()

    def set_speed(self, speed):
        self.pub.publish(speed)
        self.pub_speed = speed

    # def brake():
    #     if mC.sub_speed <= 6000:
    #         mC.publish_speed(6000)
    #     else:
    #         mC.publish_speed(4095)
    #         rospy.sleep(0.1)
    #         mC.publish_speed(6000)
    #         while True:
    #             if mC.sub_speed == 6000:
    #                 mC.publish_speed(6000)
    #                 return

class IRSensorOneController:

##################
# Peak value [650]
##################

    def __init__(self):
        self.distance = 0
        sub = rospy.Subscriber('ir_sensor_one/get_distance', Int64, IRSensorOneController.ir_distance, self)
        self.rate = rospy.Rate(10)

    @staticmethod
    def ir_distance(data, sC):
        sC.distance = data.data
        sC.rate.sleep

    def get_distance(self):
        return self.distance

class IRSensorTwoController:

##################
# Peak value [650]
##################

    def __init__(self):
        self.distance = 0
        sub = rospy.Subscriber('ir_sensor_two/get_distance', Int64, IRSensorTwoController.ir_distance, self)
        self.rate = rospy.Rate(100)

    @staticmethod
    def ir_distance(data, sC):
        sC.distance = data.data
        sC.rate.sleep()

class IMUController:

    def __init__(self):
        pass

class CameraController:

    def __init__(self):
        pass

class IMUController:

    def __init__(self):
        pass

class CameraController:

    def __init__(self):
        pass

def control(sC, mC, irOneC):
    rate = rospy.Rate(10)
    # distances = numpy.array([])
    previous_distance = 450
    while not rospy.is_shutdown():
        mC.set_speed(6000)
        distance = irOneC.get_distance()
        # distances = numpy.append(distances, distance)
        print distance
        # while distances.size > 10:
        #     distances = numpy.delete(distances, 0)
        # if distances.size == 10:
            # avg_distance = numpy.sum(distances)/10
        if distance > 500 or distance < 400:
            if distance > previous_distance:
                sC.set_position(7000)
            elif distance < previous_distance:
                sC.set_position(5000)
            else:
                sC.set_position(6000)
            print distance
        elif distance > 600:
            sC.set_position(5000)
        elif distance < 300:
            sC.set_position(7000)
        previous_distance = distance
        rate.sleep()
            # print previous_distance
            # print distances.size


#################### Main #########################

if __name__ == '__main__':
    rospy.init_node('odroid')
    sC = SteeringController()
    mC = MotorController()
    irOneC = IRSensorOneController()
    # irTwoC = IRSensorTwoController()
    # imuC = IMUController()
    # cameraC = CameraController()

    control(sC, mC, irOneC)

    # steering(sC)
    # motor(mC)
    # ir_sensor_one(irOneC)
    # ir_sensor_two(irTwoC)
    # imu(imuC)
    # camera(cameraC)
