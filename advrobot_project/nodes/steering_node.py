#!/usr/bin/env python

####################
# Range [4095, 7905]
####################

import rospy
import serial
from time import sleep
from std_msgs.msg import Int64

def set_position(data, usb):
    low = int("{0:b}".format(data.data)[-7:], base=2)
    high = int("{0:b}".format(data.data)[0:-7], base=2)
    usb.write(chr(0x84)+chr(0x00)+chr(low)+chr(high))

def steer(usb):
    pub = rospy.Publisher('steer/get_position', Int64, queue_size=1)
    while not rospy.is_shutdown():
        sub = rospy.Subscriber('steer/set_position', Int64, set_position, usb)
        usb.write(chr(0x90)+chr(0x00))
        least = ord(usb.read())
        most = ord(usb.read())
        least = "{0:08b}".format(least)
        most = "{0:08b}".format(most)
        position = "{}{}".format(most, least)
        position = int(position, base=2)
        pub.publish(position)
        sleep(0.1)

# Main function
if __name__ == '__main__':
    rospy.init_node('steering')
    # Setup serial connection with Pololu
    usb = serial.Serial('/dev/ttyACM0')
    steer(usb)
    usb.close()             # Close connection with Pololu
