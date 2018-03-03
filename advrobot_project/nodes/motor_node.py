#!/usr/bin/env python

####################
# Range [4095, 7905]
####################

import rospy
import serial
from std_msgs.msg import Int64

def set_speed(data, usb):
    print data.data
    low = int("{0:b}".format(data.data)[-7:], base=2)
    high = int("{0:b}".format(data.data)[0:-7], base=2)
    usb.write(chr(0x84)+chr(0x01)+chr(low)+chr(high))

def motor(usb):
    pub = rospy.Publisher('motor/get_speed', Int64, queue_size=1)
    while not rospy.is_shutdown():
        sub = rospy.Subscriber('motor/set_speed', Int64, set_speed, usb)
        usb.write(chr(0x90)+chr(0x01))
        least = ord(usb.read())
        most = ord(usb.read())
        least = "{0:08b}".format(least)
        most = "{0:08b}".format(most)
        speed = "{}{}".format(most, least)
        position = int(speed, base=2)
        pub.publish(position)
        rospy.sleep(0.1)

# Main function
if __name__ == '__main__':
    rospy.init_node('motor')
    # Setup serial connection with Pololu
    usb = serial.Serial('/dev/ttyACM0')
    motor(usb)
    usb.close()             # Close connection with Pololu
