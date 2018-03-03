#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int64


# Read data from IR Sensor
def listen(usb):
    pub = rospy.Publisher('ir_sensor_two/get_distance', Int64, queue_size=1)
    while not rospy.is_shutdown():
        # Request position from IR Sensor in channel 3
        usb.write(chr(0x90)+chr(0x03))
        # Read position from IR Sensor
        least = ord(usb.read())
        most = ord(usb.read())
        # Convert response to a single integer
        least = "{0:08b}".format(least)
        most = "{0:08b}".format(most)
        distance = "{}{}".format(most, least)
        distance = int(distance, base=2)
        pub.publish(distance)
        rospy.sleep(0.01)

if __name__ == '__main__':
    rospy.init_node('ir_sensor_one')
    # Setup serial connection with Pololu
    usb = serial.Serial('/dev/ttyACM0')

    listen(usb)      # Simple, less precise protocol
    # compact_protocol(usb)   # Complex, more precise protocol

    usb.close()             # Close connection with Pololu
