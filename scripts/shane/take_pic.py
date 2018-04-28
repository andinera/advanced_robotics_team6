#!/usr/bin/env python


import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
from time import time


class TakePic:

    def __init__(self):
        self.bridge = CvBridge()
        self.id = 0
        self.timer = time()
        # Define your image topic
        image_topic = "/camera/image_rect"
        # Set up your subscriber and define its callback
        rospy.Subscriber(image_topic, Image, self.image_callback)
        # Spin until ctrl + c
        rospy.spin()

    def image_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        if self.timer <= time():
            # print("Saved an image!")
            self.timer = time()
            # Save your OpenCV2 image as a jpeg
            file_name = 'LAB01_{}.jpeg'.format(self.id)
            self.id += 1
            cv2.imwrite(file_name, cv2_img)
            print self.id



if __name__ == '__main__':
    rospy.init_node('image_listener')
    TakePic()
