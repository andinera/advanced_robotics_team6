#!/usr/bin/env python

import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from time import time
from threading import Event
from keras.models import load_model

from sensor_msgs.msg import Image
from std_msgs.msg import Bool

# Class for performing object recognition on images of stop signs
class StopSign:
    # Object initialization method
    def __init__(self):
    	self.event = Event()
    	self.event.set()
        self.bridge = CvBridge()
        # Model for predicting whether a stop sign exists in the passed image
        # Image size
        self.img_width = 640
        self.img_height = 480
        input_shape = (self.img_height, self.img_width, 3)

        try:
            r = rospkg.RosPack()
            self.model = load_model(r.get_path("advanced_robotics_team6")+"/data/stop_sign/stop_sign_holding.h5")
        except IOError, e:
            print e
        self.model.compile(loss='binary_crossentropy',
                      optimizer='rmsprop',
                      metrics=['accuracy'])

        # Publishes object recognition prediction
        self.stop_sign_pub = rospy.Publisher('stop_sign/prediction',
                                             Bool,
                                             queue_size=1)
        # Subscribes to image stream
        self.image_sub = rospy.Subscriber('camera/image_rect',
                                          Image,
                                          self.image_callback,
                                          queue_size=1,
                                          buff_size=320000000)

    # Handle images received from camera
    def image_callback(self, msg):
        # Perform prediction per the set frequency
        if self.event.isSet():
    	    self.event.clear()
            # Convert ros image to cv image
            try:
                image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError, e:
                print e
                return
            # Reshape the image to the model's specifications
            image = image.reshape(-1, self.img_height, self.img_width, 3)
            # Normalize the image
            image = np.divide(image, 255.)
            # Perform prediction
            prediction = self.model.predict(image, batch_size=1)
            # Publish prediction
            if prediction >= 0.5:
                print "Stop sign detected."
                self.stop_sign_pub.publish(True)
            else:
                print "No stop sign detected."
                self.stop_sign_pub.publish(False)
            self.event.set()

if __name__ == '__main__':
    rospy.init_node('stop_sign_node', anonymous=True)
    StopSign()
    rospy.spin()
