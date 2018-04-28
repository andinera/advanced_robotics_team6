#!/usr/bin/env python

import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
import cv2
from keras.preprocessing.image import ImageDataGenerator
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D
from keras.layers import Activation, Dropout, Flatten, Dense
import numpy as np
from time import time
from threading import Event

from sensor_msgs.msg import Image
from std_msgs.msg import Bool

# Class for performing object recognition on images of stop signs
class StopSign:
    # Object initialization method
    def __init__(self):

	self.event = Event()
	self.event.set()
        self.bridge = CvBridge()
        # Manage the frequency object recognition is performed
        self.timer = time()
        self.frequency = 1
        # Model for predicting whether a stop sign exists in the passed image
        # Image size
        self.img_width = 640
        self.img_height = 480
        input_shape = (self.img_height, self.img_width, 3)
        # Layers chosen based on passed model size
        self.model = Sequential()
        self.model.add(Conv2D(32, (3, 3), input_shape=input_shape))
        self.model.add(Activation('relu'))
        self.model.add(MaxPooling2D(pool_size=(2, 2)))

        self.model.add(Conv2D(32, (3, 3)))
        self.model.add(Activation('relu'))
        self.model.add(MaxPooling2D(pool_size=(2, 2)))

        self.model.add(Conv2D(64, (3, 3)))
        self.model.add(Activation('relu'))
        self.model.add(MaxPooling2D(pool_size=(2, 2)))

        self.model.add(Flatten())
        self.model.add(Dense(64))
        self.model.add(Activation('relu'))
        self.model.add(Dropout(0.5))
        self.model.add(Dense(1))
        self.model.add(Activation('sigmoid'))
        # Load previously trained model weights from file
        try:
            r = rospkg.RosPack()
            weights_file = r.get_path("advanced_robotics_team6")+"/data/stop_sign/stop_sign_3c.h5"
            self.model.load_weights(weights_file)
        except IOError, e:
            print e
        # Make model ready to be used
        self.model.compile(loss='binary_crossentropy',
                      optimizer='rmsprop',
                      metrics=['accuracy'])

        # Publishes object recognition prediction
        self.stop_sign_pub = rospy.Publisher('stop_sign_detector',
                                             Bool,
                                             queue_size=1)
        # Subscribes to image stream
        self.image_sub = rospy.Subscriber('camera/image_rect',
                                          Image,
                                          self.image_callback)

    # Handle images received from camera
    def image_callback(self, msg):
        # Perform prediction per the set frequency
        if self.timer <= time() or self.event.isSet():
	    self.event.clear()
            self.timer += self.frequency
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
            print prediction
            self.event.set()
            # Publish prediction
            # if prediction >= 0.5:
            #     print "Stop sign detected."
            #     self.stop_sign_pub.publish(True)
            # else:
            #     print "No stop sign detected."
            #     self.stop_sign_pub.publish(False)

if __name__ == '__main__':
    rospy.init_node('stop_sign_node', anonymous=True)
    StopSign()
    rospy.spin()
