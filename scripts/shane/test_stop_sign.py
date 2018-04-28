#!/usr/bin/env python

from keras.utils.conv_utils import convert_kernel
from keras.preprocessing.image import ImageDataGenerator
from keras.models import Sequential
from keras.layers import Conv2D, MaxPooling2D
from keras.layers import Activation, Dropout, Flatten, Dense
import cv2
import numpy as np
import glob

img_width, img_height = 640, 480

input_shape = (img_height, img_width, 3)

model = Sequential()
model.add(Conv2D(32, (3, 3), input_shape=input_shape))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Conv2D(32, (3, 3)))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Conv2D(64, (3, 3)))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))

# model.add(Conv2D(64, (3, 3)))
# model.add(Activation('relu'))
# model.add(MaxPooling2D(pool_size=(2, 2)))

# model.add(Conv2D(128, (3, 3)))
# model.add(Activation('relu'))
# model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Flatten())
model.add(Dense(64))
model.add(Activation('relu'))
model.add(Dropout(0.5))
model.add(Dense(1))
model.add(Activation('sigmoid'))

try:
    model.load_weights('stop_sign_3b.h5')
except IOError, e:
    print e

# for layer in model.layers:
#     if layer.__class__.__name__ == 'Conv2D':
#         print 'hello'
#         w, b = layer.get_weights()
#         w = convert_kernel(w)
#         layer.set_weights([w, b])
# model.save_weights('theano_weights.h5')

model.compile(loss='binary_crossentropy',
              optimizer='rmsprop',
              metrics=['accuracy'])

# file_names = glob.glob("./training/stop_sign/stop_sign_*.jpeg")
# file_names = glob.glob("./training/other/other_*.jpeg")
# file_names = glob.glob("./dataset/validation/stop_sign/stop_sign_*.png")
file_names = glob.glob("./dataset/validation/other/other_*.png")
for file_name in file_names:
    image = cv2.imread(file_name)
    image = image.reshape(-1, img_height, img_width, 3)
    image = np.divide(image, 255.)

    prediction = model.predict(image, batch_size=1)
    if prediction >= 0.5:
        print "Stop sign"
        print file_name
    else:
        print "Other"
        print file_name



# image = cv2.imread('large.jpg')
# image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# cv2.imwrite('gray_stopsign.jpg', image)
