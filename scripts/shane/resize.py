#!/usr/bin/env python

import cv2
import glob
import os

id = 1000

file_names = glob.glob("./dataset/validation/*.jpeg")
for file_name in file_names:
    image = cv2.imread(file_name)
    # # new_image = cv2.resize(image, (640, 480))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    os.remove(file_name)
    cv2.imwrite(file_name, image)
    id += 1
