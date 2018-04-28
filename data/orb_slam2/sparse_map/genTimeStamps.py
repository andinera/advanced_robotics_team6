#!/usr/bin/env python

import os
import math
import numpy as np

data = np.loadtxt("/home/carl/Packages/ORB_SLAM2/Examples/Monocular/EuRoC_TimeStamps/MH01.txt")
# data = np.loadtxt("./timestamps/test.txt")

onlyfiles = next(os.walk("./LAB01"))[2] #dir is your directory path as string
print len(onlyfiles)

text_out = open("./timestamps/LAB01.txt", 'a')

for i in range(1,len(onlyfiles)):
    text_out.write("{}\n".format(data[i]))

text_out.close()
