# The following programm detects the corners in a given image 
# using FAST Algorithm for Corner Detection

import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

# Read the test image
img = cv.imread('imgy.jpg',0)

# Initiate FAST object with default values
fast = cv.FastFeatureDetector_create()

# Find and draw the keypoints
fast.setNonmaxSuppression(0)
kp = fast.detect(img,None)
img1 = cv.drawKeypoints(img, kp, None, color=(255,0,0))
# Display the output image in a file called 'output.png'
cv.imwrite('output.png',img1)