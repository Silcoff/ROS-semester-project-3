from turtle import width
import cv2 as cv
import numpy as np
import pyrealsense2 as rs
from realsense_depth import *
import matplotlib.pyplot as plt
import math

self.upperThresh = 70
self.lowerThresh = 40
self.dc = DepthCamera()
self.Height  = 300
self.Width   = 500
self.FOV = [69,42] 
self.margin = 60

def find_hand_callback(self):
    ret, depth_frame, img = self.dc.get_frame()
    
    dCrop = depth_frame[self.margin:depth_frame.shape[0]-self.margin, self.margin:depth_frame.shape[1]-self.margin]

    img = img[self.margin:img.shape[0]-self.margin, self.margin:img.shape[1]-self.margin]
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) #converts an image to grayscale
    gray = cv.GaussianBlur(gray, (3,3), cv.BORDER_DEFAULT)

    canny = cv.Canny(gray, self.lowerThresh, self.upperThresh)

    kSize = 100
    kernel = np.ones((kSize, kSize), np.uint8)

    closeThresh = cv.erode(cv.dilate(canny,kernel),kernel)


    distTrans= cv.distanceTransform(closeThresh, cv.DIST_L2, 3)

    cv.normalize(distTrans, distTrans, 0, 1.0, cv.NORM_MINMAX)

    final = cv.cvtColor(distTrans, cv.COLOR_GRAY2BGR)

    maxX = 0
    maxY = 0
    maxVal = 0
    for y in range(self.Height):
        for x in range(self.Width):
            if distTrans[y,x] != 0 and distTrans[y,x] > maxVal:
                maxVal = distTrans[y,x]
                maxX = x
                maxY = y
    
    angX = (maxX * (self.FOV[0]/self.Width)) - self.FOV[0]/2
    angY = (maxY * (self.FOV[1]/self.Height)) - self.FOV[1]/2
    d = dCrop[maxY,maxX]

    xPos = math.sin(math.radians(angX))*d
    yPos = math.sin(math.radians(angY))*d
    zPos = d
    # zPos = math.sqrt(xPos*xPos + yPos*yPos + d*d)
