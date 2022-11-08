from turtle import width
import cv2 as cv
import numpy as np
import pyrealsense2 as rs
from realsense_depth import *
import matplotlib.pyplot as plt
import math

upperThresh = 70
lowerThresh = 40

# capture = cv.VideoCapture(1)
dc = DepthCamera()

plotCount = 0
plot = np.zeros((50,3))
fig, T1 = plt.subplots()

Height  = 300
Width   = 500

FOV = [69,42] 

# def changeRes(width,height): #resizing funtion that only works on live video
#     capture.set(3,width)
#     capture.set(4,height)

margin = 60

while True:
    ret, depth_frame, img = dc.get_frame()
    
    dCrop = depth_frame[margin:depth_frame.shape[0]-margin, margin:depth_frame.shape[1]-margin]

    img = img[margin:img.shape[0]-margin, margin:img.shape[1]-margin]
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) #converts an image to grayscale
    gray = cv.GaussianBlur(gray, (3,3), cv.BORDER_DEFAULT)

    canny = cv.Canny(gray, lowerThresh, upperThresh)

    kSize = 100
    kernel = np.ones((kSize, kSize), np.uint8)

    closeThresh = cv.erode(cv.dilate(canny,kernel),kernel)


    distTrans= cv.distanceTransform(closeThresh, cv.DIST_L2, 3)

    cv.normalize(distTrans, distTrans, 0, 1.0, cv.NORM_MINMAX)

    final = cv.cvtColor(distTrans, cv.COLOR_GRAY2BGR)

    maxX = 0
    maxY = 0
    maxVal = 0
    for y in range(Height):
        for x in range(Width):
            if distTrans[y,x] != 0 and distTrans[y,x] > maxVal:
                maxVal = distTrans[y,x]
                maxX = x
                maxY = y
    
    angX = (maxX * (FOV[0]/Width)) - FOV[0]/2
    angY = (maxY * (FOV[1]/Height)) - FOV[1]/2
    d = dCrop[maxY,maxX]

    xPos = math.sin(math.radians(angX))*d
    yPos = math.sin(math.radians(angY))*d
    zPos = d
    # zPos = math.sqrt(xPos*xPos + yPos*yPos + d*d)

    if plotCount < 50:
        plot[plotCount] = (xPos, yPos, zPos)
        plotCount += 1

    if d != 0:
        cv.circle(final, (maxX,maxY), 20, (255,0,255), thickness=3)
        cv.putText(final, str([int(xPos), int(yPos), int(zPos)]), (maxX,maxY), cv.FONT_HERSHEY_PLAIN, 1, (0, 128, 0), 2)
        cv.putText(final, str([int(angX), int(angY), int(d)]), (maxX,maxY+20), cv.FONT_HERSHEY_PLAIN, 1, (0, 128, 0), 2)
    else:
        cv.circle(final, (maxX,maxY), 20, (0,0,255), thickness=3)
        cv.putText(final, "Distance Error", (maxX-60,maxY+35), cv.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 2)

    cv.imshow("Image1", final)
    # cv.imshow("Image2", canny)

    if cv.waitKey(20) & 0xFF == ord(' '): #breaks the loop when space is pressed
        break

T1.scatter(plot[:,0], plot[:,1], (plot[:,2]-200)*10)

plt.show()

cv.destroyAllWindows()