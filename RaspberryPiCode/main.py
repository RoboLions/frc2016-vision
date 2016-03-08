#!/usr/bin/env python

#Author: THE GREAT PAPYRUS (Nyeh heh heh)
import time
import cv2
import numpy as np
import sys
import time
import timeit
from networktables import NetworkTable
import subprocess
from operator import itemgetter

# resolution of the webcam
width, height = 320, 240

#Lower and upper bounds for the H, S, V values respectively
minHue = 60
minSaturation = 100
minValue = 60

maxHue = 100
maxSaturation = 255
maxValue = 255

lowerHSV = np.array([minHue, minSaturation, minValue])
upperHSV = np.array([maxHue, maxSaturation, maxValue])

# The index of the contours in the tuple returned from the findContours method
contourIndex = 1


# This subproccess runs the command, basically setting the exposure so it's not automatic, drastically reducing the performance.
subprocess.call(["v4l2-ctl", "-d" ,"/dev/video0", "-c", "exposure_auto=1", "-c", "exposure_absolute=10"]) # exposure_absolute sets the exposure
# This subprocess runs the command, to run the mjpg streamer for driver station
subprocess.Popen(["mjpg_streamer", "-i", "/usr/local/lib/input_file.so -f . -n video.jpg -r", "-o", "/usr/local/lib/output_http.so -w /usr/local/www -p 1180"])

if len(sys.argv) < 2:
    print("Error: specify an IP to connect to!")
    print("Going with default: ")
    ip = "roboRIO-1262-FRC.local"
    print(ip)
else:
    ip = sys.argv[1]

NetworkTable.setIPAddress(ip)
NetworkTable.setClientMode()
NetworkTable.initialize()

sd = NetworkTable.getTable("RaspberryPI")

sd.putNumber("imageSizeX", width)
sd.putNumber("imageSizeY", height)


# sets the video feed to cap, short for capture
cap = cv2.VideoCapture(0);

# The 3 and 4 are majick numbers that are determine the parameter being changed is width and height, respectively
cap.set(3, width) # Changes the width of the feed
cap.set(4, height) # Changes the height of the feed

def getBestContour(cntrs):
    ''' This method takes in a list of contours and returns the best contour to track, based on solidity. If no contour is present it returns None '''
    if len(cntrs) == 0:
        return None
    
    cnt = cntrs[0]
    area = cv2.contourArea(cnt)
    hull = cv2.convexHull(cnt)
    hull_area = cv2.contourArea(hull)
    
    if hull_area > 0:
        targetSolidity = 0.35
        solidityTolerance = 0.1

        minSolidity = targetSolidity - solidityTolerance
        maxSolidity = targetSolidity + solidityTolerance
        
        solidity = float(area)/hull_area
        
        if minSolidity < solidity and solidity < maxSolidity:
            return cnt

    return None
    
# GOT this calcualtions from GOOGLE DRIVE: https://docs.google.com/a/prhsrobotics.com/spreadsheets/d/1j2z3Uly7T2C6El34SFLA19RGCt04RwKtTRwmxWzCv3Q/edit?usp=sharing
# Takes in the area of a contour and calculates the x and y offsets
def calculateXOffset(area):
    return max(0, 0.052 * area - 17.496)
def calculateYOffset(area):
    return min(0, 0.099*area - 81.217)    

def main():
    # captureSuccess is true if the camera was read properly
    # image is the image read from the camera
    captureSuccess, image = cap.read()
    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Creates a image Threshold based on the lower and upper bounds on HSV values
    imgThresh = cv2.inRange(hsv, lowerHSV, upperHSV)

    #imgThresh = cv2.GaussianBlur(imgThresh, (3,3), 2)
    #imgThresh = cv2.dilate(imgThresh, np.ones((5,5),np.uint8))
    #imgThresh = cv2.erode(imgThresh, np.ones((5,5),np.uint8))

    # We really don't need to use im2 and hierarchy variables, just ignore these for now
    contours = cv2.findContours(imgThresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[contourIndex]
    
    #sorts the contour from bigges to smallest
    sortedContours = sorted(contours, key = cv2.contourArea, reverse=True)

    bestContour = getBestContour(sortedContours)

    if bestContour != None:
        #x, y = findCenterXY(cnt)
        rect = cv2.minAreaRect(bestContour)

        # Finds the points of the minimum rotated rectangle
        # Box is just a list of points (numpy.ndarray)
        box = cv2.boxPoints(rect)
        
        # A list of the Box points sorted by lowest y values, to find the top line of the box
        sortedBoxPoints = sorted(box, key = itemgetter(1))
        
        box = np.int0(box)
        
        #center of the contour
        centerPoint = (sortedBoxPoints[0] + sortedBoxPoints[1]) / 2
        centerPoint = tuple(centerPoint)
        
        area = cv2.contourArea(bestContour)
        
        offset = (calculateXOffset(area), calculateYOffset(area))
        
        targetPoint = tuple(map(int, map(sum, zip(centerPoint, offset))))

        # Draws the rotated rectangle in blue
        cv2.drawContours(image, [box], 0, (255, 0, 0), 2)
        #draws point in middle of contour in yellow
        cv2.circle(image, centerPoint, 5, (0, 255, 255), cv2.FILLED)
        
        #draws targetPoint in Orange
        cv2.circle(image, targetPoint, 5, (0, 180, 255), cv2.FILLED)
        
        

        print "(%d, %d)" % targetPoint
        sd.putNumber("x", targetPoint[0])
        sd.putNumber("y", targetPoint[1])
        sd.putNumber("area", area)
        sd.putBoolean("contourFound", True)
    else:
        print "no contours"
        sd.putBoolean("contourFound", False)
        
    # Center Cirle in Yellow
    cv2.circle(image, (width/2, height/2), 10, (0, 0, 255), 1)




    # show the frame
    # cv2.imshow("Frame", image)

    # writes the frames to a file to be read by the mjpeg streamer
    cv2.imwrite("/home/pi/Desktop/frc2016-vision/RaspberryPiCode/video.jpg", image)
    
while True:
    main()
    key = cv2.waitKey(1) & 0xFF

    # if the `esc` key was pressed, break from the loop
    if key == 27:
    	break

