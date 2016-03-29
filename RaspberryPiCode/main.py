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

# targeting dots
targetingDots = [(165, 187), (143, 162), (143, 134)]

#Lower and upper bounds for the H, S, V values respectively
minHue = 55
minSaturation = 75
minValue = 138

maxHue = 92
maxSaturation = 255
maxValue = 255

minBoxArea = 650

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
    ip = "10.12.61.2"
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

    rect = cv2.minAreaRect(cnt)
    box = sorted(cv2.boxPoints(rect), key = itemgetter(1))
    box_area = np.linalg.norm(box[0] - box[1]) * np.linalg.norm(box[0] - box[2])

    if hull_area > 0 and box_area > minBoxArea:
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
    # return max(0, 0.0103009 * area)
    return max(0, 0.5 * 0.01545135 * area)
def calculateYOffset(area):
    # return min(0, 0.0196113 * area - 81.217)
    # return min(0, 0.00841481 * area - 48.79153378)
    return min(0, 0.00841481 * area - 63.70994870)

def drawTargetingDots(image):
    for point in targetingDots:
        cv2.circle(image, point, 5, (0, 0, 255), cv2.FILLED)

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
        boxArea = np.linalg.norm(sortedBoxPoints[0] - sortedBoxPoints[1]) * np.linalg.norm(sortedBoxPoints[0] - sortedBoxPoints[2])
	
        dist1 = np.linalg.norm(sortedBoxPoints[0] - sortedBoxPoints[1])
        dist2 = np.linalg.norm(sortedBoxPoints[0] - sortedBoxPoints[2])

        if dist1 >= dist2:
            centerPoint = (sortedBoxPoints[0] + sortedBoxPoints[1]) / 2
        else:
	    centerPoint = (sortedBoxPoints[0] + sortedBoxPoints[2]) / 2

        box = np.int0(box)
        
        #center of the contour
        centerPoint = tuple(centerPoint)
        
        area = cv2.contourArea(bestContour)
        
        offset = (calculateXOffset(boxArea), calculateYOffset(boxArea))
        print str(offset[0]) + ',' + str(offset[1]) + ',' + str(boxArea)
        
        targetPoint = tuple(map(int, map(sum, zip(centerPoint, offset))))
        estimatedShotPoint = tuple(map(int, map(lambda (x, y): x - y, zip((width/2, height/2), offset))))

        # Draws the rotated rectangle in blue
        cv2.drawContours(image, [box], 0, (255, 0, 0), 2)
        # cv2.drawContours(image, contours, 0, (255, 255, 0), 2) # TODO: remove
        #draws point in middle of contour in yellow
        cv2.circle(image, centerPoint, 5, (0, 255, 255), cv2.FILLED)
        
        #draws estimatedShotPoint in Orange
        cv2.circle(image, estimatedShotPoint, 5, (0, 180, 255), cv2.FILLED)
        
        # print "(%d, %d)" % targetPoint
        sd.putNumber("x", targetPoint[0])
        sd.putNumber("y", targetPoint[1])
        sd.putNumber("area", boxArea)
        sd.putBoolean("contourFound", True)
    else:
        # print "no contours"
        sd.putBoolean("contourFound", False)
        
    # Center Cirle in Yellow
    cv2.circle(image, (width/2, height/2), 10, (0, 0, 255), 1)

    drawTargetingDots(image)

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

