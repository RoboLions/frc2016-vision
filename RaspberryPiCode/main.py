#!/usr/bin/env python

#Author: THE GREAT PAPYRUS (Nyeh heh heh)
import time
import cv2
import numpy as np
import sys
import time
import timeit
import os
import re
from networktables import NetworkTable
import subprocess

# camera USB port number
# (you can find it by running lsusb -t)
cameraUSBPort = 5

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

cameraPath = "/dev/v4l/by-path/platform-3f980000.usb-usb-0:1.{0}:1.0-video-index0".format(cameraUSBPort)

if not os.path.exists(cameraPath):
    print("Fatal error: no camera found at USB port {0}!".format(cameraUSBPort))
    print("({0} does not exist)".format(cameraPath))
    sys.exit(1)

cameraPath = os.path.realpath(cameraPath)

try:
    match = re.match(r'/dev/video(\d+)', cameraPath)
    if match:
        cameraId = int(match.group(1))
    else:
        raise ValueError()
except ValueError:
    print("Fatal error: could not extract camera ID from camera path")
    print("(From camera at USB port {0}, got camera path: {1})".format(cameraUSBPort, cameraPath))

print("Using camera at USB port {0} ({1}, camera ID {2})".format(cameraUSBPort, cameraPath, cameraId))

# This subproccess runs the command, basically setting the exposure so it's not automatic, drastically reducing the performance.
# subprocess.call(["v4l2-ctl", "-d" , cameraPath, "-c", "exposure_auto=1", "-c", "exposure_absolute=10"]) # exposure_absolute sets the exposure
# This subprocess runs the command, to run the mjpg streamer for driver station
subprocess.Popen(["mjpg_streamer", "-i", "/usr/local/lib/input_file.so -f /home/pi/Desktop/frc2016-vision/RaspberryPiCode/ -n video.jpg -r", "-o", "/usr/local/lib/output_http.so -w /usr/local/www -p 1180"])


if len(sys.argv) < 2:
    print("Error: specify an IP to connect to!")
    print("Going with default: ")
    ip = "roboRIO-1261-FRC.local"
    print(ip)
else:
    ip = sys.argv[1]

print("Connecting to %s" % ip)

NetworkTable.setIPAddress(ip)
NetworkTable.setClientMode()
NetworkTable.initialize()

sd = NetworkTable.getTable("RaspberryPI")

sd.putNumber("imageSizeX", width)
sd.putNumber("imageSizeY", height)

# sets the video feed to cap, short for capture
cap = cv2.VideoCapture(cameraId)

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

def main():
    # captureSuccess is true if the camera was read properly
    # image is the image read from the camera
    captureSuccess, image = cap.read()
    #print captureSuccess    
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

        # Finds the lengths of the two sides of the rectangle
        dist1 = np.linalg.norm(box[1] - box[2])
        dist2 = np.linalg.norm(box[2] - box[3])

        # Compares which distance is greater, and then assigns the mid point of the longer side as the targetPoint
        if dist1 >= dist2:
            targetPoint = (box[1] + box[2]) / 2
        else:
            targetPoint = (box[2] + box[3]) / 2
        
        box = np.int0(box)

        targetPoint = tuple(targetPoint)

        # Draws the rotated rectangle in blue
        cv2.drawContours(image, [box], 0, (255, 0, 0), 2)
        #draws point in middle of contour in Red
        cv2.circle(image, targetPoint, 5, (0, 0, 255), cv2.FILLED)

        print "(%d, %d)" % targetPoint
        sd.putNumber("x", targetPoint[0])
        sd.putNumber("y", targetPoint[1])
        sd.putNumber("area", cv2.contourArea(bestContour))
        sd.putBoolean("contourFound", True)
    else:
        print "no contours"
        sd.putBoolean("contourFound", False)



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

