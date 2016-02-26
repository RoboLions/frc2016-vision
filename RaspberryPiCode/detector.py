#!/usr/bin/env python

#Author: THE GREAT PAPYRUS (Nyeh heh heh)
import time
import cv2
import numpy as np
import sys
import time
import timeit
import pygame
from networktables import NetworkTable
import subprocess

# resolution of the webcam
width, height = 320, 240
display_size = (640, 480)

#Lower and upper bounds for the H, S, V values respectively
minHue = 100
minSaturation = 100
minValue = 0

maxHue = 120
maxSaturation = 255
maxValue = 255

min_area = 500

lowerHSV = np.array([minHue, minSaturation, minValue])
upperHSV = np.array([maxHue, maxSaturation, maxValue])

musicPlaying = False

# The index of the contours in the tuple returned from the findContours method
contourIndex = 1


# This subproccess runs the command, basically setting the exposure so it's not automatic, drastically reducing the performance.
subprocess.call(["v4l2-ctl", "-d" ,"/dev/video0", "-c", "exposure_auto=3"]) # exposure_absolute sets the exposure

pygame.mixer.init()
pygame.mixer.music.load("/home/pi/Music/found.wav")

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
    
    if area > min_area:
        return cnt

    return None

def main():
    global musicPlaying
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

    cv2.drawContours(image, bestContour, -1, (0, 0, 255), 2)

    if bestContour != None and not musicPlaying:
        pygame.mixer.music.play()
        musicPlaying = True
    elif bestContour == None and musicPlaying:
        pygame.mixer.music.pause()
        musicPlaying = False

    # show the frame
    display_image = cv2.resize(image, display_size)
    cv2.imshow("Frame", display_image)
    
while True:
    main()
    key = cv2.waitKey(1) & 0xFF

    # if the `esc` key was pressed, break from the loop
    if key == 27:
    	break

