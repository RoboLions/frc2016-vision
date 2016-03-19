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

# targeting dots
targetingDots = [(165, 187), (161, 162), (143, 134)]

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

def main():
    # captureSuccess is true if the camera was read properly
    # image is the image read from the camera
    captureSuccess, image = cap.read()
    #print captureSuccess    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    cv2.circle(image, (width / 2, height / 2), 10, (0, 0, 255), 1)

    for point in targetingDots:
        cv2.circle(image, point, 5, (0, 0, 255), cv2.FILLED)

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

