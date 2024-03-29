# import the necessary packages
#from picamera.array import PiRGBArray
#from picamera import PiCamera
import time
import cv2
import numpy as np
import sys
import time
import timeit
from networktables import NetworkTable
import subprocess

subprocess.call(["v4l2-ctl", "-d" ,"/dev/video1", "-c", "exposure_auto=1", "-c", "exposure_absolute=50"])

if len(sys.argv) < 2:
    print("Error: specify an IP to connect to!")
    print("Going with default: ")
    ip = "roboRIO-1262-FRC.local"
    print(ip)
else:
    ip = sys.argv[1]

#NetworkTable.setIPAddress(ip)
#NetworkTable.setClientMode()
#NetworkTable.initialize()

#sd = NetworkTable.getTable("RaspberryPI")

#resolution of the webcam
#width, height = 640, 360
#width, height = 427, 240
width, height = 320, 240 #3.76315808296 #FPS: 26.5734252443
#width, height = 256, 144 #3.61019802094 #FPS: 27.6993116222

# allow the camera to warmup
#time.sleep(0.1)
cap = cv2.VideoCapture(1);
cap.set(3, width)
cap.set(4, height)

#sd.putNumber("imageSizeX", width)
#sd.putNumber("imageSizeY", height)

def findCenterXY(cnt):
        '''returns center x, y values'''
        M = cv2.moments(cnt)
        x = int(M['m10']/M['m00'])
        y = int(M['m01']/M['m00'])

        return x, y


lower = np.array([84, 110, 50])
upper = np.array([131, 255, 255])

# capture frames from the camera
#for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
def main():
    _, image = cap.read()
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #Lower and upper bounds for the H, S, V values respectivley
    imgThresh = cv2.inRange(hsv, lower, upper)

    imgThresh = cv2.GaussianBlur(imgThresh, (3,3), 2)
    imgThresh = cv2.dilate(imgThresh, np.ones((5,5),np.uint8))
    imgThresh = cv2.erode(imgThresh, np.ones((5,5),np.uint8))

    _,contours, _ = cv2.findContours(imgThresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #find biggest contour
    cntrs = sorted(contours, key = cv2.contourArea, reverse=True)
    if len(cntrs) > 0:
        cnt = cntrs[0]
        #print cv2.contourArea(cnt)
        if cv2.contourArea(cnt) > 10:
            x, y = findCenterXY(cnt)
            cv2.drawContours(image, cnt, -1, (0, 255, 0), 1)
            cv2.circle(image, (x, y), 5, (255, 255,0), cv2.FILLED) #draws point in middle of contour
            print "x: ", x , "y: ", y
            #sd.putNumber("x", x)
            #sd.putNumber("y", y)
            #sd.putNumber("area", cv2.contourArea(cnt))
            #sd.putBoolean("contourFound", True)
    #else:
        #sd.putBoolean("contourFound", False)
        #print "no contours"

    # show the frame
    #cv2.imshow("Frame", image)
    cv2.imwrite("/tmp/stream/cv.jpg", image)

while True:
    main()
    key = cv2.waitKey(1) & 0xFF

    # if the `esc` key was pressed, break from the loop
    if key == 27:
    	break

