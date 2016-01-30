# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

#resolution of the webcam
width, height = 640, 480
 
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (width, height)
rawCapture = PiRGBArray(camera, size=(width, height))
 
# allow the camera to warmup
time.sleep(0.1)

def findBiggestContour(contours):
        tempArea = 0
        cnt = 'no contours'
        for i in range(len(contours)):
                area = cv2.contourArea(contours[i])
                if area > tempArea:
                        tempArea = area
                        cnt = contours[i]
        return cnt


def findCenterXY(cnt):
        '''returns center x, y values'''
        M = cv2.moments(cnt)
        x = int(M['m10']/M['m00'])
        y = int(M['m01']/M['m00'])
        
        return x, y

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #Lower and upper bounds for the H, S, V values respectivley
        lowerYellow = np.array([26, 99, 154])
        upperYellow = np.array([51, 255, 255])

        imgThresh = cv2.inRange(hsv, lowerYellow, upperYellow)

        #imgThresh = cv2.GaussianBlur(imgThresh, (3,3), 2)
        #imgThresh = cv2.dilate(imgThresh, np.ones((5,5),np.uint8))
        #imgThresh = cv2.erode(imgThresh, np.ones((5,5),np.uint8))

        im2, contours, hierarchy = cv2.findContours(imgThresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
        
        cnt = findBiggestContour(contours)

        if cnt != 'no contours':
                x, y = findCenterXY(cnt)
                cv2.circle(image, (x, y), 5, (255, 0,0), cv2.FILLED) #draws point in middle of contour
                print "x: ", x , "y: ", y
        else:
                print "no contours"
                        
	# show the frame
	cv2.imshow("Frame", image)
	
	key = cv2.waitKey(1) & 0xFF
 
	# clear the stream in preparation for the next frame
	rawCapture.truncate(0)
 
	# if the `esc` key was pressed, break from the loop
	if key == 27:
		break
