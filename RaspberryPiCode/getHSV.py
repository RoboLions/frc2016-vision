import cv2
import numpy as np

# move u* and l* sliders to find upper and lower end of hsv range respectively. 
# hit q to quit

cap=cv2.VideoCapture(0)

def nothing(x):
    pass

cv2.namedWindow("result")

h,s,v=100,100,100

cv2.createTrackbar('lh', 'result', 0, 255, nothing)
cv2.createTrackbar('ls', 'result', 0, 255, nothing)
cv2.createTrackbar('lv', 'result', 0, 255, nothing)

cv2.createTrackbar('uh', 'result', 0, 255, nothing)
cv2.createTrackbar('us', 'result', 0, 255, nothing)
cv2.createTrackbar('uv', 'result', 0, 255, nothing)

while True:

    _, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV )
    lh=cv2.getTrackbarPos('lh', 'result')
    ls=cv2.getTrackbarPos('ls', 'result')
    lv=cv2.getTrackbarPos('lv', 'result')

    uh=cv2.getTrackbarPos('uh', 'result')
    us=cv2.getTrackbarPos('us', 'result')
    uv=cv2.getTrackbarPos('uv', 'result')

    lower = np.array([lh,ls,lv])
    upper = np.array([uh,us,uv])

    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask = mask )

    cv2.imshow('result', result )

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break

cap.release
cv2.destroyAllWindows()
