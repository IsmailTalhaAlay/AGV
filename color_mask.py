from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2 
import numpy as np
import time

frameWidth = 320
frameHeight = 240

def empty(a):
    pass

camera = PiCamera()
camera.resolution = (frameWidth,frameHeight)
camera.framerate = 32
camera.vflip = True
rawCapture = PiRGBArray(camera, size=(frameWidth,frameHeight))
time.sleep(0.1)

cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars",640,240)
cv2.createTrackbar("Hue Min","TrackBars",63,179,empty)  
cv2.createTrackbar("Hue Max","TrackBars",135,179,empty)
cv2.createTrackbar("Sat Min","TrackBars",92,255,empty)
cv2.createTrackbar("Sat Max","TrackBars",255,255,empty)
cv2.createTrackbar("Val Min","TrackBars",255,255,empty)
cv2.createTrackbar("Val Max","TrackBars",255,255,empty)


for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
      
    img = frame.array

    h_min = cv2.getTrackbarPos("Hue Min","TrackBars") 
    h_max = cv2.getTrackbarPos("Hue Max","TrackBars")
    s_min = cv2.getTrackbarPos("Sat Min","TrackBars")
    s_max = cv2.getTrackbarPos("Sat Max","TrackBars")
    v_min = cv2.getTrackbarPos("Val Min","TrackBars")
    v_max = cv2.getTrackbarPos("Val Max","TrackBars")
    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])

    
    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    imgMask = cv2.inRange(imgHSV,lower,upper)
    
    cv2.imshow("Result",img)
    cv2.imshow("Mask",imgMask)

    
    key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        print(h_min,h_max,s_min,s_max,v_min,v_max)
        break
