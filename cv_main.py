#!/usr/bin/env python3
from picamera.array import PiRGBArray
from picamera import PiCamera
import sys
import cv2
import numpy as np
import time

### Global Variables ###

RESOLUTION = (320,240)

CENTER = (
    RESOLUTION[0] // 2,
    RESOLUTION[1] // 2
)

BRIGHTNESS = (10)

AREA_THRESHOLD = 750

CASCADE_PATH =  "/home/pi/Desktop/codes/resources/haarcascade_frontalface_default.xml"

### Functions ###

def hsvBound(blue,green,red):
    color = np.uint8([[[blue, green, red]]])
    hsv_color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    hue = hsv_color[0][0][0]
    h_min = hue - 10
    h_max = hue + 10
    s_min = 50
    s_max = 255
    v_min = 50
    v_max = 255
    lower = np.array([h_min,s_min,v_min])
    upper = np.array([h_max,s_max,v_max])
    return lower,upper



def ColorDetect(img,Boolean):
    if Boolean:
        imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        imgMask = cv2.inRange(imgHSV,lower,upper)
        imgResult = cv2.bitwise_and(img,img,mask=imgMask)
        contours,hierarchy = cv2.findContours(imgMask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > AREA_THRESHOLD:
                cv2.drawContours(imgContour,cnt,-1,(255,0,0))
                cv2.drawContours(imgMask,cnt,-1,(255,0,0))
                peri = cv2.arcLength(cnt,True) #Peri ve approx neden yapıyoruz?
                approx = cv2.approxPolyDP(cnt,0.02*peri,True)
                #print(peri,approx)
                
                x, y, w, h = cv2.boundingRect(approx)
                cv2.rectangle(imgContour,(x,y),(x+w,y+h),(255,0,0),5)

                x_center = x + w//2
                y_center = y + h//2
                e_x = CENTER[0] - x_center
                e_y = CENTER[1] - y_center

                cv2.line(imgContour,(CENTER[0],CENTER[1]),(x_center,y_center),(0,255,0),3)
                print(e_x,e_y)
                return e_x,e_y

                
                
        #cv2.imshow("HSV Video",imgHSV)
        cv2.imshow("Mask",imgMask)
        #cv2.imshow("Result",imgResult)



def FaceDetect(img,Boolean):
    if Boolean:
        faceCascade = cv2.CascadeClassifier(CASCADE_PATH)
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = faceCascade.detectMultiScale(imgGray,1.3,4)
        for (x,y,w,h) in faces:
            cv2.rectangle(imgContour,(x,y),(x+w,y+h),(255,0,0),5)
            x_center = x + w//2
            y_center = y + h//2
            e_x = CENTER[0] - x_center
            e_y = CENTER[1] - y_center
            cv2.line(imgContour,(CENTER[0],CENTER[1]),(x_center,y_center),(0,255,0),3)
            print(e_x,e_y)
            return e_x,e_y

### Take user input ###

inp0 = input("Face or Color? \n")
if inp0 == "Face":
    b1 = False
    b2 = True
elif inp0 == "Color":
    b1 = True
    b2 = False
    inp1 = input("BGR or HSV? \n")
    if inp1 == "BGR":
        b, g, r = input("B: \n"), input("G: \n"), input("R: \n")
        lower, upper = hsvBound(b,g,r)
    elif inp1 == "HSV":
        t = input("Enter HSV values: \n")
        a = tuple(int(x) for x in t.split())
        print(a)
        lower = np.array([a[0],a[2],a[4]])
        lower = lower.astype(np.int16)
        upper = np.array([a[1],a[3],a[5]])
        upper = upper.astype(np.int16)
        print(lower,upper)        
            
### Ready video frame for capture ###

camera = PiCamera()
camera.resolution = (RESOLUTION[0],RESOLUTION[1])
camera.framerate = 32
camera.vflip = True
rawCapture = PiRGBArray(camera, size=(RESOLUTION[0],RESOLUTION[1]))
time.sleep(0.1)

### Main loop ###

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    img = frame.array
   
    imgContour = img.copy()
    ColorDetect(img,b1)
    FaceDetect(img,b2)

    #cv2.imshow("Original Video",img)
    cv2.imshow("Contour",imgContour)
    
    key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
