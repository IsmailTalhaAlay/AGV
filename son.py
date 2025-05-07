from picamera.array import PiRGBArray
from picamera import PiCamera
import sys
import cv2
import numpy as np
import time
import serial

### Global Variables ###

RESOLUTION = (320, 240)

CENTER = (
    RESOLUTION[0] // 2,
    RESOLUTION[1] // 2
)

BRIGHTNESS = (10)

AREA_THRESHOLD = 750

CASCADE_PATH =  "/home/pi/Desktop/codes/resources/haarcascade_frontalface_default.xml"

pan_angle = 30   
tilt_angle = 30

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



def ColorDetect(img):
    imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    imgMask = cv2.inRange(imgHSV,lower,upper)
    imgResult = cv2.bitwise_and(img,img,mask=imgMask)
    contours,hierarchy = cv2.findContours(imgMask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    if len(contours) > 0:
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > AREA_THRESHOLD:
                cv2.drawContours(imgContour,cnt,-1,(255,0,0))
                cv2.drawContours(imgMask,cnt,-1,(255,0,0))
                peri = cv2.arcLength(cnt,True) 
                approx = cv2.approxPolyDP(cnt,0.02*peri,True)
                #print(peri,approx)
                x, y, w, h = cv2.boundingRect(approx)
                cv2.rectangle(imgContour,(x,y),(x+w,y+h),(255,0,0),5)
                x_center = x + w//2
                y_center = y + h//2
                cv2.line(imgContour,(CENTER[0],CENTER[1]),(x_center,y_center),(0,255,0),3)
                return (x_center,y_center)
    return (CENTER[0],CENTER[1])
            

                
                
    #cv2.imshow("HSV Video",imgHSV)
    cv2.imshow("Mask",imgMask)
    #cv2.imshow("Result",imgResult)




def FaceDetect(img):
    faceCascade = cv2.CascadeClassifier(CASCADE_PATH)
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(imgGray,1.1,4, minSize = (30,30))
    if len(faces) > 0:
        (x, y, w, h) = faces[0]
        cv2.rectangle(imgContour,(x,y),(x+w,y+h),(255,0,0),5)
        x_center = x + w//2
        y_center = y + h//2
        cv2.line(imgContour,(CENTER[0],CENTER[1]),(x_center,y_center),(0,255,0),3)
        return (x_center, y_center)
    return (CENTER[0], CENTER[1])

### Take user input ###

inp0 = input("Face or Color? \n")
if inp0 == "f":
    b1 = False
    b2 = True
elif inp0 == "c":
    b1 = True
    b2 = False
    inp1 = input("BGR or HSV? \n")
    if inp1 == "b":
        b, g, r = input("B: \n"), input("G: \n"), input("R: \n")
        lower, upper = hsvBound(b,g,r)
    elif inp1 == "h":
        t = input("Enter HSV values: \n")
        a = tuple(int(x) for x in t.split())
        print(a)
        lower = np.array([a[0],a[2],a[4]])
        lower = lower.astype(np.int16)
        upper = np.array([a[1],a[3],a[5]])
        upper = upper.astype(np.int16)
        print(lower,upper)        
            
def PanAngle(x):
    global pan_angle
    if x < CENTER[0] - 10:
        pan_angle -= int(0.1*np.abs(CENTER[0]-x))
        if pan_angle < 0:
            pan_angle = 0
    if x > CENTER[0] + 10:
        pan_angle += int(0.1*np.abs(CENTER[0]-x))
        if pan_angle > 180:
            pan_angle = 180
    return pan_angle

def TiltAngle(y):
    global tilt_angle
    if y < CENTER[1] - 10:
        tilt_angle -= int(0.1*np.abs(CENTER[1]-y))
        if tilt_angle < 0:
            tilt_angle = 0
    if y > CENTER[1] + 10:
        tilt_angle += int(0.1*np.abs(CENTER[1]-y))
        if tilt_angle > 90:
            tilt_angle = 90
    return tilt_angle


### Ready video frame for capture and initialize PID###

camera = PiCamera()
camera.resolution = (RESOLUTION[0],RESOLUTION[1])
camera.framerate = 32
camera.vflip = True
rawCapture = PiRGBArray(camera, size=(RESOLUTION[0],RESOLUTION[1]))
time.sleep(0.1)


### Main loop ###

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.flush()
    
    img = frame.array
   
    imgContour = img.copy()

    if b1:
        (x_center, y_center) = ColorDetect(img)
    elif b2:
        (x_center, y_center) = FaceDetect(img)

    pan_angle = PanAngle(x_center)
    tilt_angle = TiltAngle(y_center)
    print(pan_angle,tilt_angle)

    sendStr = str(tilt_angle) + "t" + "," + str(pan_angle) + "p" + ","
    ser.write(sendStr.encode('utf-8'))
    time.sleep(0.05)

    cv2.imshow("Original Video",img)
    cv2.imshow("Contour",imgContour)
    
    key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
