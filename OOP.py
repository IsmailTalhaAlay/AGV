from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2 as cv
import numpy as np
import time
from timeit import default_timer as timer
import serial

pan_angle = 0
tilt_angle = 30


class Tracking():

    def __init__(self):
        #cap = cv.VideoCapture(0,cv.CAP_DSHOW)
        self.whT = 416
        self.confThreshold = 0.5
        self.nmsThreshold = 0.2
        self.flag = True

        self.prev_frame_time = 0
        self.new_frame_time = 0

        #### LOAD MODEL
        ## Coco Names
        self.classNames = ["Drone"]

        ## Model Files
        modelConfiguration = "yolov3_custom.cfg"
        modelWeights = "yolov3_custom_final_old.weights"

        net = cv.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
        net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)
        camera = PiCamera()

        camera.resolution = (640,480)

        #camera.framerate = 32

        camera.vflip = True

        rawCapture = PiRGBArray(camera,size=(640,480))

        ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

        ser.flush()

        time.sleep(0.1)
        

        self.tracker = cv.TrackerCSRT_create()

        #### Ana Döngü
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            img = frame.array

            #img = cv.flip(img, 1)

            #img = cv.resize(img, (1280,720),interpolation=cv.INTER_AREA)

            #FPS Counter
            self.new_frame_time = time.time()
            fps = 1 / (self.new_frame_time - self.prev_frame_time)
            self.prev_frame_time = self.new_frame_time

            hT, wT, cT = img.shape

            self.center = (320,240)

            # Locking Rectangle
            area_y = int(hT / 10)
            area_x = int(wT / 4)

            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            gauss = cv.GaussianBlur(gray, (3, 3), 0)
            #gauss = cv.adaptiveThreshold(gauss,255,cv.ADAPTIVE_THRESH_MEAN_C,cv.THRESH_BINARY,17,7)

            if self.flag:
                print("Finding")
                blob = cv.dnn.blobFromImage(img, 1 / 255, (self.whT, self.whT), [0, 0, 0], 1, crop=False)
                net.setInput(blob)
                layersNames = net.getLayerNames()
                outputNames = [(layersNames[i[0] - 1]) for i in net.getUnconnectedOutLayers()]
                outputs = net.forward(outputNames)
                coord = self.findObjects(outputs, img, hT, wT, cT, area_y, area_x)
                print(coord)

                if coord == None:
                    pass
                else:

                    ### Calculating time
                    #start = timer()

                    ok = self.tracker.init(img, (coord[0], coord[1], coord[2], coord[3]))
                    print((coord[0], coord[1], coord[2], coord[3]))
                    print(ok)
                    cv.putText(img, "+", (coord[0] + coord[2], coord[1] + coord[3]), cv.FONT_HERSHEY_SIMPLEX, 1,
                               (255, 0, 255), 3)
                    self.flag = False

            else:
                print("Tracking")
                ok, bbox = self.tracker.update(gauss)
                print(bbox)

                if ok:
                    p1 = (int(bbox[0]), int(bbox[1]))
                    p2 = (int(bbox[0] + bbox[2]),
                          int(bbox[1] + bbox[3]))
                    cv.rectangle(img, p1, p2, (0, 0, 255), 2, 2)

                    center_target = ((bbox[0] + int(bbox[2]/2)),(bbox[1]+int(bbox[3]/2)))
                    x_center = center_target[0]
                    y_center = center_target[1]
                    pan_angle = self.PanAngle(x_center,0.05)
                    tilt_angle = self.TiltAngle(y_center,0.05)
                    print("Servo Angles: ",pan_angle,tilt_angle)
                    sendStr = str(tilt_angle) + "t" + "," + str(pan_angle) + "p" + ","
                    ser.write(sendStr.encode('utf-8'))
                    
                    

                    #cv.putText(img, "+",center_target, cv.FONT_HERSHEY_SIMPLEX, 1,
                               #(255, 0, 255), 3)

                    if area_x + bbox[2] / 2 <= bbox[0] + bbox[2] / 2 <= wT - area_x - bbox[2] / 2 and area_y + bbox[
                        3] / 2 <= bbox[1] + bbox[3] / 2 <= hT - area_y - bbox[3] / 2:
                        cv.putText(img, "Target Detected!", (10, 40), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                    ### Calculating Time
                    #end = timer()
                    #track_time = end - start

                    #if track_time > 5:
                    #    self.flag = True

                else:
                    self.flag = True

            #cv.putText(img, "FPS : " + str(fps), (100, 50), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

            cv.rectangle(img, (area_x, area_y), (wT - area_x, hT - area_y), (255, 0, 255), 3)
            cv.putText(img, "+", (self.center[0], self.center[1]), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 1)

            cv.imshow('Image', img)
            cv.imshow('Gauss',gauss)
            
            
            rawCapture.truncate(0)
            k = cv.waitKey(24) & 0xff
            if k == 27:
                break
        rawCapture.truncate(0)
        cap.release()
        cv.destroyAllWindows()

    def detected(self):
        pass

    def findObjects(self, outputs, img, hT, wT, cT, area_y, area_x):
        bbox = []
        classIds = []
        confs = []

        for output in outputs:
            for det in output:
                scores = det[5:]
                classId = np.argmax(scores)
                confidence = scores[classId]
                if confidence > self.confThreshold:
                    w, h = int(det[2] * wT), int(det[3] * hT)
                    x, y = int((det[0] * wT) - w / 2), int((det[1] * hT) - h / 2)
                    bbox.append([x, y, w, h])
                    classIds.append(classId)
                    confs.append(float(confidence))

        indices = cv.dnn.NMSBoxes(bbox, confs, self.confThreshold, self.nmsThreshold)

        for i in indices:
            i = i[0]
            box = bbox[i]
            x, y, w, h = box[0], box[1], box[2], box[3]

            cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)

            if area_x + w / 2 <= x + w / 2 <= wT - area_x - w / 2 and area_y + h / 2 <= y + h / 2 <= hT - area_y - h / 2:
                cv.putText(img, "Target Detected!", (10, 40), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        try:
            return (x, y, w, h)
        except UnboundLocalError:
            pass
    def PanAngle(self,x,gain):

        global pan_angle

        if x < 320 - 10: #Threshold for center

            pan_angle -= int(gain*np.abs(320-x)) #Control speed of servo panning proportional to error

            if pan_angle < 0:

                pan_angle = 0

        if x > 320 + 10:

            pan_angle += int(gain*np.abs(320-x))       

            if pan_angle > 180:

                pan_angle = 180

        return pan_angle

    def TiltAngle(self,y,gain):

        global tilt_angle

        if y < 240 - 10:

            tilt_angle -= int(gain*np.abs(240-y))

            if tilt_angle < 0:

                tilt_angle = 0

        if y > 240 + 10:

            tilt_angle += int(gain*np.abs(240-y))  

            if tilt_angle > 120:

                tilt_angle = 120

        return tilt_angle

tracking = Tracking()

