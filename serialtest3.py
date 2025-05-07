#!/usr/bin/env python3
import serial
import time

tilt_angle = 0
pan_angle = 0 

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.flush()
    while tilt_angle or pan_angle != 90:
        sendStr = str(tilt_angle) + "t" + "," + str(pan_angle) + "p" + ","
        ser.write(sendStr.encode('utf-8'))
        line = ser.readline().decode('utf-8').rstrip()
        time.sleep(2)
        tilt_angle += 5
        pan_angle += 5
        print(line)
    
