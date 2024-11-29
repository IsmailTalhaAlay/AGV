#!/usr/bin/env python

from cmath import pi
from pickle import TRUE
import serial
import time, math
import minimalmodbus                # Modbus Communication library
import numpy as np
import rospy                                            

from math import ceil, sin, cos
from threading import current_thread
from geometry_msgs.msg import Twist, Vector3, Quaternion        # Message types
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32

# vehicle parameters 

PI=3.1415
wheel_diameter = 0.1    # all terms in meter
wheel_track = 0.73
wheel_env=wheel_diameter*PI
wheel_base=0.73

# transmission parameters 

gear_ratio = 20

# motor characteristics

max_speed=31.41
max_freq=50000

# odom initial values

x = 0
y = 0
th = 0

"""
---Permission Denied solution---
sudo su
cd /
cd dev
chown amr ttyUSB0
"""

# Modbus Communication Parameters

servo_x_addr2 = 3
servo_x_addr1 = 2

instrument = minimalmodbus.Instrument("/dev/ttyUSB0", servo_x_addr2)  #3 
instrument1 = minimalmodbus.Instrument("/dev/ttyUSB1", servo_x_addr1) #4

instrument.serial.baudrate = 115200
instrument.serial.bytesize = 8
instrument.serial.parity   = serial.PARITY_NONE
instrument.serial.stopbits = 2
instrument.serial.timeout  = 0.2
instrument.serial.write_timeout  = 2.0
instrument.mode = minimalmodbus.MODE_ASCII
instrument.handle_local_echo = None
#instrument.clear_buffers_before_each_transaction = True
#instrument.close_port_after_each_call = True

instrument1.serial.baudrate = 115200
instrument1.serial.bytesize = 8
instrument1.serial.parity   = serial.PARITY_NONE
instrument1.serial.stopbits = 2
instrument1.serial.timeout  = 0.2
instrument1.serial.write_timeout  = 2.0
instrument1.mode = minimalmodbus.MODE_ASCII

son_write = instrument.write_register( 532, 257, 0, 6, False)          # Servo NO On DI1     P2-10
son_write = instrument1.write_register( 532, 257, 0, 6, False)          # Servo NO On DI1     P2-10
#son_read = instrument.read_register(532, 0, 3, False)
#print("Servo ON : "+str(son_read))

spd0_write = instrument.write_register( 536, 276, 0, 6, False)        # SPD0 NO On   DI3     P2-12
spd0_write = instrument1.write_register( 536, 276, 0, 6, False)        # SPD0 NO On   DI3     P2-12
#spd0_read = instrument.read_register(536, 0, 3, False)
#print("SPD0 : "+str(spd0_read))


spd1_write = instrument.write_register( 538, 277, 0, 6, False)       # SPD1 NO On    DI4     P2-13
spd1_write = instrument1.write_register( 538, 277, 0, 6, False)       # SPD1 NO On    DI4     P2-13
spd1_read = instrument.read_register(538, 0, 3, False)
print("SPD1 : "+str(spd1_read))

ie_write = instrument.write_register( 780, 13, 0, 6, False)          # Internal Parameter DI1 DI3 DI4 Enable P3-06
ie_write = instrument1.write_register( 780, 13, 0, 6, False)          # Internal Parameter DI1 DI3 DI4 Enable P3-06
ie_read = instrument.read_register(780, 0, 3, False)                  
print("Internal Parameter : "+str(ie_read))

di_write = instrument.write_register( 1038, 13, 0, 6, False)           # Digital Input Enable
di_write = instrument1.write_register( 1038, 13, 0, 6, False)           # Digital Input Enable
di_read = instrument.read_register(1038, 0, 3, False)                  # Digital Input  P4-07
print("Digital Input : "+str(di_read))

def callback3(cmd_vel):

    global previous_time, x, y, th, encoder2, encoder1
    speed_send_right = 0
    speed_send_left = 0
    dir1= 0
    dir2= 0

    time.sleep(0.5)
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z))

    speed_req = cmd_vel.linear.x;                                                                   # Calculation of necessary motor speed.
    angular_speed_req = cmd_vel.angular.z;
    speed_req_left = (speed_req + wheel_track*angular_speed_req);
    speed_req_right = (speed_req - wheel_track*angular_speed_req); 
    
    speed_send_left = int((speed_req_left*max_freq*gear_ratio)/(max_speed))
    speed_send_right = int((speed_req_right*max_freq*gear_ratio)/(max_speed))
    #speed_send_left  = round(speed_send_left)
    #speed_send_right = round(speed_send_right)

    #speed_send_left = speed_send_left*1000
    #speed_send_right = speed_send_right*1000

    print("Speed Send: [%f, %f]"%(speed_send_right, speed_send_left))


    scmd_write = instrument.write_long( 278, speed_send_left, True, 3)             # S3 Speed P1-11
    #encoder2 = instrument.read_long(278, 3, True, 3)                # S3 Speed P1-11
    #print("Speed Command : "+str(encoder2))

    scmd_write1 = instrument1.write_long( 278, speed_send_right, True, 3)             # S3 Speed P1-11
    #encoder1 = instrument1.read_long(278, 3, True, 3)                # S3 Speed P1-11
    #print("Speed Command : "+str(encoder1))


def speed_control_rev():

    rospy.init_node('actuator',anonymous = True)                                # Node initiation. Node name: actuator
    rospy.Subscriber("cmd_vel", Twist, callback3, queue_size=1)
    rospy.spin()
    time.sleep(0.1)

if __name__ == '__main__':

    speed_control_rev()
    rate = rospy.Rate(1)
    
