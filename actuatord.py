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
max_freq=100000

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

servo_x_addr = 1

instrument = minimalmodbus.Instrument("/dev/ttyUSB1", servo_x_addr) 

instrument.serial.baudrate = 115200
instrument.serial.bytesize = 7
instrument.serial.parity   = serial.PARITY_EVEN 
instrument.serial.stopbits = 1
instrument.serial.timeout  = 0.05
instrument.mode = minimalmodbus.MODE_ASCII


def callback3(cmd_vel):

    global previous_time, x, y, th, encoder1, encoder2
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
    
    speed_send_left = ((speed_req_left*max_freq*gear_ratio)/(max_speed))/1000
    speed_send_right = ((speed_req_right*max_freq*gear_ratio)/(max_speed))/1000
    speed_send_left  = round(speed_send_left)
    speed_send_right = round(speed_send_right)

    speed_send_left = speed_send_left*1000
    speed_send_right = speed_send_right*1000

    if (speed_send_left  >0):                                                                       # Direction observation
        dir2 = 0
    else:
        dir2 = 1

    if (speed_send_right > 0):
        dir1 = 0
    else:
        dir1 = 1

    print("Speed Send: [%f, %f]"%(speed_send_right, speed_send_left))

    var_write = instrument.write_register(8182, abs(speed_send_left))           # 1st motor pulse freq.  
    #var_write = instrument.write_registers(8182, [5000])                       # 1st motor pulse freq.
    var_write = instrument.write_bit(2052,dir1,5)                               # 1st motor direction = 0 False clockwise 
    var_write = instrument.write_register(8185, abs(speed_send_right))          # 2nd motor pulse freq.
    #var_write = instrument.write_registers(8185, [10000])                      # 2nd motor pulse freq.
    var_write = instrument.write_bit(2048,dir2,5)                               # 2nd motor direction
    encoder2 = instrument.read_register(8187,   0, 3, True)                     # 1st motor encoder output                      8187  = D4091    
    encoder1 = instrument.read_register(8191, 0, 3, True)                       # 2nd motor encoder output                      8191  = D4095

                                                        # Encoder input correlation
                                                       
    #encoder1 = math.ceil(encoder1/100.0)
    #encoder1 = encoder1*10
    #encoder2 = math.ceil(encoder2/100.0)
    #encoder2 = encoder2*10

    encoder1 = ((encoder1)*2*pi*0.1)/600/20
    encoder2 = ((encoder2)*2*pi*0.1)/600/20
    #encoder1 = ((encoder1)*2*pi*0.1)/60
    #encoder2 = ((encoder2)*2*pi*0.1)/60

    

    print("encoder value right[rpm]",encoder1) 
    print("encoder value left[rpm]",encoder2)

    pub = rospy.Publisher("encoders", Vector3, queue_size=1)                    # Publisher definition. Topic name: encoders, Message type: Vector3
    pub.publish(encoder1,encoder2,0)                                            # Publish encoder data
    rospy.loginfo("Transmitted encoder data : %f , %f",encoder1,encoder2) 


def speed_control_rev():

    rospy.init_node('actuator',anonymous = True)                                # Node initiation. Node name: actuator
    rospy.Subscriber("cmd_vel", Twist, callback3, queue_size=1)                 # Subscriber definition. Topic name: cmd_vel, Message type: Twist. callback2 function
    rospy.spin()
    time.sleep(0.1)

if __name__ == '__main__':

    speed_control_rev()
    rate = rospy.Rate(1)

    