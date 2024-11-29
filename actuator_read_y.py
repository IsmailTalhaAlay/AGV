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

#driver
servo_x_addr0 = 3
servo_x_addr1 = 2
instrument0 = minimalmodbus.Instrument("/dev/ttyUSB0", servo_x_addr0) #3 

instrument1 = minimalmodbus.Instrument("/dev/ttyUSB1", servo_x_addr1) #4

instrument0.serial.baudrate = 115200
instrument0.serial.bytesize = 8
instrument0.serial.parity   = serial.PARITY_NONE
instrument0.serial.stopbits = 2
instrument0.serial.timeout  = 0.2
instrument0.mode = minimalmodbus.MODE_ASCII

instrument1.serial.baudrate = 115200
instrument1.serial.bytesize = 8
instrument1.serial.parity   = serial.PARITY_NONE
instrument1.serial.stopbits = 2
instrument1.serial.timeout  = 0.2
instrument1.mode = minimalmodbus.MODE_ASCII



def callback3(cmd_vel):

    global previous_time, x, y, th, encoder1, encoder2

    encoder2 = instrument0.read_register(18, 0, 3, True)
    encoder1 = instrument1.read_register(18, 0, 3, True)   
                   

    
    print(encoder2)
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

    
