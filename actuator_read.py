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

instrument = minimalmodbus.Instrument("/dev/driver2", servo_x_addr0) #driver2
instrument.serial.baudrate = 115200
instrument.serial.bytesize = 7
instrument.serial.parity   = serial.PARITY_EVEN
instrument.serial.stopbits = 1
instrument.serial.timeout  = 0.5 #0.05
instrument.mode = minimalmodbus.MODE_ASCII
instrument.clear_buffers_before_each_transaction = True
instrument.close_port_after_each_call = True

instrument1 = minimalmodbus.Instrument("/dev/driver1", servo_x_addr1) #driver1
instrument1.serial.baudrate = 115200
instrument1.serial.bytesize = 7
instrument1.serial.parity   = serial.PARITY_EVEN
instrument1.serial.stopbits = 1
instrument1.serial.timeout  = 0.5 #0.05
instrument1.mode = minimalmodbus.MODE_ASCII




global previous_time, x, y, th, encoder1, encoder2


def speed_control_rev():
    encoder1 = 0
    encoder2 = 0
    pub = rospy.Publisher("encoders", Vector3, queue_size=1)                    # Publisher definition. Topic name: encoders, Message type: Vector3
                                       
    rospy.loginfo("Transmitted encoder data : %f , %f",encoder1,encoder2) 
    rospy.init_node('actuator',anonymous = True)                                # Node initiation. Node name: actuator
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        encoder1 = instrument.read_register(18, 0, 3, True)
        encoder2 = instrument1.read_register(18, 0, 3, True)
        encoder1 = ((encoder1)*2*pi*0.1)/600/20
        encoder2 = ((encoder2)*2*pi*0.1)/600/20
        print(encoder2)
        print(encoder1)
        pub.publish(encoder1,encoder2,0) 
        rate.sleep()

        

if __name__ == '__main__':

    speed_control_rev()
    

    
