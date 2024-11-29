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

instrument = minimalmodbus.Instrument("/dev/ttyUSB0", servo_x_addr0)
instrument.serial.baudrate = 115200
instrument.serial.bytesize = 7
instrument.serial.parity   = serial.PARITY_EVEN
instrument.serial.stopbits = 1
instrument.serial.timeout  = 0.5 #0.05
instrument.mode = minimalmodbus.MODE_ASCII
instrument.clear_buffers_before_each_transaction = True
instrument.close_port_after_each_call = True

instrument1 = minimalmodbus.Instrument("/dev/ttyUSB2", servo_x_addr1)
instrument1.serial.baudrate = 115200
instrument1.serial.bytesize = 7
instrument1.serial.parity   = serial.PARITY_EVEN
instrument1.serial.stopbits = 1
instrument1.serial.timeout  = 0.5 #0.05
instrument1.mode = minimalmodbus.MODE_ASCII

global previous_time, x, y, th, encoder1, encoder2

encoder2 = instrument.read_register(18, 0, 3, True)
encoder1 = instrument.read_register(18, 0, 3, True)
print(encoder2)
print(encoder1)
#encoder2 = instrument.read_registers(18, 1, 3)
#encoder2 = instrument.read_long(18, 3, True, 3)

#encoder1 = encoder2