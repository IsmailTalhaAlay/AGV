#!/usr/bin/env python
import argparse
import math
import rospy
import numpy as np
import sys
import csv
from time import time
from rospy.core import is_shutdown

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField


#pub_mag = rospy.Publisher("/imu/mag",MagneticField,queue_size=10)



imu = Imu()
mag = MagneticField()

def imu_callback(imu_data):
    imu = imu_data
    imu.header.frame_id = "imu_link"
    imu.linear_acceleration.x *= -1
    imu.linear_acceleration.y *= -1
    imu_publisher.publish(imu)

    
if __name__ == "__main__":
    rospy.init_node("imu_edit", anonymous=True)
    imu_publisher = rospy.Publisher("imu/data_raw", Imu, queue_size=10)
    rospy.Subscriber("imu", Imu, imu_callback)
    rospy.spin()
