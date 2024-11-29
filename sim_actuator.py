#!/usr/bin/env python

from cmath import pi
from pickle import TRUE
import serial
import time, math
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

def callback2(cmd_vel):

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


    encoder1 = 40
    encoder2 = 80


    print("encoder value right[rpm]",encoder1) 
    print("encoder value left[rpm]",encoder2)

    pub = rospy.Publisher("encoders", Vector3, queue_size=1)                    # Publisher definition. Topic name: encoders, Message type: Vector3
    pub.publish(encoder1,encoder2,0)                                            # Publish encoder data
    rospy.loginfo("Transmitted encoder data : %f , %f",encoder1,encoder2) 

def speed_control_rev():

    rospy.init_node('actuator',anonymous = True)                                # Node initiation. Node name: actuator
    rospy.Subscriber("cmd_vel", Twist, callback2, queue_size=1)                 # Subscriber definition. Topic name: cmd_vel, Message type: Twist. callback2 function
    rospy.spin()
    time.sleep(0.1)

if __name__ == '__main__':

    speed_control_rev()
    rate = rospy.Rate(1)

    
