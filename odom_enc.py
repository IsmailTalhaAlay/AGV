#!/usr/bin/env python

import rospy
import tf
import time
from threading import current_thread
from math import sin, cos

from geometry_msgs.msg import Twist, Vector3, Quaternion                # Message libraries
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

# Vehicle Parameters

wheelbase = 0.73

"""initial Pose"""

th = 0
x = 0
y = 0
encoder1= 0
encoder2 =0 

def odom_callback(encoders):
    global previous_time, th, x, y

    rospy.loginfo("Received encoder data message!")
    rospy.loginfo("Linear Components: [%f,%f]"%(encoders.x,encoders.y))
    current_time = rospy.Time.now()
    
    """compute robot odometry"""
    vL= encoders.x
    vR = encoders.y
    vL= (encoders.x)*1000
    vR = (encoders.y)*1000
    vL = round(vL)/1000
    vR = round(vR)/1000
    vx = (vL + vR) / 2 
    vy = 0              #kinematic constraint
    vth = (vL - vR) / wheelbase
    

    vx= (vx)*1000
    vth = (vth)*1000 
    vx = round(vx)/1000
    vth = round(vth)/1000

    #if (abs(vth) < 0.005):
   #     vth = 0
    #else: vth = vth


    dt = current_time.to_sec() - previous_time.to_sec()
    #dt =1
    dx = (vx * cos(th) - vy * sin(th)) * dt
    dy = (vx * sin(th) - vy * cos(th)) * dt
    dth = vth * dt
    x += dx
    y += dy
    th += dth





    odom_quat = tf.transformations.quaternion_from_euler(0,0,-th) #convert euler to quat since Odom msg uses quat

    """publish transform message"""

    """translation_vector = (x, y, 0)
    odom_broadcaster.sendTransform(
        translation_vector,
        odom_quat,
        current_time,
        "childbase_link",
        "odomen") """
    translation_vector2 = (-x, -y, 0)    #qy, -qx, 0 
    odom_broadcaster.sendTransform(
        translation_vector2,
        odom_quat,
        current_time,
        "odomenc",
        "base_link")
    
    """publish odom message"""

    odomenc = Odometry()
    odomenc.header.stamp = current_time
    odomenc.header.frame_id = "odomenc"

    odomenc.pose.pose.position.x = x
    odomenc.pose.pose.position.y = y
    odomenc.pose.pose.position.z = 0
    odomenc.pose.pose.orientation = Quaternion(*odom_quat) #odom_quat is a numpy array which uses index but odom.pose.pose.orientation is a msg class
    
    odomenc.child_frame_id = "childbase_link"
    odomenc.twist.twist.linear.x = vx
    odomenc.twist.twist.linear.y = vy
    odomenc.twist.twist.angular.z = vth
    
    odom_publisher.publish(odomenc) 

    previous_time = current_time

if __name__ == '__main__':
  
    rospy.init_node('odom_encoder', anonymous=True)                             # Node initiation. Node name: odom_encoder
    odom_broadcaster = tf.TransformBroadcaster() 
    odom_publisher = rospy.Publisher("odomenc", Odometry, queue_size=3)            # Publisher definition. Topic name: odom, Message type: Odometry
    previous_time = rospy.Time.now()
    rospy.Subscriber("encoders", Vector3 , odom_callback)                       # Subscriber definition. Topic name: encoders, Message type: Vector3, odom_callback function
    rospy.spin()



