#!/usr/bin/env python


import rospy
import tf
import time
from threading import current_thread
from math import sin, cos

from geometry_msgs.msg import Twist, Vector3, Quaternion                # Message libraries
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32



def odom_zero():
    th=0
    current_time = rospy.Time.now()
    pub = rospy.Publisher("odom", Odometry, queue_size=3)
    rospy.init_node('odom_zero', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom_quat = tf.transformations.quaternion_from_euler(0,0,th) #convert euler to quat since Odom msg uses quat
    odom.pose.pose.position.x = 0
    odom.pose.pose.position.y = 0
    odom.pose.pose.position.z = 0
    odom.pose.pose.orientation = Quaternion(*odom_quat) #odom_quat is a numpy array which uses index but odom.pose.pose.orientation is a msg class
    
    odom.child_frame_id = "base_link"
    odom.twist.twist.linear.x = 0
    odom.twist.twist.linear.y = 0
    odom.twist.twist.angular.z = 0

if __name__ == '__main__':
    try:
        odom_zero()
    except rospy.ROSInterruptException:
        pass