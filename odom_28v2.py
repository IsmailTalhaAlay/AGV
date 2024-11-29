#!/usr/bin/env python3


import rospy
import tf
import time
from threading import current_thread
from math import sin, cos

from geometry_msgs.msg import Twist, Vector3, Quaternion                # Message libraries
from tf.transformations import euler_from_quaternion, quaternion_from_euler


from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

roll = pitch = yaw = 0
def get_rotation (sub):
    global qroll, qpitch, qyaw, qr_pitch, qr_roll, qr_qyaw,  rx, ry, rz, rw, qx, qy, qz, qlvx, qlvy, qlvz, qavx, qavy, qavz, x, y

    rospy.loginfo("Received msckf data message!")
    current_time = rospy.Time.now()

    orientation_q = sub.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (qroll, qpitch, qyaw) = euler_from_quaternion (orientation_list)

    #qr_roll = qpitch
    #qr_pitch = -qroll+110

    #qr_pitch = -qroll
    
    #qr_roll = -qpitch
    #qr_qyaw = qyaw 

    qr_pitch = qpitch
    qr_roll = qroll
    qr_qyaw = qyaw
 

    orientationright = quaternion_from_euler(qr_roll, qr_pitch, qr_qyaw)

    rx = orientationright[0]
    ry = orientationright[1]
    rz = orientationright[2]
    rw = orientationright[3]

    position_q = sub.pose.pose.position
    [qx, qy, qz] = [position_q.x, position_q.y, position_q.z]

    twistlinear_q = sub.twist.twist.linear
    [qlvx, qlvy, qlvz] = [twistlinear_q.x, twistlinear_q.y, twistlinear_q.z]

    twistangular_q = sub.twist.twist.angular
    [qavx, qavy, qavz] = [twistangular_q.x, twistangular_q.y, twistangular_q.z]



    odom_quat = tf.transformations.quaternion_from_euler(0, 0, qyaw)
    #odom_quat2 = tf.transformations.quaternion_from_euler(0, 0, 0)

    """compute robot odometry""" 

    #odom_quat = tf.transformations.quaternion_from_euler(0,0,qr_qyaw) #convert euler to quat since Odom msg uses quat

    """publish transform message"""
    """
    if vx != 0:
        translation_vector = (x, y, 0)
    else:
        translation_vector = (0, 0, 0)
    """
    
    """translation_vector2 = (0.55*cos(qr_qyaw), 0.55*sin(qr_qyaw), 0.53)    #qy, -qx, 0 
    odom_broadcaster2.sendTransform(
        translation_vector2,
        odom_quat2,
        current_time,
        "odom_right",
        "odom")   
    """ 

    """publish odom message"""

    ######


    odom_right = Odometry()
    odom_right.header.stamp = current_time
    odom_right.header.frame_id = "odom_right"

    odom_right.pose.pose.position.x = -qy  #qy
    odom_right.pose.pose.position.y = qx  #-qx
    odom_right.pose.pose.position.z = 0
    #odom_right.pose.covariance = covtw
    
    odom_right.pose.pose.orientation = Quaternion(*odom_quat) #odom_quat is a numpy array which uses index but odom.pose.pose.orientation is a msg class
    
    odom_right.child_frame_id = "msckfright_link" #msckfright_link

    odom_right.twist.twist.linear.x = qlvy
    odom_right.twist.twist.linear.y = -qlvx
    odom_right.twist.twist.linear.z = 0


    odom_publisher.publish(odom_right)


if __name__ == '__main__':
     
    rospy.init_node('odom_12', anonymous=True)                             # Node initiation. Node name: odom_encoder
    odom_broadcaster = tf.TransformBroadcaster() 
    #odom_broadcaster2 = tf.TransformBroadcaster()
    odom_publisher = rospy.Publisher("odom_right", Odometry, queue_size=50) 
    sub = rospy.Subscriber ('/agv/vio/odom', Odometry, get_rotation)                      # Subscriber definition. Topic name: encoders, Message type: Vector3, odom_callback function
    rospy.Rate(100)
    rospy.spin()

