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

xo = 0
yo = 0
tho =  0
xa= 0.55
ya = 0


def get_rotation (subor):
    global previous_time, qxr, qyr, qzr, qlvxr, qlvyr, qlvzr, qavxr, qavyr, qavzr, xo, yo, tho, qrroll, qrpitch, qryaw, xa, ya

    rospy.loginfo("Received msckf data message!")
    current_time = rospy.Time.now()

    orientation_qr = subor.pose.pose.orientation
    orientationr_list = [orientation_qr.x, orientation_qr.y, orientation_qr.z, orientation_qr.w]
    (qrroll, qrpitch, qryaw) = euler_from_quaternion (orientationr_list)

    pos_qr = subor.pose.pose.position
    [qxr, qyr, qzr] = [pos_qr.x, pos_qr.y, pos_qr.z]

    twistlinear_qr = subor.twist.twist.linear
    [qlvxr, qlvyr, qlvzr] = [twistlinear_qr.x, twistlinear_qr.y, twistlinear_qr.z]

    twistangular_qr = subor.twist.twist.angular
    [qavxr, qavyr, qavzr] = [twistangular_qr.x, twistangular_qr.y, twistangular_qr.z]

    wo = qavzr
    phi =(90-qryaw)

    va_yp = qlvyr-wo*0.55*sin(phi)
    va_xp = qlvxr+wo*0.55*cos(phi)

    va_y = +wo*0.55*sin(phi)
    va_x = -wo*0.55*cos(phi)

    dto = current_time.to_sec() - previous_time.to_sec()
    dta_x = (va_xp+va_x)*dto
    dta_y = (va_yp+va_y)*dto

    xa += dta_x
    ya += dta_y

    xo = xa-0.55*cos(qryaw)
    yo = ya-0.55*sin(qryaw)

    tho = qryaw

    vtho = qavzr
    vxo = -va_x*cos(tho)
    vyo = va_x*sin(tho)

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, tho)

    translation_vector = (xo, yo, 0)    #qy, -qx, 0
    odom_broadcaster.sendTransform(
        translation_vector,
        odom_quat,
        current_time,
        "base_link",
        "odom")

    """publish odom message"""

    ######

    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link" 

    odom.pose.pose.position.x = xo
    odom.pose.pose.position.y = yo
    odom.pose.pose.position.z = 0

    odom.pose.pose.orientation = Quaternion(*odom_quat)

    odom.child_frame_id = "base_link"
    odom.twist.twist.linear.x = vxo
    odom.twist.twist.linear.y = vyo
    odom.twist.twist.angular.z = vtho

    odom_publisher.publish(odom)

    previous_time = current_time

if __name__ == '__main__':
     
    rospy.init_node('odom_11', anonymous=True)                             # Node initiation. Node name: odom_encoder
    odom_broadcaster = tf.TransformBroadcaster() 
    #odom_broadcaster2 = tf.TransformBroadcaster()
    odom_publisher = rospy.Publisher("odom", Odometry, queue_size=50) 
    previous_time = rospy.Time.now()
    #sub = rospy.Subscriber ('/agv/vio/odom', Odometry, get_rotation)                      # Subscriber definition. Topic name: encoders, Message type: Vector3, odom_callback function
    subor = rospy.Subscriber ('/odom_right', Odometry, get_rotation) 
    rospy.Rate(100)
    rospy.spin()
