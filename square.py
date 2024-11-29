#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist

def move():
    # Starts a new node
    rospy.init_node('square_trajectory', anonymous=True)
    velocity_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    vel_msg = Twist()

    lin_speed = 0.2
    ang_speed = 0.2
    lin_distance = 1
    ang_distance = math.pi/2	# 90 degrees
    delay = 1

    while not rospy.is_shutdown():

        square = input("Type True for actuation :")

        if square == True:

            current_distance = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            time.sleep(delay)
            vel_msg.linear.x = abs(lin_speed)
            velocity_publisher.publish(vel_msg)
            t0 = rospy.Time.now().to_sec()
            
            while(current_distance < lin_distance):

                velocity_publisher.publish(vel_msg)
                t1=rospy.Time.now().to_sec()
                current_distance= lin_speed*(t1-t0)

            current_distance = 0
            vel_msg.linear.x = 0
            velocity_publisher.publish(vel_msg)
            time.sleep(delay)
            vel_msg.angular.z = abs(ang_speed)
            velocity_publisher.publish(vel_msg)
            t2 = rospy.Time.now().to_sec()

            while(current_distance < ang_distance):

                velocity_publisher.publish(vel_msg)
                t3=rospy.Time.now().to_sec()
                current_distance= lin_speed*(t3-t2)

            current_distance = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            time.sleep(delay)
            vel_msg.linear.x = abs(lin_speed)
            velocity_publisher.publish(vel_msg)
            t4 = rospy.Time.now().to_sec()
            
            while(current_distance < lin_distance):

                velocity_publisher.publish(vel_msg)
                t5=rospy.Time.now().to_sec()
                current_distance= lin_speed*(t5-t4)

            current_distance = 0
            vel_msg.linear.x = 0
            velocity_publisher.publish(vel_msg)
            time.sleep(delay)
            vel_msg.angular.z = abs(ang_speed)
            velocity_publisher.publish(vel_msg)
            t6 = rospy.Time.now().to_sec()

            while(current_distance < ang_distance):

                velocity_publisher.publish(vel_msg)
                t7=rospy.Time.now().to_sec()
                current_distance= lin_speed*(t7-t6)

            current_distance = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            time.sleep(delay)
            vel_msg.linear.x = abs(lin_speed)
            velocity_publisher.publish(vel_msg)
            t8 = rospy.Time.now().to_sec()

            while(current_distance < lin_distance):

                velocity_publisher.publish(vel_msg)
                t9=rospy.Time.now().to_sec()
                current_distance= lin_speed*(t9-t8)

            current_distance = 0
            vel_msg.linear.x = 0
            velocity_publisher.publish(vel_msg)
            time.sleep(delay)
            vel_msg.angular.z = abs(ang_speed)
            velocity_publisher.publish(vel_msg)
            t10 = rospy.Time.now().to_sec()

            while(current_distance < ang_distance):

                velocity_publisher.publish(vel_msg)
                t11=rospy.Time.now().to_sec()
                current_distance= lin_speed*(t11-t10)

            current_distance = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            time.sleep(delay)
            vel_msg.linear.x = abs(lin_speed)
            velocity_publisher.publish(vel_msg)
            t12 = rospy.Time.now().to_sec()
            
            while(current_distance < lin_distance):

                velocity_publisher.publish(vel_msg)
                t13=rospy.Time.now().to_sec()
                current_distance= lin_speed*(t13-t12)

            current_distance = 0
            vel_msg.linear.x = 0
            velocity_publisher.publish(vel_msg)
            time.sleep(delay)
            vel_msg.angular.z = abs(ang_speed)
            velocity_publisher.publish(vel_msg)
            t14 = rospy.Time.now().to_sec()

            while(current_distance < ang_distance):

                velocity_publisher.publish(vel_msg)
                t15=rospy.Time.now().to_sec()
                current_distance= lin_speed*(t15-t14)

            current_distance = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)

        else: print("try again.")

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
