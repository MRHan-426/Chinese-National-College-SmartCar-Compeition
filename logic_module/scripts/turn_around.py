#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: ziqi han
# Create: 07/03/2021
# Latest Modify: 04/27/2023
# Description: Mecanum Model Turn Around
# This file is used in 16th National University Students Intelligent Car Race

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

turn_around_finished_flag = False
turn_around2_finished_flag = False


def turn_around_callback(data):
    turn_around_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    global turn_around_finished_flag
    if not turn_around_finished_flag:
        print("receive_first!")
        turn_around_finished_flag = True
        rate = rospy.Rate(50)
        rospy.sleep(0.2)
        vel_msg = Twist()
        count = 0
        while not rospy.is_shutdown():
            while (count < data):
                vel_msg.angular.z = 100
                turn_around_pub.publish(vel_msg)
                count = count + 1
                rate.sleep()
            break


def T_callback(data):
    turn_around_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    global turn_around_finished_flag
    if not turn_around_finished_flag:
        print("receive_first!")
        turn_around_finished_flag = True
        rate = rospy.Rate(50)
        rospy.sleep(0.2)
        vel_msg = Twist()
        count = 0
        while not rospy.is_shutdown():
            while (count < data):
                vel_msg.angular.z = -100
                turn_around_pub.publish(vel_msg)
                count = count + 1
                rate.sleep()
            break


if __name__ == '__main__':
    rospy.init_node('turn_around', anonymous=True)
    turn_around_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    # turn_around_callback(25)
    # rospy.sleep(1)
    T_callback(50)
