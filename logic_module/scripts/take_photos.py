#! /usr/bin/env python2
# -*- coding: utf-8 -*-

# Author: ziqi han
# Create: 05/20/2021
# Latest Modify: 04/27/2023
# Description: Take photos to build dataset
# This file is used in 16th National University Students Intelligent Car Race

import numpy as np
import time
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
i = 0


def callback(data):
    global i
    # Convert the format of the picture in ROS
    # to the format in Opencv through cvBridge
    bridge = CvBridge()
    cv2.imwrite(
        '/home/ucar/data/desk/%d.jpg' % i,
        bridge.imgmsg_to_cv2(data, "bgr8"))
    i = i + 1
    rospy.sleep(1)


def aruco_demo():
    # create node
    rospy.init_node('ucar_cam', anonymous=True)

    # Subscribe to the image message sent by usb_cam,
    # and enter the callback function callback() after receiving the message
    rospy.Subscriber('usb_cam/image_raw', Image, callback)

    rospy.spin()


if __name__ == '__main__':
    aruco_demo()
