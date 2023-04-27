#! /usr/bin/env python2
# -*- coding: utf-8 -*-

# Author: ziqi han
# Create: 03/20/2021
# Latest Modify: 04/27/2023
# Description: Detect Aruco Tag
# This file is used in 16th National University Students Intelligent Car Race

import numpy as np
import time
import cv2
import cv2.aruco as aruco
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
pub = rospy.Publisher('marker_id', String, queue_size=1)


def callback(data):
    # Convert the format of the picture in ROS
    # to the format in Opencv through cvBridge
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, "bgr8")

    # Axisymmetrically flip the photo,
    # otherwise the QR code is mirrored
    frame = cv2.flip(frame, 1)

    # Convert the image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Load the aruco dictionary, this game uses the 4x4 aruco code
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)

    # Create aruco detection parameters, the default is enough
    parameters = aruco.DetectorParameters_create()

    # Detect the corner information of aruco code
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # If an aruco code is detected, output its number
    if ids is not None:
        print(ids)
    pub.publish(ids)
    # Draw the outer frame of the aruco code
    aruco.drawDetectedMarkers(frame, corners, ids)
    cv2.imshow("frame", frame)

    key = cv2.waitKey(1)


def aruco_demo():
    rospy.init_node('ucar_cam', anonymous=True)

    rospy.Subscriber('usb_cam/image_raw', Image, callback)

    rospy.spin()


if __name__ == '__main__':
    aruco_demo()
