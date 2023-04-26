#! /usr/bin/env python2
# -*- coding: utf-8 -*-
import numpy as np
import time
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
i = 0
def callback(data):
    global i
    #通过cvBridge将ROS中图片的格式转换为Opencv中的格式
    bridge = CvBridge()
    cv2.imwrite(
    '/home/ucar/data/canzuo/%d.jpg'%i,
    bridge.imgmsg_to_cv2(data, "bgr8"))
    i = i + 1
    rospy.sleep(1)


def aruco_demo():
    #创立节点
    rospy.init_node('ucar_cam', anonymous=True) 
    
    #订阅usb_cam发出的图像消息，接收到消息后进入回调函数callback()
    rospy.Subscriber('usb_cam/image_raw', Image, callback)  

    #等待
    rospy.spin()                                            
 
if __name__ == '__main__':
    aruco_demo()
