#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Author: ziqi han
# Create: 04/13/2021
# Latest Modify: 04/27/2023
# Description: Detect Aruco Code, determine assigned task
# This file is used in 16th National University Students Intelligent Car Race

import rospy
from darknet_ros_msgs.msg import classes

from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import cv2
import os
import os
from threading import main_thread
import cv2
import cv2.aruco as aruco

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()
qr_finished_flag = False

# ========================================Play QR code Sound===========================================


def play_QR_code(QR_code_id):
    id_str = str(QR_code_id)
    dish_id = int(id_str[2])+1  # 1,2,3
    os.system("play /home/ucar/ucar_ws/src/logic_module/wav/dish_%d.wav" % dish_id)


def img_callback(data):
    global qr_finished_flag
    if not qr_finished_flag:
        aruco_img = cv2.imread('/home/ucar/ucar_ws/src/logic_module/qr_detection/qr_code.jpg')
        _, QR_code_id, _ = aruco.detectMarkers(aruco_img, aruco_dict, parameters=parameters)

        # If a QR code is detected
        if QR_code_id is not None:
            print('detected QR by raw image!.......')
            print(QR_code_id)
            # Play QR code recognition content
            play_QR_code(QR_code_id)
            qr_finished_flag = True
            print('finished playing wav!.......')
        else:
            print("================= not detected ================")
        # # If no QR code is detected: histogram equalization
        # else:
        #     gray = cv2.cvtColor(aruco_img, cv2.COLOR_BGR2GRAY)
        #     equ = cv2.equalizeHist(gray)
        #     cv2.imwrite('/home/ucar/ucar_ws/src/logic_module/qr_detection/qr_code_equ.jpg', equ)
        #     corners, QR_code_id, rejectedImgPoints = aruco.detectMarkers(equ,aruco_dict,parameters=parameters)
        #     if QR_code_id is not None:
        #         print('detected QR by equalized image!')
        #         print(QR_code_id)


if __name__ == '__main__':
    rospy.init_node('qr_code', anonymous=True)
    rospy.Subscriber('/reached_qr', Int8, img_callback)
    rospy.spin()
