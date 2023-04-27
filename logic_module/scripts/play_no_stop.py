#! /usr/bin/env python2
# -*- coding: utf-8 -*-

# Author: ziqi han
# Create: 03/25/2022
# Latest Modify: 04/27/2023
# Description: Logic of the project, Direct the car from starting point to end point
# This file is used in 17th National University Students Intelligent Car Race

from numpy.core.numeric import moveaxis
import rospy
from darknet_ros_msgs.msg import classes
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import cv2
import os


# ===========================================initialization===========================================
PARKING_ID = 2  # 1,2,3

# ---------------------------Set the QR code target point-----------------------------------
target_qrcode = Pose(Point(2.609, 1.150, 0.000), Quaternion(0.000, 0.000, 0.996, 0.0927))

# turn transition point
# Position(2.116, 1.730, 0.000), Orientation(0.000, 0.000, 0.006, 1.000)
target_temp = Pose(Point(2.116, 1.730, 0.000), Quaternion(0.000, 0.000, 0.006, 1.000))

# ---------------------------Set the parking position target point-----------------------------------
# D1
if PARKING_ID == 1:
    # Position(1.217, 2.702, 0.000), Orientation(0.000, 0.000, 0.714, 0.700)
    # Pose(Point(1.217, 2.702, 0.000), Quaternion(0.000, 0.000, 0.714, 0.700))
    target_parking = Pose(Point(1.25, 2.75, 0.000), Quaternion(0.000, 0.000, 0.714, 0.700))
# D2
elif PARKING_ID == 2:
    target_parking = Pose(Point(0.75, 2.65, 0.000), Quaternion(0.000, 0.000, 0.7, 0.714))
# D3
elif PARKING_ID == 3:
    target_parking = Pose(Point(0.324, 2.65, 0.000), Quaternion(0.000, 0.000, 0.726, 0.687))


pos_flag = 0
"""
0: start position
1: Arrive at the identification position in Zone 7
2: Arrive at the identification position in Zone 6
3: Zone 6 transition point
4: Arrive at the QR code recognition position
5: Arrive at Area C
6: The transfer in area C is completed
7: reach the end
"""

reached_temp_flag = False
reached_end_flag = False
broadcast_finished_flag = False


bridge = CvBridge()


def goto_point(point):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose = point
    goal_pub.publish(goal_pose)


def mic_call_back(data):
    # go to turn transition point
    goto_point(target_temp)


def classes_callback(msg):
    return


def goal_callback(msg):
    global pos_flag
    if msg.status.status == 3:
        rospy.sleep(0.5)
        pos_flag += 1


def img_callback(data):
    # define picture to_down' coefficient of ratio
    global bridge, pos_flag, reached_temp_flag, broadcast_finished_flag
    global reached_end_flag

# -------------------Arrive at the temporary location---------------------------------------
    if pos_flag == 1 and not reached_temp_flag:
        # Go to the QR code
        goto_point(target_qrcode)
        reached_temp_flag = True

# -------------------Arrive at the QR code recognition point---------------------------------------
    elif pos_flag == 2 and not broadcast_finished_flag:
        print('reached qrcode position......')
        cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
        aruco_img = cv2.flip(cv_img, 1)
        cv2.imwrite(
            '/home/ucar/ucar_ws/src/logic_module/qr_detection/qr_code.jpg', aruco_img)
        print('written qrcode into local file......')
        # publish QR message
        startmsg = Int8()
        startmsg.data = 1
        reached_qr_pub.publish(startmsg)
        # Go to the third character identification point
        rospy.sleep(1)
        goto_point(target_parking)
        broadcast_finished_flag = True

# -------------------reach destination---------------------------------------
    elif pos_flag == 3 and not reached_end_flag:
        rospy.loginfo("reached END!!!!!!!!!!!!!!!!!!!!!!!!")

        # broadcast person
        person_num = 2
        longhair_num = 1
        glasses_num = 1
        os.system("play /home/ucar/ucar_ws/src/logic_module/wav/model_%d.wav" % person_num)
        # broadcast longhair
        os.system("play /home/ucar/ucar_ws/src/logic_module/wav/longhair_%d.wav" % longhair_num)
        # broadcast glasses
        os.system("play /home/ucar/ucar_ws/src/logic_module/wav/glasses_%d.wav" % glasses_num)
        os.system("play /home/ucar/ucar_ws/src/logic_module/wav/tts_sample_4.wav")
        rospy.loginfo('+++++++++++++++++++++++++++++++++++++++')
        rospy.loginfo('glasses_num: %d, longhair_num: %d' % (glasses_num, longhair_num))
        rospy.loginfo('+++++++++++++++++++++++++++++++++++++++')

        reached_end_flag = True
    else:
        pass


if __name__ == '__main__':
    rospy.init_node('play', anonymous=True)
    rospy.Subscriber("/start_others", Int8, mic_call_back)
    rospy.Subscriber("move_base/result", MoveBaseActionResult, goal_callback)
    rospy.Subscriber("/darknet_ros/classes_num", classes, classes_callback)
    rospy.Subscriber('usb_cam/image_raw', Image, img_callback)
    goal_pub = rospy.Publisher(
        '/move_base_simple/goal', PoseStamped, queue_size=1)
    img_pub = rospy.Publisher('/final_image', Image, queue_size=1)
    reached_qr_pub = rospy.Publisher('/reached_qr', Int8, queue_size=1)
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.spin()
