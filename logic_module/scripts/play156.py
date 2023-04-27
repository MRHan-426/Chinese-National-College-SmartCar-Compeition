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

# ===========================================Initialization===========================================
PARKING_ID = 2  # 1,2,3

# ---------------------------Set the QR code target point-----------------------------------
target_qrcode = Pose(Point(2.609, 1.150, 0.000), Quaternion(0.000, 0.000, 0.996, 0.0927))

# ---------------------------Set the target point for person recognition-----------------------------------
target_detection1 = Pose(Point(1.910, 3.262, 0.000), Quaternion(0.000, 0.000, -0.056, 0.998))
target_detection2 = Pose(Point(1.809, 2.332, 0.000), Quaternion(0.000, 0.000, -0.706, 0.709))
# target_detection2 = Pose(Point(1.818, 2.439, 0.000), Quaternion(0.000, 0.000, -0.711, 0.703))
target_detection3 = Pose(Point(0.287, 1.176, 0.000), Quaternion(0.000, 0.000, 0.719, 0.694))
target_detection4 = Pose(Point(0.287, 1.176, 0.000), Quaternion(0.000, 0.000, 0.192, 0.981))

# Position(2.116, 1.730, 0.000), Orientation(0.000, 0.000, 0.006, 1.000)
target_temp = Pose(Point(2.116, 1.730, 0.000), Quaternion(0.000, 0.000, 0.006, 1.000))

# ---------------------------Set the parking position target point-----------------------------------
if PARKING_ID == 1:  # D1
    # Position(1.217, 2.702, 0.000), Orientation(0.000, 0.000, 0.714, 0.700)
    # Pose(Point(1.217, 2.702, 0.000), Quaternion(0.000, 0.000, 0.714, 0.700))
    target_parking = Pose(Point(1.25, 2.75, 0.000), Quaternion(0.000, 0.000, 0.714, 0.700))
elif PARKING_ID == 2:  # D2
    target_parking = Pose(Point(0.75, 2.65, 0.000), Quaternion(0.000, 0.000, 0.7, 0.714))
elif PARKING_ID == 3:  # D3
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
reached_1_flag = False
reached_2_flag = False
reached_temp_flag = False
reached_3_flag = False
reached_4_flag = False
reached_end_flag = False
broadcast_finished_flag = False
Img_1 = Image()
Img_2 = Image()
Img_3 = Image()
Img_4 = Image()


class DetectNum():
    def __init__(self):
        self.glasses_num_pic1 = 0
        self.longhair_num_pic1 = 0
        self.glasses_num_pic2 = 0
        self.longhair_num_pic2 = 0
        self.glasses_num_pic3 = 0
        self.longhair_num_pic3 = 0
        self.glasses_num_pic4 = 0
        self.longhair_num_pic4 = 0

    def get_total_glasses(self):
        return self.glasses_num_pic1 + self.glasses_num_pic2 + self.glasses_num_pic3 + self.glasses_num_pic4

    def get_total_longhair(self):
        return self.longhair_num_pic1 + self.longhair_num_pic2 + self.longhair_num_pic3 + self.longhair_num_pic4


myDetectNum = DetectNum()
bridge = CvBridge()


def goto_point(point):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose = point
    goal_pub.publish(goal_pose)


def mic_call_back(data):
    # Go to the first recognition point
    goto_point(target_temp)


def classes_callback(msg):
    global myDetectNum, pos_flag
    # if pos_flag == 1:
    #     myDetectNum.glasses_num_pic1 = msg.glass_num
    #     myDetectNum.longhair_num_pic1 = msg.long_hair_num
    #     rospy.loginfo("================================")
    #     rospy.loginfo('recognition position = 1')
    #     rospy.loginfo('glass = %d,  longhair = %d.' %(msg.glass_num, msg.long_hair_num))
    #     # rospy.loginfo(msg)
    #     rospy.loginfo("================================")

    # elif pos_flag == 2:
    #     myDetectNum.glasses_num_pic2 = msg.glass_num
    #     myDetectNum.longhair_num_pic2 = msg.long_hair_num
    #     rospy.loginfo("================================")
    #     rospy.loginfo('recognition position = 2')
    #     rospy.loginfo('glass = %d,  longhair = %d.' %(msg.glass_num, msg.long_hair_num))
    #     # rospy.loginfo(msg)
    #     rospy.loginfo("================================")

    if pos_flag == 5:
        myDetectNum.glasses_num_pic3 = msg.glass_cut_num
        myDetectNum.longhair_num_pic3 = msg.long_hair_cut_num
        rospy.loginfo("================================")
        rospy.loginfo('recognition position = 3')
        rospy.loginfo('glass = %d,  longhair = %d.' %
                      (msg.glass_cut_num, msg.long_hair_cut_num))
        # rospy.loginfo(msg)
        rospy.loginfo("================================")
    elif pos_flag == 6:
        myDetectNum.glasses_num_pic4 = msg.glass_num
        myDetectNum.longhair_num_pic4 = msg.long_hair_num
        rospy.loginfo("================================")
        rospy.loginfo('recognition position = 4')
        rospy.loginfo('glass = %d,  longhair = %d.' %
                      (msg.glass_num, msg.long_hair_num))
        # rospy.loginfo(msg)
        rospy.loginfo("================================")


def goal_callback(msg):
    global pos_flag
    if msg.status.status == 3:
        if pos_flag == 0:
            pos_flag += 3
        elif pos_flag == 5:
            rospy.sleep(2)
            pos_flag += 1
        elif pos_flag != 7:
            rospy.sleep(0.5)
            pos_flag += 1


def img_callback(data):
    # define picture to_down' coefficient of ratio
    global bridge, pos_flag, reached_temp_flag, Img_1, Img_2, Img_3, Img_4, broadcast_finished_flag
    global reached_1_flag, reached_2_flag, reached_3_flag, reached_4_flag, reached_end_flag
    global myDetectNum
# # -------------------Reach the first person identification point---------------------------------------
#     if pos_flag == 1 and not reached_1_flag:
#         Img_1 = data
#         print('reaching 1st detect position.......')
#         cv2.imwrite(
#             '/home/ucar/ucar_ws/src/logic_module/data/1.jpg',
#             bridge.imgmsg_to_cv2(data, "bgr8"))
#         # Go to the second person identification point
#         goto_point(target_detection2)

#         img_pub.publish(Img_1)
#         # rospy.sleep(1)
#         reached_1_flag = True

# # -------------------Reach the second person identification point---------------------------------------
#     elif pos_flag == 2 and not reached_2_flag:
#         Img_2 = data
#         print('reaching 2nd detect position.......')
#         cv2.imwrite(
#             '/home/ucar/ucar_ws/src/logic_module/data/2.jpg',
#             bridge.imgmsg_to_cv2(data, "bgr8"))
#         img_pub.publish(Img_2)
#         goto_point(target_temp)
#         # rospy.sleep(1)
#         reached_2_flag = True

# -------------------Arrive at the temporary location---------------------------------------
    if pos_flag == 3 and not reached_temp_flag:
        # Go to QR code
        goto_point(target_qrcode)
        reached_temp_flag = True

# -------------------Arrive at the QR code recognition point---------------------------------------
    elif pos_flag == 4 and not broadcast_finished_flag:
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
        # Go to the third person identification point
        rospy.sleep(1)
        goto_point(target_detection3)
        broadcast_finished_flag = True

# -------------------Arrived at the third person identification point---------------------------------------
    elif pos_flag == 5 and not reached_3_flag:
        print('reaching 3rd detect position.......')
        Img_3 = data
        img_pub.publish(Img_3)
        cv2.imwrite(
            '/home/ucar/ucar_ws/src/logic_module/data/3.jpg',
            bridge.imgmsg_to_cv2(data, "bgr8"))
        # Go to the fourth person identification point
        goto_point(target_detection4)
        # rospy.sleep(1)
        reached_3_flag = True

# -------------------Arrived at the fourth person identification point---------------------------------------
    elif pos_flag == 6 and not reached_4_flag:
        print('reaching 4th detect position.......')
        Img_4 = data
        cv2.imwrite(
            '/home/ucar/ucar_ws/src/logic_module/data/4.jpg',
            bridge.imgmsg_to_cv2(data, "bgr8"))
        # go to end
        goto_point(target_parking)
        img_pub.publish(Img_4)
        reached_4_flag = True

# -------------------reach destination---------------------------------------
    elif pos_flag == 7 and not reached_end_flag:
        rospy.loginfo("reached END!!!!!!!!!!!!!!!!!!!!!!!!")

        glasses_num = myDetectNum.get_total_glasses()
        if glasses_num > 2:
            glasses_num = 2
        longhair_num = myDetectNum.get_total_longhair()
        if longhair_num > 2:
            longhair_num = 2
        # broadcast person
        person_num = 2
        os.system(
            "play /home/ucar/ucar_ws/src/logic_module/wav/model_%d.wav" % person_num)
        # broadcast longhair
        os.system(
            "play /home/ucar/ucar_ws/src/logic_module/wav/longhair_%d.wav" % longhair_num)
        # broadcast glasses
        os.system(
            "play /home/ucar/ucar_ws/src/logic_module/wav/glasses_%d.wav" % glasses_num)
        os.system("play /home/ucar/ucar_ws/src/logic_module/wav/tts_sample_4.wav")
        rospy.loginfo('+++++++++++++++++++++++++++++++++++++++')
        rospy.loginfo('glasses_num: %d, longhair_num: %d' %
                      (glasses_num, longhair_num))
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
