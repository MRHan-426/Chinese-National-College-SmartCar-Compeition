#! /usr/bin/env python2
# -*- coding: utf-8 -*-

# Author: ziqi han
# Create: 03/24/2021
# Latest Modify: 04/27/2023
# Description: Logic of the project, Direct the car from starting point to end point
# This file is used in 16th National University Students Intelligent Car Race

from numpy.core.numeric import moveaxis
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
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

# ---------------------------set target point-----------------------------------

# Room B
target_detectionB = Pose(Point(4.995, -0.476, 0), Quaternion(0.000, 0.000, 0.468, 0.884))

# Room C
target_detectionC = Pose(Point(3.28, -1.002, 0.000), Quaternion(0.000, 0.000, -0.705, 0.709))
# Room D
target_detectionD = Pose(Point(1.324, -0.429, 0.000), Quaternion(0.000, 0.000, -0.431, 0.903))

# add points
# target_detectionD3 = Pose(Point(2.016, 1.844, 0.000), Quaternion(0.000, 0.000, 0.705, 0.71))

# ---------------------------Set the parking position target point-----------------------------------
target_parking = Pose(Point(0.292, 2.817, 0.000), Quaternion(0.000, 0.000, 1.000, -0.002))

pos_flag = 0
"""
0: start position
1: Arrive in room B
2: Arrive in room C
3: Arrive in room D
4: reach the end
"""
# Mission completed flag
reached_B_flag = False
reached_C_flag = False
reached_D_flag = False
reached_end_flag = False

bridge = CvBridge()


def goto_point(point):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose = point
    goal_pub.publish(goal_pose)


def mic_call_back(data):
    # Go to the first point
    goto_point(target_detectionB)


def classes_callback(msg):
    global pos_flag
    global B_list, C_list, D_list

    img_class = msg.bounding_boxes.Class
    img_id = msg.bounding_boxes.id
    if pos_flag == 0:
        B_list.append(img_id)
        if img_id >= 0 and img_id <= 19:
            pos_flag = 1
            reached_B_flag = True
            goto_point(target_detectionC)

    elif pos_flag == 1:
        C_list.append(img_id)
        if img_id >= 0 and img_id <= 19:
            pos_flag = 2
            reached_C_flag = True
            goto_point(target_detectionD)

    elif pos_flag == 2:
        D_list.append(img_id)
        if img_id >= 0 and img_id <= 19:
            pos_flag = 3
            reached_D_flag = True
            goto_point(target_parking)


def goal_callback(msg):
    global pos_flag
    if msg.status.status == 3:
        if pos_flag != 5:
            rospy.sleep(0.5)
        pos_flag += 1


def img_callback(data):
    # define picture to_down' coefficient of ratio
    global bridge, pos_flag, Img_1, Img_2, Img_3
    global reached_B_flag, reached_C_flag, reached_D_flag, reached_D3_flag, reached_end_flag
    global myDetectNum
    # -------------------Reach the first person identification point---------------------------------------
    if pos_flag == 1 and not reached_B_flag:
        print('reaching 1st detect position.......')
        Img_1 = data
        img_pub.publish(Img_1)
        cv2.imwrite(
            '/home/ucar/ucar_ws/src/logic_module/data/B.jpg',
            bridge.imgmsg_to_cv2(data, "bgr8"))
        # Go to room B2
        goto_point(target_detectionC)
        # rospy.sleep(1)
        reached_B_flag = True

    # -------------------Arrive in room C---------------------------------------
    elif pos_flag == 2 and not reached_C_flag:
        print('reaching 2nd detect position.......')
        Img_2 = data
        img_pub.publish(Img_2)
        cv2.imwrite(
            '/home/ucar/ucar_ws/src/logic_module/data/C.jpg',
            bridge.imgmsg_to_cv2(data, "bgr8"))
        # go to room D
        goto_point(target_detectionD)
        # rospy.sleep(1)
        reached_C_flag = True

    # -------------------Arrived at the third person identification point---------------------------------------
    elif pos_flag == 3 and not reached_D_flag:
        print('reaching 3rd detect position.......')
        Img_3 = data
        img_pub.publish(Img_3)
        cv2.imwrite(
            '/home/ucar/ucar_ws/src/logic_module/data/D.jpg',
            bridge.imgmsg_to_cv2(data, "bgr8"))
        # go to room D
        goto_point(target_detectionD3)
        # rospy.sleep(1)
        reached_D_flag = True

    # -------------------Arriving at the third person identification point ---------------------------------------

    elif pos_flag == 4 and not reached_D3_flag:
        goto_point(target_parking)
        # rospy.sleep(1)
        reached_D3_flag = True

    # -------------------reach destination---------------------------------------
    elif pos_flag == 5 and not reached_end_flag:
        rospy.loginfo("reached END!!!!!!!!!!!!!!!!!!!!!!!!")

        glasses_num = myDetectNum.get_total_glasses()
        if glasses_num > 2:
            glasses_num = 2
        longhair_num = myDetectNum.get_total_longhair()
        if longhair_num > 2:
            longhair_num = 2
        # Voice broadcast the number of people with long hair/glasses
        # broadcast person
        person_num = 2
        os.system("play /home/ucar/ucar_ws/src/logic_module/wav/1.wav")
        # os.system("play /home/ucar/ucar_ws/src/logic_module/wav/model_%d.wav" % person_num)
        # # broadcast longhair
        # os.system("play /home/ucar/ucar_ws/src/logic_module/wav/longhair_%d.wav" % longhair_num)
        # # broadcast glasses
        # os.system("play /home/ucar/ucar_ws/src/logic_module/wav/glasses_%d.wav" % glasses_num)
        # os.system("play /home/ucar/ucar_ws/src/logic_module/wav/tts_sample_4.wav")
        # rospy.loginfo('+++++++++++++++++++++++++++++++++++++++')
        # rospy.loginfo('glasses_num: %d, longhair_num: %d' % (glasses_num, longhair_num))
        # rospy.loginfo('+++++++++++++++++++++++++++++++++++++++')

    #             rospy.loginfo("================================")
    #     rospy.loginfo("This room is Kitchen!")
    #     rospy.loginfo("================================")
    # elif img_id>=8 and img_id<=11:
    #     rospy.loginfo("================================")
    #     rospy.loginfo("This room is Bedroom!")
    #     rospy.loginfo("================================")
    # elif img_id>=12 and img_id<=19:
    #     rospy.loginfo("================================")
    #     rospy.loginfo("This room is Livingroom!")
    #     rospy.loginfo("================================")
        reached_end_flag = True
    else:
        pass


if __name__ == '__main__':
    rospy.init_node('play', anonymous=True)

    # After voice wake-up, the microphone module publishes a start_others message, with msg = 1, entering the mic_call_back function at this point.
    rospy.Subscriber("/start_others", Int8, mic_call_back)

    # After each navigation is completed, the move_base node publishes move_base/result, with the message type being MoveBaseActionResult, entering the goal_callback function at this point.
    rospy.Subscriber("move_base/result", MoveBaseActionResult, goal_callback)

    # After each photo is taken, the yolo node will recognize the image and publish /darknet_ros/classes_num, entering the classes_callback function at this point.
    rospy.Subscriber("/darknet_ros/classes_num", classes, classes_callback)

    # Subscribe to the camera node to get images in real-time. It is recommended to use this method and avoid directly calling cv.capture.
    rospy.Subscriber('usb_cam/image_raw', Image, img_callback)

    # Publish /move_base_simple/goal to specify the next coordinate point for the robot.
    goal_pub = rospy.Publisher(
        '/move_base_simple/goal', PoseStamped, queue_size=1)

    # Publish /final_image to the yolo node, letting it recognize the image.
    img_pub = rospy.Publisher('/final_image', Image, queue_size=1)

    # Initialization of move_base.
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.spin()
