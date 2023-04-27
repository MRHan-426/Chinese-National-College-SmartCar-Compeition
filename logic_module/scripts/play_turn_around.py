#! /usr/bin/env python2
# -*- coding: utf-8 -*-

# Author: ziqi han
# Create: 05/30/2021
# Latest Modify: 04/27/2023
# Description: Logic of the project, Direct the car from starting point to end point
# This file is used in 16th National University Students Intelligent Car Race

from numpy.core.numeric import moveaxis
import rospy
from darknet_ros_msgs.msg import classes
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import os

# ===========================================initialization===========================================

# ---------------------------set target point-----------------------------------

# Room B
target_detectionB = Pose(Point(5.032, -0.238, 0), Quaternion(0.000, 0.000, -0.574, 0.819))
# Room C
target_detectionC = Pose(Point(3.284, -0.42, 0.000), Quaternion(0.000, 0.000, -0.704, 0.71))
# Room D
target_detectionD = Pose(Point(1.686, -0.389, 0.000), Quaternion(0.000, 0.000, -0.793, 0.610))
# Room D, At the Enterance
target_detection_temp = Pose(Point(2.068, 1.549, 0.000), Quaternion(0.000, 0.000, 0.825, 0.566))

# ---------------------------Set the parking position target point-----------------------------------
target_parking = Pose(Point(0.335, 2.579, 0.000), Quaternion(0.000, 0.000, 1.000, -0.011))

pos_flag = 0
"""
0: start position
1: Arrive in room B
2: First spin in room B
3: Spin a second time in room B
4: Arrive in room C
5: First spin in room C
6: Arrive in room D
7: First spin in room D
8: Spin a second time in room D
9: Pause at gate D to prevent rotation
10: reach the finish line
"""

reached_B_flag = False
# reached_B2_flag = False
# reached_B3_flag = False

reached_C_flag = False
# reached_C2_flag = False

reached_D_flag = False
# reached_D2_flag = False
# reached_D3_flag = False

reached_temp_flag = False
reached_end_flag = False

Img_B1 = Image()
Img_B2 = Image()
Img_B3 = Image()

Img_C1 = Image()
Img_C2 = Image()

Img_D1 = Image()
Img_D2 = Image()
Img_D3 = Image()


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
    # Go to the first point
    goto_point(target_detectionB)


def classes_callback(msg):
    global myDetectNum, pos_flag
    if pos_flag == 3:
        myDetectNum.glasses_num_pic1 = msg.glass_num
        myDetectNum.longhair_num_pic1 = msg.long_hair_num
        rospy.loginfo("================================")
        rospy.loginfo('recognition position = 1')
        rospy.loginfo('glass = %d,  longhair = %d.' % (msg.glass_num, msg.long_hair_num))
        # rospy.loginfo(msg)
        rospy.loginfo("================================")

    elif pos_flag == 4:
        myDetectNum.glasses_num_pic2 = msg.glass_num
        myDetectNum.longhair_num_pic2 = msg.long_hair_num
        rospy.loginfo("================================")
        rospy.loginfo('recognition position = 2')
        rospy.loginfo('glass = %d,  longhair = %d.' % (msg.glass_num, msg.long_hair_num))
        # rospy.loginfo(msg)
        rospy.loginfo("================================")

    elif pos_flag == 5:
        myDetectNum.glasses_num_pic3 = msg.glass_cut_num
        myDetectNum.longhair_num_pic3 = msg.long_hair_cut_num
        rospy.loginfo("================================")
        rospy.loginfo('recognition position = 3')
        rospy.loginfo('glass = %d,  longhair = %d.' % (msg.glass_cut_num, msg.long_hair_cut_num))
        # rospy.loginfo(msg)
        rospy.loginfo("================================")


def goal_callback(msg):
    global pos_flag
    if msg.status.status == 3:
        if pos_flag != 10:
            rospy.sleep(0.5)
        pos_flag += 1


def turn_120_around():
    turn_120_around_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(50)
    rospy.sleep(0.2)
    vel_msg = Twist()
    count = 0
    while not rospy.is_shutdown():
        while (count < 120):
            vel_msg.angular.z = 1
            turn_120_around_pub.publish(vel_msg)
            count = count + 1
            rate.sleep()
        break


def turn_180_around():
    turn_180_around_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(50)
    rospy.sleep(0.2)
    vel_msg = Twist()
    count = 0
    while not rospy.is_shutdown():
        while (count < 180):
            vel_msg.angular.z = 1
            turn_180_around_pub.publish(vel_msg)
            count = count + 1
            rate.sleep()
        break


def img_callback(data):
    global bridge, pos_flag, Img_B1, Img_B2, Img_B3, Img_C1, Img_C2, Img_D1, Img_D2, Img_D3
    global reached_B_flag, reached_B2_flag, reached_B3_flag, reached_C_flag, reached_C2_flag, reached_D_flag, reached_D2_flag, reached_D3_flag
    global reached_end_flag, reached_temp_flag
    global myDetectNum
    # -------------------Arrive in room B---------------------------------------
    if pos_flag == 1 and not reached_B_flag:
        Img_B1 = data
        # img_pub.publish(Img_B1)
        cv2.imwrite(
            '/home/ucar/ucar_ws/src/logic_module/data/B1.jpg',
            bridge.imgmsg_to_cv2(data, "bgr8"))
        reached_B_flag = True

        ### Rotate 120° ###
        pos_flag = 2
        # turn_120_around()
        rospy.sleep(0.2)

    elif pos_flag == 2:
        Img_B2 = data
        # img_pub.publish(Img_B2)
        cv2.imwrite(
            '/home/ucar/ucar_ws/src/logic_module/data/B2.jpg',
            bridge.imgmsg_to_cv2(data, "bgr8"))

        ### Rotate 120° ###
        pos_flag = 3
        # turn_120_around()
        rospy.sleep(0.2)

    elif pos_flag == 3:
        Img_B3 = data
        # img_pub.publish(Img_B3)
        cv2.imwrite(
            '/home/ucar/ucar_ws/src/logic_module/data/B3.jpg',
            bridge.imgmsg_to_cv2(Img_B3, "bgr8"))
        # go to room C
        goto_point(target_detectionC)
        # rospy.sleep(1)

# -------------------Arrive in room C---------------------------------------
    elif pos_flag == 4 and not reached_C_flag:
        Img_C1 = data
        # img_pub.publish(Img_C1)
        cv2.imwrite(
            '/home/ucar/ucar_ws/src/logic_module/data/C1.jpg',
            bridge.imgmsg_to_cv2(data, "bgr8"))
        reached_C_flag = True

        ### Rotate 180° ###
        pos_flag = 5
        turn_180_around()
        rospy.sleep(0.2)

    elif pos_flag == 5:
        Img_C2 = data
        img_pub.publish(Img_C2)
        cv2.imwrite(
            '/home/ucar/ucar_ws/src/logic_module/data/C2.jpg',
            bridge.imgmsg_to_cv2(Img_C2, "bgr8"))

        # go to room D
        goto_point(target_detectionD)
        # rospy.sleep(1)

# -------------------Arrive in room D---------------------------------------
    elif pos_flag == 6 and not reached_D_flag:
        Img_D1 = data
        img_pub.publish(Img_D1)
        cv2.imwrite(
            '/home/ucar/ucar_ws/src/logic_module/data/D1.jpg',
            bridge.imgmsg_to_cv2(data, "bgr8"))
        reached_D_flag = True

        ### Rotate 120° ###
        pos_flag = 7
        turn_120_around()
        rospy.sleep(0.2)

    elif pos_flag == 7:
        Img_D2 = data
        img_pub.publish(Img_D2)
        cv2.imwrite(
            '/home/ucar/ucar_ws/src/logic_module/data/D2.jpg',
            bridge.imgmsg_to_cv2(Img_D2, "bgr8"))

        ### Rotate 120° ###
        pos_flag = 8
        turn_120_around()
        rospy.sleep(0.2)

    elif pos_flag == 8:
        Img_D3 = data
        img_pub.publish(Img_D3)
        cv2.imwrite(
            '/home/ucar/ucar_ws/src/logic_module/data/D3.jpg',
            bridge.imgmsg_to_cv2(Img_D3, "bgr8"))

        # Go to the pause point at the door of D
        goto_point(target_detection_temp)
        # rospy.sleep(1)

# -------------------Arrive at the temporary point---------------------------------------
    elif pos_flag == 9 and not reached_temp_flag:
        # go to room D
        goto_point(target_parking)
        # rospy.sleep(1)
        reached_temp_flag = True

# -------------------reach destination---------------------------------------
    elif pos_flag == 10 and not reached_end_flag:
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
