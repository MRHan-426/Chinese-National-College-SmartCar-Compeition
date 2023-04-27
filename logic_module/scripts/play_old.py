#! /usr/bin/env python2
# -*- coding: utf-8 -*-
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

# ===========================================初始化===========================================

# ---------------------------设定目标点-----------------------------------

# B房间
target_detectionB = Pose(Point(4.995, -0.476, 0), Quaternion(0.000, 0.000, 0.468, 0.884))

# C房间
target_detectionC = Pose(Point(3.28, -1.002, 0.000), Quaternion(0.000, 0.000, -0.705, 0.709))
# D房间
target_detectionD = Pose(Point(1.324, -0.429, 0.000), Quaternion(0.000, 0.000, -0.431, 0.903))

# 增加点位
# target_detectionD3 = Pose(Point(2.016, 1.844, 0.000), Quaternion(0.000, 0.000, 0.705, 0.71))
# ---------------------------设定停车位置目标点-----------------------------------
target_parking = Pose(Point(0.292, 2.817, 0.000), Quaternion(0.000, 0.000, 1.000, -0.002))

pos_flag = 0
"""
0: 起始位置
1：到达B房间
2：到达C房间
3: 到达D房间
4: 到达终点
"""
# 任务完成标志
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
    # 前往第一个点
    goto_point(target_detectionB)


def classes_callback(msg):
    global pos_flag
    global B_list, C_list, D_list

    img_class = msg.bounding_boxes.Class
    img_id = msg.bounding_boxes.id
    if pos_flag == 0 :
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
    global reached_B_flag,reached_C_flag,reached_D_flag,reached_D3_flag,reached_end_flag
    global myDetectNum
    # -------------------到达第一个人物识别点---------------------------------------
    if pos_flag == 1 and not reached_B_flag:
        print('reaching 1st detect position.......')
        Img_1 = data
        img_pub.publish(Img_1)
        cv2.imwrite(
            '/home/ucar/ucar_ws/src/logic_module/data/B.jpg',
            bridge.imgmsg_to_cv2(data, "bgr8"))
        # 前往B2房间
        goto_point(target_detectionC)
        # rospy.sleep(1)
        reached_B_flag = True

    # -------------------到达C房间---------------------------------------
    elif pos_flag == 2 and not reached_C_flag:
        print('reaching 2nd detect position.......')
        Img_2 = data
        img_pub.publish(Img_2)
        cv2.imwrite(
            '/home/ucar/ucar_ws/src/logic_module/data/C.jpg',
            bridge.imgmsg_to_cv2(data, "bgr8"))
        # 前往D房间
        goto_point(target_detectionD)
        # rospy.sleep(1)
        reached_C_flag = True

    # -------------------到达第三个人物识别点---------------------------------------
    elif pos_flag == 3 and not reached_D_flag:
        print('reaching 3rd detect position.......')
        Img_3 = data
        img_pub.publish(Img_3)
        cv2.imwrite(
            '/home/ucar/ucar_ws/src/logic_module/data/D.jpg',
            bridge.imgmsg_to_cv2(data, "bgr8"))
        # 前往D房间
        goto_point(target_detectionD3)
        # rospy.sleep(1)
        reached_D_flag = True

    # -------------------到达第三个人物识别点---------------------------------------

    elif pos_flag == 4 and not reached_D3_flag:
        goto_point(target_parking)
        # rospy.sleep(1)
        reached_D3_flag = True

    # -------------------到达终点---------------------------------------
    elif pos_flag == 5 and not reached_end_flag:
        rospy.loginfo("reached END!!!!!!!!!!!!!!!!!!!!!!!!")

        glasses_num = myDetectNum.get_total_glasses()
        if glasses_num > 2:
            glasses_num = 2
        longhair_num = myDetectNum.get_total_longhair()
        if longhair_num > 2:
            longhair_num = 2
        # 语音播报长发/戴眼镜人数
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

    # 语音唤醒后，麦克风模块发布 start_others消息，msg = 1，此时进入mic_call_back函数。
    rospy.Subscriber("/start_others", Int8, mic_call_back)

    # 每次导航完成时，move_base节点会发布 move_base/result，消息类型是MoveBaseActionResult，此时进入goal_callback函数。
    rospy.Subscriber("move_base/result", MoveBaseActionResult, goal_callback)

    # 每次拍照后，yolo节点会识别图片，并发布"/darknet_ros/bounding_boxes"，此时进入classes_callback函数。
    rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes , classes_callback)

    # 订阅照相机节点，实时获得图片。推荐使用此法，尽量不要直接调用cv.capture。
    rospy.Subscriber('usb_cam/image_raw', Image, img_callback)

    # 发布/move_base_simple/goal，给小车指定下一个坐标点。
    goal_pub = rospy.Publisher(
        '/move_base_simple/goal', PoseStamped, queue_size=1)

    # 发布/final_image，给yolo节点，让它识别。
    img_pub = rospy.Publisher('/final_image', Image, queue_size=1)

    # move_base的初始化
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.spin()
