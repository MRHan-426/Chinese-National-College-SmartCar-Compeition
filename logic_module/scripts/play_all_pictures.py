#! /usr/bin/env python2
# -*- coding: utf-8 -*-
from numpy.core.numeric import moveaxis
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import cv2
import os

# ===========================================初始化===========================================

# ---------------------------设定目标点-----------------------------------

# B房间
target_detectionB = Pose(Point(4.996, 0.017, 0), Quaternion(0.000, 0.000, 0.477, 0.879))

#B1房间
target_detectionB2 = Pose(Point(5.006, -0.611, 0), Quaternion(0.000, 0.000, 0.979, -0.206))

# C房间
target_detectionC = Pose(Point(3.446,-1.097, 0.000), Quaternion(0.000, 0.000, 0.495, 0.869))
#C1房间
target_detectionC2 = Pose(Point(3.206,-1.15, 0.000), Quaternion(0.000, 0.000, 0.914, 0.407))

# D房间
target_detectionD = Pose(Point(1.536, 0.662, 0.000), Quaternion(0.000, 0.000, -0.39, 0.921))
# D1房间
target_detectionD2 = Pose(Point(1.024, -0.846, 0.000), Quaternion(0.000, 0.000, 0.579, 0.816))

# ---------------------------设定停车位置目标点-----------------------------------
target_parking = Pose(Point(0.3, 2.75, 0.000), Quaternion(0.000, 0.000, 1.000, 0))

pos_flag = 0
"""
0: 起始位置
1：到达B房间
2：到达C房间
3: 到达D房间
4: 到达终点
"""
bridge = CvBridge()
B_list = []
C_list = []
D_list = []

def goto_point(point):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose = point
    goal_pub.publish(goal_pose)


def mic_call_back(data):
    # 前往第一个点
    goto_point(target_detectionB)


def classes_callback(msg):
    global B_list, C_list, D_list

    # img_class = msg.bounding_boxes[0].Class
    img_id = int(msg.bounding_boxes[0].id)
    img_prob = msg.bounding_boxes[0].probability
    if img_prob > 0.5:
        if pos_flag == 0:
            B_list.append(img_id)
            # if img_id in [8,9,10,11,16,17,18,19,4,5,6,7,24,25,26,27,28,29,30,31]:    
            #     pos_B_finish = 1

            #     pos_flag = 1
            #     goto_point(target_detectionC)

        # B Room pos_flag
        elif pos_flag == 1 or pos_flag ==2:
            loc_pose = rospy.wait_for_message('pose',PoseWithCovarianceStamped)
            if loc_pose.pose.pose.position.x >4:
                B_list.append(img_id)
            else:
                C_list.append(img_id)
                rospy.loginfo(loc_pose.pose.pose.position.x)

        # C Room pos_flag
        elif pos_flag == 3 or pos_flag==4:
            loc_pose = rospy.wait_for_message('pose',PoseWithCovarianceStamped)
            if loc_pose.pose.pose.position.x >2.5:
                C_list.append(img_id)
            else:
                D_list.append(img_id)

        # D Room pos_flag
        elif pos_flag == 5 or pos_flag==6:
            loc_pose = rospy.wait_for_message('pose',PoseWithCovarianceStamped)
            if loc_pose.pose.pose.position.y < 1.5 :

                D_list.append(img_id)


# 新函数！取出现频次前三的标号
def filter_list(my_list):
    dic = {}
    my_newlist = []
    for i in my_list:
        dic[i] = dic.get(i, 0) + 1
    counts = dic.values()
    counts = sorted(counts, reverse=True)

    for i in counts:
        for key, value in dic.items():
            if i == value:
                my_newlist.append(key)
        if len(my_newlist) >= 3:
            break
    return my_newlist


def goal_callback(msg):
    global pos_flag
    global B_list, C_list, D_list

    type_B = type_C = type_D = "Unknown"

    if msg.status.status == 3:
        pos_flag += 1
        if pos_flag == 1:
            goto_point(target_detectionB2)

        elif pos_flag == 2:
            goto_point(target_detectionC)

        elif pos_flag == 3:
            goto_point(target_detectionC2)
        
        elif pos_flag == 4:
            goto_point(target_detectionD)
        elif pos_flag == 5:
            goto_point(target_detectionD2)
       
        elif pos_flag == 6:
            goto_point(target_parking)

        elif pos_flag == 7:
            rospy.loginfo("======================================")
            rospy.loginfo(B_list)
            rospy.loginfo("======================================")   
            rospy.loginfo(C_list)
            rospy.loginfo("======================================")
            rospy.loginfo(D_list)
            rospy.loginfo("======================================")

            # 取出现频次前三的标号
            B_list = filter_list(B_list)
            C_list = filter_list(C_list)
            D_list = filter_list(D_list)
            # 如果需要观察结果取消注释
            # rospy.loginfo("======================================")
            # rospy.loginfo(B_list)
            # rospy.loginfo("======================================")
            # rospy.loginfo(C_list)
            # rospy.loginfo("======================================")
            # rospy.loginfo(D_list)
            # rospy.loginfo("======================================")

            # 同时出现多个标志物，优先级餐厅＞卧室＞客厅
            for i in B_list:
                if i in [24, 25, 26, 27, 28, 29, 30, 31]:
                    flag = True
                    for j in B_list:
                        if j in [4, 5, 6, 7, 8, 9, 10, 11, 16, 17, 18, 19, 20, 21, 22, 23]:
                            flag = False
                            break
                    if flag:
                        type_B = "Livingroom"
                        break
                if (i in [4, 5, 6, 7]) or (i in [0, 1, 2, 3] and i in [20, 21, 22, 23]):
                    flag = True
                    for j in B_list:
                        if j in [8, 9, 10, 11, 16, 17, 18, 19]:
                            flag = False
                            break
                    if flag:
                        type_B = "Bedroom"
                        break
                if i in [8, 9, 10, 11, 16, 17, 18, 19]:
                    flag = True
                    for j in B_list:
                        if j in [0, 1, 2, 3]:
                            flag = False
                            break
                    if flag:
                        type_B = "Kitchen"
                        break

            for i in C_list:
                if i in [24, 25, 26, 27, 28, 29, 30, 31]:
                    flag = True
                    for j in C_list:
                        if j in [4, 5, 6, 7, 8, 9, 10, 11, 16, 17, 18, 19, 20, 21, 22, 23]:
                            flag = False
                            break
                    if flag:
                        type_C = "Livingroom"
                        break
                if (i in [4, 5, 6, 7]) or (i in [0, 1, 2, 3] and i in [20, 21, 22, 23]):
                    flag = True
                    for j in C_list:
                        if j in [8, 9, 10, 11, 16, 17, 18, 19]:
                            flag = False
                            break
                    if flag:
                        type_C = "Bedroom"
                        break
                if i in [8, 9, 10, 11, 16, 17, 18, 19]:
                    flag = True
                    for j in C_list:
                        if j in [0, 1, 2, 3]:
                            flag = False
                            break
                    if flag:
                        type_C = "Kitchen"
                        break

            for i in D_list:
                if i in [24, 25, 26, 27, 28, 29, 30, 31]:
                    flag = True
                    for j in D_list:
                        if j in [4, 5, 6, 7, 8, 9, 10, 11, 16, 17, 18, 19, 20, 21, 22, 23]:
                            flag = False
                            break
                    if flag:
                        type_D = "Livingroom"
                        break
                if (i in [4, 5, 6, 7]) or (i in [0, 1, 2, 3] and i in [20, 21, 22, 23]):
                    flag = True
                    for j in D_list:
                        if j in [8, 9, 10, 11, 16, 17, 18, 19]:
                            flag = False
                            break
                    if flag:
                        type_D = "Bedroom"
                        break
                if i in [8, 9, 10, 11, 16, 17, 18, 19]:
                    flag = True
                    for j in D_list:
                        if j in [0, 1, 2, 3]:
                            flag = False
                            break
                    if flag:
                        type_D = "Kitchen"
                        break

            if type_B==type_C :
                type_C="Unknown"
            elif type_D==type_C :
                type_D="Unknown"
            elif type_D==type_B :
                type_D="Unknown"

            rospy.loginfo("B:%s C:%s D:%s" % (type_B, type_C, type_D))

            if type_B == "Kitchen" and type_C == "Bedroom" and type_D == "Livingroom":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/1.wav")
            elif type_B == "Unknown" and type_C == "Bedroom" and type_D == "Livingroom":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/1.wav")
            elif type_B == "Kitchen" and type_C == "Unknown" and type_D == "Livingroom":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/1.wav")
            elif type_B == "Kitchen" and type_C == "Bedroom" and type_D == "Unknown":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/1.wav")


            if type_B == "Kitchen" and type_C == "Livingroom" and type_D == "Bedroom":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/2.wav")
            elif type_B == "Unknown" and type_C == "Livingroom" and type_D == "Bedroom":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/2.wav")
            elif type_B == "Kitchen" and type_C == "Unknown" and type_D == "Bedroom":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/2.wav")
            elif type_B == "Kitchen" and type_C == "Livingroom" and type_D == "Unknown":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/2.wav")


            elif type_B == "Bedroom" and type_C == "Kitchen" and type_D == "Livingroom":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/3.wav")
            elif type_B == "Unknown" and type_C == "Kitchen" and type_D == "Livingroom":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/3.wav")
            elif type_B == "Bedroom" and type_C == "Unknown" and type_D == "Livingroom":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/3.wav")
            elif type_B == "Bedroom" and type_C == "Kitchen" and type_D == "Unknown":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/3.wav")

            elif type_B == "Bedroom" and type_C == "Livingroom" and type_D == "Kitchen":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/4.wav")
            elif type_B == "Unknown" and type_C == "Livingroom" and type_D == "Kitchen":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/4.wav")
            elif type_B == "Bedroom" and type_C == "Unknown" and type_D == "Kitchen":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/4.wav")
            elif type_B == "Bedroom" and type_C == "Livingroom" and type_D == "Unknown":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/4.wav")


            elif type_B == "Livingroom" and type_C == "Kitchen" and type_D == "Bedroom":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/6.wav")
            elif type_B == "Unknown" and type_C == "Kitchen" and type_D == "Bedroom":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/6.wav")
            elif type_B == "Livingroom" and type_C == "Unknown" and type_D == "Bedroom":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/6.wav")
            elif type_B == "Livingroom" and type_C == "Kitchen" and type_D == "Unknown":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/6.wav")

            elif (type_B == "Livingroom" and type_C == "Unknown" and type_D == "Unknown") \
                or (type_B == "Unknown" and type_C == "Kitchen" and type_D == "Unknown") \
                or (type_B == "Unknown" and type_C == "Unknown" and type_D == "Bedroom"):
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/6.wav")
            elif (type_B == "Livingroom" and type_C == "Unknown" and type_D == "Unknown") \
                or (type_B == "Unknown" and type_C == "Bedroom" and type_D == "Unknown") \
                or (type_B == "Unknown" and type_C == "Unknown" and type_D == "Kitchen"):
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/5.wav")
            elif (type_B == "Bedroom" and type_C == "Unknown" and type_D == "Unknown") \
                or (type_B == "Unknown" and type_C == "Livingroom" and type_D == "Unknown") \
                or (type_B == "Unknown" and type_C == "Unknown" and type_D == "Kitchen"):
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/4.wav")
            
            else:
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/5.wav")

if __name__ == '__main__':
    rospy.init_node('play', anonymous=True)

    # 语音唤醒后，麦克风模块发布 start_others消息，msg = 1，此时进入mic_call_back函数。
    rospy.Subscriber("/start_others", Int8, mic_call_back)

    # 每次导航完成时，move_base节点会发布 move_base/result，消息类型是MoveBaseActionResult，此时进入goal_callback函数。
    rospy.Subscriber("move_base/result", MoveBaseActionResult, goal_callback)

    # 每次拍照后，yolo节点会识别图片，并发布"/darknet_ros/bounding_boxes"，此时进入classes_callback函数。
    rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes , classes_callback)

    # 发布/move_base_simple/goal，给小车指定下一个坐标点。
    goal_pub = rospy.Publisher(
        '/move_base_simple/goal', PoseStamped, queue_size=1)

    # move_base的初始化
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.spin()
