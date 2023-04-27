#! /usr/bin/env python2
# -*- coding: utf-8 -*-

# Author: ziqi han
# Create: 08/03/2021
# Latest Modify: 04/27/2023
# Description: Logic of the project, Direct the car from starting point to end point

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

# ===========================================initialization===========================================

# ---------------------------set target point-----------------------------------

# # Room B Plan1
# target_detectionB = Pose(Point(5.0, -0.5, 0), Quaternion(0.000, 0.000, 0.0, 1))

# # Room C
# target_detectionC = Pose(Point(3.0,-0.75, 0.000), Quaternion(0.000, 0.000, -0.705, 0.709))
# # Room D
# target_detectionD = Pose(Point(2, 1, 0.000), Quaternion(0.000, 0.000, -0.791, 0.612))

# # Room B   40.56 Normal
# target_detectionB = Pose(Point(5.0, -0.5, 0), Quaternion(0.000, 0.000, 0.0, 1))
# # Room C
# target_detectionC = Pose(Point(3.25,-0.5, 0.000), Quaternion(0.000, 0.000, -0.705, 0.709))
# # Room D
# target_detectionD = Pose(Point(1.5, -0.5, 0.000), Quaternion(0.000, 0.000, -0.707, 0.707))


# Room B    Extreme
target_detectionB = Pose(Point(4.75, 1.25, 0), Quaternion(0.000, 0.000, -0.705, 0.709))
# Room C
target_detectionC = Pose(Point(3.25, 0.25, 0.000), Quaternion(0.000, 0.000, -0.705, 0.709))
# Room D
target_detectionD = Pose(Point(2.25, 1.25, 0.000), Quaternion(0.000, 0.000, -0.707, 0.707))

# Add points
test1 = Pose(Point(2.068, 1.308, 0.000), Quaternion(0.000, 0.000, -0.752, 0.659))
test2 = Pose(Point(1.756, 0.171, 0.000), Quaternion(0.000, 0.000, -0.732, 0.681))
test3 = Pose(Point(1.575, -1.118, 0.000), Quaternion(0.000, 0.0000, -0.753, 0.658))
test4 = Pose(Point(0.953, -0.868, 0.000), Quaternion(0.000, 0.000, 0.852, 0.524))
test5 = Pose(Point(0.786, 0.282, 0.000), Quaternion(0.000, 0.000, 0.473, 0.881))
test6 = Pose(Point(1.349, 1.142, 0.0000), Quaternion(0.000, 0.000, 0.278, 0.961))


# ---------------------------Set the parking position target point-----------------------------------
target_parking = Pose(Point(0.3, 2.75, 0.000), Quaternion(0.000, 0.000, 1.000, 0))

pos_flag = 0
"""
0: start position
1: Arrive in room B
2: Arrive in room C
3: Arrive in room D
4: reach the end
"""
bridge = CvBridge()
B_list = []
C_list = []
D_list = []
pos_B_finish = 0
pos_C_finish = 0
pos_D_finish = 0


def goto_point(point):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose = point
    goal_pub.publish(goal_pose)


def mic_call_back(data):
    # Go to the first point
    goto_point(target_detectionB)


def classes_callback(msg):
    global pos_flag, pos_B_finish, pos_C_finish, pos_D_finish
    global B_list, C_list, D_list

    # img_class = msg.bounding_boxes[0].Class
    img_id = int(msg.bounding_boxes[0].id)
    img_prob = msg.bounding_boxes[0].probability
    if img_prob > 0.3:
        if pos_flag == 0:
            B_list.append(img_id)
            # if img_id in [8,9,10,11,16,17,18,19,4,5,6,7,24,25,26,27,28,29,30,31]:
            #     pos_B_finish = 1

            #     pos_flag = 1
            #     goto_point(target_detectionC)

        elif pos_flag == 1:
            loc_pose = rospy.wait_for_message('pose', PoseWithCovarianceStamped)
            if loc_pose.pose.pose.position.x > 4:
                B_list.append(img_id)
            else:
                C_list.append(img_id)
                rospy.loginfo(loc_pose.pose.pose.position.x)
            # if img_id in [8,9,10,11,16,17,18,19,4,5,6,7,24,25,26,27,28,29,30,31]:
            #     pos_C_finish = 1

        #     pos_flag = 2
        #     goto_point(target_detectionD)

        elif pos_flag == 2:
            loc_pose = rospy.wait_for_message('pose', PoseWithCovarianceStamped)
            if loc_pose.pose.pose.position.x > 2.5:
                C_list.append(img_id)
            else:
                D_list.append(img_id)
            # if img_id in [8,9,10,11,16,17,18,19,4,5,6,7,24,25,26,27,28,29,30,31]:
            #     pos_D_finish = 1

            #     pos_flag = 3
            #     goto_point(target_parking)
        elif pos_flag == 3:
            loc_pose = rospy.wait_for_message('pose', PoseWithCovarianceStamped)
            if loc_pose.pose.pose.position.y < 1.5:

                D_list.append(img_id)

            # if img_id in [8,9,10,11,16,17,18,19,4,5,6,7,24,25,26,27,28,29,30,31]:
            #     pos_D_finish = 1


# Get the top three labels with the frequency
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
            goto_point(target_detectionC)
        elif pos_flag == 2:
            goto_point(target_detectionD)
        # elif pos_flag == 3:
        #     goto_point(test1)
        # elif pos_flag == 4:
        #     goto_point(test2)
        # elif pos_flag ==5:
        #     goto_point(test3)
        # elif pos_flag ==6:
        #     goto_point(test4)
        # elif pos_flag ==7:
        #     goto_point(test5)
        # elif pos_flag ==8:
        #     goto_point(test6)

        elif pos_flag == 3:
            rospy.sleep(0.5)
            goto_point(target_parking)
        elif pos_flag == 4:
            rospy.loginfo("======================================")
            rospy.loginfo(B_list)
            rospy.loginfo("======================================")
            rospy.loginfo(C_list)
            rospy.loginfo("======================================")
            rospy.loginfo(D_list)
            rospy.loginfo("======================================")

            # Get the top three labels with the frequency
            B_list = filter_list(B_list)
            C_list = filter_list(C_list)
            D_list = filter_list(D_list)
            # rospy.loginfo("======================================")
            # rospy.loginfo(B_list)
            # rospy.loginfo("======================================")
            # rospy.loginfo(C_list)
            # rospy.loginfo("======================================")
            # rospy.loginfo(D_list)
            # rospy.loginfo("======================================")

            # Multiple markers appear at the same time,
            # the priority is restaurant > bedroom > living room,
            # if there are people and pets,
            # it will be judged as a bedroom immediately
            for i in B_list:
                if i in [0, 1, 2, 3] and i in [20, 21, 22, 23]:
                    type_B = "Bedroom"
                    break
                if i in [24, 25, 26, 27, 28, 29, 30, 31]:
                    flag = True
                    for j in B_list:
                        if j in [4, 5, 6, 7, 8, 9, 10, 11, 16, 17, 18, 19, 20, 21, 22, 23]:
                            flag = False
                            break
                    if flag:
                        type_B = "Livingroom"
                        break
                if i in [4, 5, 6, 7]:
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
                if i in [0, 1, 2, 3] and i in [20, 21, 22, 23]:
                    type_C = "Bedroom"
                    break
                if i in [24, 25, 26, 27, 28, 29, 30, 31]:
                    flag = True
                    for j in C_list:
                        if j in [4, 5, 6, 7, 8, 9, 10, 11, 16, 17, 18, 19, 20, 21, 22, 23]:
                            flag = False
                            break
                    if flag:
                        type_C = "Livingroom"
                        break
                if i in [4, 5, 6, 7]:
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
                if i in [0, 1, 2, 3] and i in [20, 21, 22, 23]:
                    type_D = "Bedroom"
                    break
                if i in [24, 25, 26, 27, 28, 29, 30, 31]:
                    flag = True
                    for j in D_list:
                        if j in [4, 5, 6, 7, 8, 9, 10, 11, 16, 17, 18, 19, 20, 21, 22, 23]:
                            flag = False
                            break
                    if flag:
                        type_D = "Livingroom"
                        break
                if i in [4, 5, 6, 7]:
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

            rospy.loginfo("B:%s C:%s D:%s" % (type_B, type_C, type_D))

            if type_B == type_C:
                type_C = "Unknown"
            elif type_D == type_C:
                type_C = "Unknown"
            elif type_D == type_B:
                type_D = "Unknown"

            if type_B == "Kitchen" and type_C == "Bedroom" and type_D == "Livingroom":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/1.wav")
            elif type_B == "Unknown" and type_C == "Bedroom" and type_D == "Livingroom":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/1.wav")
            elif type_B == "Kitchen" and type_C == "Unknown" and type_D == "Livingroom":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/1.wav")
            elif type_B == "Kitchen" and type_C == "Bedroom" and type_D == "Unknown":
                os.system("play /home/ucar/ucar_ws/src/logic_module/wav/1.wav")

            elif type_B == "Kitchen" and type_C == "Livingroom" and type_D == "Bedroom":
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
    # After voice activation, the microphone module publishes a start_others message with msg = 1, and the mic_call_back function is executed.
    rospy.Subscriber("/start_others", Int8, mic_call_back)

    # When navigation is completed, the move_base node publishes move_base/result with the message type MoveBaseActionResult, and the goal_callback function is executed.
    rospy.Subscriber("move_base/result", MoveBaseActionResult, goal_callback)

    rospy.Subscriber("move_base/status", GoalStatusArray, navi_fail_callback)

    # After taking a photo, the yolo node recognizes the image and publishes "/darknet_ros/bounding_boxes", and the classes_callback function is executed.
    rospy.Subscriber("/darknet_ros/bounding_boxes",
                     BoundingBoxes, classes_callback)

    # Publishes /move_base_simple/goal to specify the next coordinate point for the robot.
    goal_pub = rospy.Publisher(
        '/move_base_simple/goal', PoseStamped, queue_size=1)

    # Initialization of move_base
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.spin()
