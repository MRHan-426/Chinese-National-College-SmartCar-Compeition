#! /usr/bin/env python2
# -*- coding: utf-8 -*-
from numpy.core.numeric import moveaxis
import rospy
from darknet_ros_msgs.msg import classes
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseActionResult,MoveBaseGoal,MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import cv2
import os


# ===========================================初始化===========================================
PARKING_ID = 2 # 1,2,3

# ---------------------------设定二维码目标点-----------------------------------
target_qrcode = Pose(Point(2.609, 1.150, 0.000), Quaternion(0.000, 0.000, 0.996, 0.0927))

#转弯过渡点
#Position(2.116, 1.730, 0.000), Orientation(0.000, 0.000, 0.006, 1.000)
target_temp = Pose(Point(2.116, 1.730, 0.000),Quaternion(0.000, 0.000, 0.006, 1.000))

# ---------------------------设定停车位置目标点-----------------------------------
if PARKING_ID == 1:         #D1
#Position(1.217, 2.702, 0.000), Orientation(0.000, 0.000, 0.714, 0.700)
    target_parking = Pose(Point(1.25, 2.75, 0.000), Quaternion(0.000, 0.000, 0.714, 0.700))            #Pose(Point(1.217, 2.702, 0.000), Quaternion(0.000, 0.000, 0.714, 0.700))
elif PARKING_ID == 2:       #D2
    target_parking = Pose(Point(0.75, 2.65, 0.000), Quaternion(0.000, 0.000, 0.7, 0.714))
elif PARKING_ID == 3:       #D3
    target_parking = Pose(Point(0.324, 2.65, 0.000), Quaternion(0.000, 0.000, 0.726, 0.687))



pos_flag = 0
"""
0: 起始位置
1：到达7区识别位置
2：到达6区识别位置
3: 6区过渡点
4：到达二维码识别位置
5：到达C区
6：C区转完
7：到达终点
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
    # 前往转弯过渡点
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

# -------------------到达临时位置点---------------------------------------
    if pos_flag == 1 and not reached_temp_flag:
        # 前往二维码
        goto_point(target_qrcode)
        reached_temp_flag = True
 
# -------------------到达二维码识别点---------------------------------------
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
        # 前往第三个人物识别点
        rospy.sleep(1)
        goto_point(target_parking)
        broadcast_finished_flag = True

# -------------------到达终点---------------------------------------
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
