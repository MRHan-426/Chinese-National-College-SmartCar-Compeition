#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# Author: ziqi han
# Create: 05/19/2021
# Latest Modify: 04/27/2023
# Description: Test Voice Wake up
# This file is used in 16th National University Students Intelligent Car Race

import roslib
import rospy
import math
from geometry msgs.msg import Twist
from std msgs.msg import String


def calLBack(msg):
    rospy.loginfo(msg.data)
    if msg.data.find("向前走") > -1:  # Go forward
        print(information)


if __name__ == "__main__":
    rospy.init_node('test')
    information = "test"
    rospy.sleep(1)
    rospy.Subscriber('voicelNords', String, callBack)
    rospy.spin()
