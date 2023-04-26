#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import roslib
import rospy
import math
from geometry msgs.msg import Twist
from std msgs.msg import String

def calLBack(msg):
    rospy.loginfo(msg.data)
    if msg.data.find("向前走")>-1:
        print(information)
        
if __name__=="__main__":
    rospy.init_node('test')
    information = "test"
    rospy.sleep(1)
    rospy.Subscriber('voicelNords', String, callBack)
    rospy.spin()