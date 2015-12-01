#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv_bridge
import cv2
import rospy
import rospkg
import sys
from sensor_msgs.msg import Image

def callback_bg(data):
    bridge = cv_bridge.CvBridge()
    rospack = rospkg.RosPack()
    path = rospack.get_path('baxter_vision_example') + u"/data/background.jpg" 
    img = bridge.imgmsg_to_cv2(data,"bgr8")
    cv2.imwrite(path,img)

def callback_fg(data):
    bridge = cv_bridge.CvBridge()
    rospack = rospkg.RosPack()
    path = rospack.get_path('baxter_vision_example') + u"/data/foreground.jpg" 
    img = bridge.imgmsg_to_cv2(data,"bgr8")
    cv2.imwrite(path,img)

def image_saver():
    rospy.init_node('image_saver',anonymous=True)
    rospy.Subscriber("split_fore_background/output/bg_mask",Image,callback_bg)
    rospy.Subscriber("split_fore_background/output/fg_mask",Image,callback_fg)
    rospy.spin()

if __name__ == '__main__':
    image_saver()
