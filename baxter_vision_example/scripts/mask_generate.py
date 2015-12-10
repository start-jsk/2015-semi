#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv_bridge
import cv2
import rospy
import rospkg
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import Int16

class mask_generate():
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.rospack = rospkg.RosPack()
        self.img_path = self.rospack.get_path('baxter_vision_example') + u"/data/fg.jpg" 
        self.bg_prv = cv2.imread(self.img_path)
        self.delta_x = 0
        self.threshold = 100
        rospy.init_node('mask_generate',anonymous=True)
        rospy.Subscriber("split_fore_background/output/fg",Image,self.callback_bg)    
        rospy.Subscriber("mask_compare/output/delta_x",Int16,self.callback_x)
        rospy.spin()

    def callback_bg(self,data):
        self.bg_cur = self.bridge.imgmsg_to_cv2(data,data.encoding)
        self.mask = self.compare_generate(self.bg_prv,self.bg_cur,self.delta_x)
        self.mask_img_path = self.rospack.get_path('baxter_vision_example') + u"/data/mask.jpg" 
        cv2.imwrite(self.mask_img_path,self.mask)

    def callback_x(self,msg):
        self.delta_x = msg.data

    def compare_generate(self,img_prv,img_cur,delta_x):
        mask = np.copy(np.array(img_cur))
        for y in range(0,len(img_cur)):
            for x in range(0,len(img_cur[0])):
                if delta_x > 0:
                    if x < len(img_cur[0]) - delta_x :
                        if abs(int(sum(img_cur[y][x])) -int(sum(img_prv[y][x+delta_x]))) > self.threshold:
                            mask[y][x] = [0,0,0]
                        else :
                            mask[y][x] = [255,255,255]
                    else :
                        mask[y][x] = [255,255,255]
                else:
                    if x > -delta_x:
                        if abs(int(sum(img_cur[y][x])) - int(sum(img_prv[y][x+delta_x]))) > self.threshold:
                            mask[y][x] = [0,0,0]
                        else :
                            mask[y][x] = [255,255,255]
                    else:
                        mask[y][x] = [255,255,255]
        return mask

if __name__ =='__main__':
    mask_generator = mask_generate()
