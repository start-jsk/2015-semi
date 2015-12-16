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
    def __init__(self,threshold):
        self.bridge = cv_bridge.CvBridge()
        self.rospack = rospkg.RosPack()
        self.img_path = self.rospack.get_path('baxter_vision_example') + u"/data/fg.jpg" 
        self.bg_prv = cv2.imread(self.img_path)
        self.delta_x = 0
        self.count = 8
        self.kernel_size = 3 
        self.threshold = int(threshold)
        rospy.init_node('mask_generate',anonymous=True)
        rospy.Subscriber("split_fore_background/output/fg",Image,self.callback_bg)    
        rospy.Subscriber("mask_compare/output/delta_x",Int16,self.callback_x)
        rospy.spin()

    def callback_bg(self,data):
        self.bg_cur = self.bridge.imgmsg_to_cv2(data,data.encoding)
        self.mask = self.compare_generate(self.bg_prv,self.bg_cur,self.delta_x)
        self.mask_ = self.remove_noise(self.mask,self.count,self.kernel_size)
        self.mask_img_path = self.rospack.get_path('baxter_vision_example') + u"/data/mask.jpg" 
        self.mask_img_path_ = self.rospack.get_path('baxter_vision_example') + u"/data/mask_removed_noise.jpg" 
        print self.delta_x
        cv2.imwrite(self.mask_img_path,self.mask)
        cv2.imwrite(self.mask_img_path_,self.mask_)    

    def callback_x(self,msg):
        self.delta_x = msg.data

    def remove_noise(self,mask,count,kernel_size):
        if not count > 0:
            return mask
        else:
            kernel = np.ones((kernel_size,kernel_size),np.uint8)
            for i in range(0,count):
                mask = cv2.dilate(mask,kernel,iterations=1)
            for i in range(0,count):
                mask = cv2.erode(mask,kernel,iterations=1)
            return mask

    def compare_generate(self,img_prv,img_cur,delta_x):
        mask = np.copy(np.array(img_cur))
        img_prv_moved = np.copy(np.array(img_prv))
        for y in range(0,len(img_cur)):
            for x in range(0,len(img_cur[0])):
                if delta_x > 0:
                    # if x < len(img_cur[0]) - delta_x :
                    if x > delta_x:
                        if abs(int(sum(img_cur[y][x])) -int(sum(img_prv[y][x-delta_x]))) > self.threshold:
                            mask[y][x] = [0,0,0]
                        else :
                            mask[y][x] = [255,255,255]
                        img_prv_moved[y][x] = img_prv[y][x-delta_x] 
                    else :
                        mask[y][x] = [255,255,255]
                        img_prv_moved[y][x] = [255,255,255]
                else:
                    # if x > -delta_x:
                    if x < len(img_cur[0]) + delta_x:
                        if abs(int(sum(img_cur[y][x])) - int(sum(img_prv[y][x-delta_x]))) > self.threshold:
                            mask[y][x] = [0,0,0]
                        else :
                            mask[y][x] = [255,255,255]
                        img_prv_moved[y][x] = img_prv[y][x-delta_x] 
                    else:
                        mask[y][x] = [255,255,255]
                        img_prv_moved[y][x] = [255,255,255]
        cv2.imwrite(self.rospack.get_path('baxter_vision_example') + u"/data/img_prv_moved.jpg",img_prv_moved)
        return mask

if __name__ =='__main__':
    mask_generator = mask_generate(sys.argv[1])
