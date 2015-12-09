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

class mask_compare():

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.rospack = rospkg.RosPack()
        self.img_path = self.rospack.get_path('baxter_vision_example') + u"/data/bg_mask.jpg"
        self.bg_mask_prv = cv2.imread(self.img_path)
        [self.bg_mask_prv_x, self.bg_mask_prv_y] = self.calcGraph(self.bg_mask_prv)
        self.threshold_x = 100
        self.threshold_y = 100
        self.min_x_delta = 0
        self.min_y_delta = 0
        rospy.init_node('mask_compare',anonymous=True)
        self.pub_delta_x = rospy.Publisher('mask_compare/output/delta_x', Int16,queue_size=1)
        self.pub_delta_y = rospy.Publisher('mask_compare/output/delta_y', Int16,queue_size=1)
        rospy.Subscriber("split_fore_background/output/bg_mask",Image,self.callback)
        rospy.spin()

    def callback(self,data):
        self.bg_mask_cur = self.bridge.imgmsg_to_cv2(data,data.encoding)
        [self.bg_mask_cur_x, self.bg_mask_cur_y] = self.calcGraph(self.bg_mask_cur)
        min_x = float('inf')
        min_y = float('inf')
        min_x_delta = 0
        min_y_delta = 0
    
        for delta_y in range(0,self.threshold_y):
            # for negative delta_y
            sub_y_value = 0 
            for cur_y in range(0,len(self.bg_mask_cur_y)-delta_y):
                sub_y_value = sub_y_value + abs(int(self.bg_mask_cur_y[cur_y][0]) - int(self.bg_mask_prv_y[cur_y+delta_y][0]))
            sub_y_value = sub_y_value / (len(self.bg_mask_cur_y)-delta_y)
            if min_y > sub_y_value:
                min_y = sub_y_value
                self.min_y_delta = -delta_y
           # for positive delta_y
            sub_y_value = 0 
            for prv_y in range(0,len(self.bg_mask_prv_y)-delta_y):
                sub_y_value += abs(int(self.bg_mask_cur_y[prv_y+delta_y][0]) - int(self.bg_mask_prv_y[prv_y][0]))
            sub_y_value = sub_y_value / (len(self.bg_mask_prv_y)-delta_y)
            if min_y > sub_y_value:
                min_y = sub_y_value
                self.min_y_delta = delta_y
        for delta_x in range(0,self.threshold_x):
            # for negative delta_x
            sub_x_value = 0 
            for cur_x in range(0,len(self.bg_mask_cur_x)-delta_x):
                sub_x_value += abs(int(self.bg_mask_cur_x[cur_x][0]) - int(self.bg_mask_prv_x[cur_x+delta_x][0]))
            sub_x_value = sub_x_value / (len(self.bg_mask_cur_x)-delta_x)
            if min_x > sub_x_value:
                min_x = sub_x_value
                self.min_x_delta = -delta_x
            # for positive delta_x
            sub_x_value = 0
            for prv_x in range(0,len(self.bg_mask_prv_x)-delta_x):
                sub_x_value += abs(int(self.bg_mask_cur_x[prv_x+delta_x][0]) - int(self.bg_mask_prv_x[prv_x][0]))
            sub_x_value = sub_x_value / (len(self.bg_mask_prv_x)-delta_x)
            if min_x > sub_x_value:
                min_x = sub_x_value
                self.min_x_delta = delta_x 
        self.publish()
        print self.min_x_delta
        print self.min_y_delta

    def calcGraph(self,data):
        graph_x = []
        graph_y = []
        for y in range(0,len(data)):
            graph_y.append(sum(data[y]))
        for x in range (0,len(data[0])):
            sum_x = 0
            for y in range(0,len(data)):
                sum_x += data[y][x]
            graph_x.append(sum_x)
        return [graph_x,graph_y]

    def publish(self):
        self.pub_delta_x.publish(self.min_x_delta)
        self.pub_delta_y.publish(self.min_y_delta)

if __name__ == '__main__':
    comparer = mask_compare()
