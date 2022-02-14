#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np 
from math import *
from geometry_msgs.msg import Pose

ball_color = 'green'

color_dist = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              'green': {'Lower': np.array([35, 43, 35]), 'Upper': np.array([90, 255, 255])},
              'all': {'Lower': np.array([0, 0, 0]), 'Upper': np.array([255, 255, 255])},
            }

class Image_converter:
    def __init__(self):
        self.bridge = CvBridge()

        self.image_pub = rospy.Publisher('colorcube_detect',Image,queue_size = 10)

        self.image_sub = rospy.Subscriber('/unicycle/camera1/image_raw',Image,self.callback)

    def callback(self, data):
        try:
            #将相机数据转化为cv可以处理的np格式
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
            
        except CvBridgeError as e:
                print e
        detect_image = self.detect_table(cv_image)
        try:
            #发布处理后的图片
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(detect_image, "bgr8"))
        except CvBridgeError as e:
                print e

    def detect_table(self,image):
        #高斯滤波降噪
        g_image = cv2.GaussianBlur(image, (5, 5), 0)
        #RGB空间转为HSV空间
        hsv = cv2.cvtColor(g_image, cv2.COLOR_BGR2HSV)
        #腐蚀消除噪点
        erode_hsv = cv2.erode(hsv, None, iterations=2)
        #筛选指定颜色的区域
        inRange_hsv = cv2.inRange(erode_hsv, color_dist[ball_color]['Lower'], color_dist[ball_color]['Upper'])
        #提取该区域的轮廓
        contours = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        for i in contours:
            x, y, w, h = cv2.boundingRect(i)
            #画图框出该区域
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255,255), 3)
        return image

if __name__ == "__main__":
    rospy.init_node("vision_manager")
    rospy.loginfo("start")
    Image_converter()
    rospy.spin()

