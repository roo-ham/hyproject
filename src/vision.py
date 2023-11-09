#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import UInt8MultiArray
from basement import Basement

class VisionImage:
    def __init__(self, base:Basement):
        self.basement = base
        rospy.Subscriber("/hyproject/image_loader/bgr_top",\
            UInt8MultiArray, self.bgr_top_callback)
        rospy.Subscriber("/hyproject/image_loader/bgr_bottom",\
            UInt8MultiArray, self.bgr_bottom_callback)
        print("I'm VisionImage")
    def bgr_top_callback(self, data):
        self.basement.bgr_top = data
    def bgr_bottom_callback(self, data):
        self.basement.bgr_bottom = data
    def get_yellow(self):
        img = cv2.cvtColor(self.basement.bgr_bottom, cv2.COLOR_BGR2HSV)
        under_yellow = img[:, :, 0] < 15
        over_yellow = img[:, :, 0] > 35
        img[:, :, 0] = np.where(under_yellow, 0, img[:, :, 0])
        img[:, :, 1] = np.where(under_yellow, 50, 255)
        img[:, :, 2] = np.where(under_yellow, 50, 255)
        img[:, :, 0] = np.where(over_yellow, 128, img[:, :, 0])
        img[:, :, 2] = np.where(over_yellow, 50, 255)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cv2.namedWindow("hyproject", cv2.WINDOW_NORMAL)
        cv2.imshow("hyproject", img)
        cv2.waitKey(1)
        return ~(under_yellow & over_yellow)

class VisionMarker:
    def __init__(self, base:Basement):
        self.basement = base
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)
        print("I'm VisionMarker")
    def callback(self, data):
        pass