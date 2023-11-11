#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from cv_bridge import CvBridge
import cv2
from ar_track_alvar_msgs.msg import AlvarMarkers
from basement import Basement
from sensor_msgs.msg import CompressedImage

class VisionImage:
    def __init__(self, base:Basement):
        self.basement = base
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.callback)
        print("I'm VisionImage")
    def callback(self, data):
        bridge = CvBridge()
        origin = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        origin = cv2.resize(origin, (256, 256), interpolation=cv2.INTER_NEAREST)
        self.basement.bgr_top = origin[0:128, :, :]
        self.basement.bgr_bottom = origin[128:256, :, :]
    def get_yellow(self):
        img = cv2.cvtColor(self.basement.bgr_bottom, cv2.COLOR_BGR2HSV)
        under_yellow = img[:, :, 0] < 15
        over_yellow = img[:, :, 0] > 35
        img[:, :, 0] = np.where(under_yellow, 0, img[:, :, 0])
        img[:, :, 2] = np.where(under_yellow, 50, img[:, :, 2])
        img[:, :, 0] = np.where(over_yellow, 128, img[:, :, 0])
        img[:, :, 2] = np.where(over_yellow, 50, img[:, :, 2])
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