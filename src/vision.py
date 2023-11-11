#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2
from basement import Basement
from sensor_msgs.msg import CompressedImage
from ar_track_alvar_msgs.msg import AlvarMarkers

class VisionImage:
    def __init__(self, base:Basement):
        self.basement = base
        self.vision_image.sub_image_raw = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.callback, queue_size=1)
        print("I'm VisionImage")
    def get_yellow(self):
        under_yellow = self.basement.img_h < 15
        over_yellow = self.basement.img_h > 35
        return ~(under_yellow | over_yellow)
    def get_white(self):
        over_sat = self.basement.img_s > 230
        over_bri = self.basement.img_v > 230
        return (over_sat & over_bri)
    def get_black(self):
        over_sat = self.basement.img_s > 230
        over_bri = self.basement.img_v < 50
        return (over_sat & over_bri)
    def update(self):
        img = self.basement.get_bgr_bottom()
        yellow = self.get_yellow()
        white = self.get_white()
        black = self.get_black()
        img[:, :, 0] = np.where(yellow, 20, img[:, :, 0])
        img[:, :, 1] = np.where(yellow, 255, img[:, :, 1])
        img[:, :, 2] = np.where(yellow, 255, img[:, :, 2])

        img[:, :, 1] = np.where(white, 0, img[:, :, 1])
        img[:, :, 2] = np.where(white, 255, img[:, :, 2])

        img[:, :, 1] = np.where(black, 0, img[:, :, 1])
        img[:, :, 2] = np.where(black, 0, img[:, :, 2])
        cv2.namedWindow("hyproject", cv2.WINDOW_NORMAL)
        cv2.imshow("hyproject", img)
        cv2.waitKey(1)
    def callback(self, data):
        self.timeout = 60
        bridge = CvBridge()
        origin = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        origin = cv2.resize(origin, (256, 256), interpolation=cv2.INTER_NEAREST)
        self.__bgr_full = origin
        self.__bgr_bottom = origin[128:256, :, :]
        img_hsv = cv2.cvtColor(origin[128:256, :, :], cv2.COLOR_BGR2HSV)
        self.img_h, self.img_s, self.img_v = img_hsv[:, :, 0], img_hsv[:, :, 1], img_hsv[:, :, 2]

class VisionMarker:
    def __init__(self, base:Basement):
        self.basement = base
        self.vision_image.sub_marker = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)
        print("I'm VisionMarker")
    def callback(self, data):
        pass