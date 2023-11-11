#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge

class Basement:
    def __init__(self, name0:str = "release"):
        self.name = name0
        self.__bgr_full = np.zeros((256,256,3), np.uint8)
        self.__bgr_bottom = np.zeros((128,256,3), np.uint8)
        self.img_h, self.img_s, self.img_v = np.zeros((128,256), np.uint8),\
            np.zeros((128,256), np.uint8),\
                np.zeros((128,256), np.uint8)
        self.sub_image_raw = None
        self.sub_marker = None
        self.start()
    def update(self):
        self.timeout -= 1
        if self.timeout < 0 :
            self.restart()
    def get_bgr_full(self) -> np.ndarray:
        return self.__bgr_full.copy()
    def get_bgr_bottom(self) -> np.ndarray:
        return self.__bgr_bottom.copy()

    def restart(self):
        print("restarting...")
        del self.sub_image_raw
        del self.sub_marker
        rospy.signal_shutdown("restarting hyproject...")
    def start(self):
        self.timeout = 60

    def image_raw_callback(self, data):
        self.timeout = 60
        bridge = CvBridge()
        origin = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        origin = cv2.resize(origin, (256, 256), interpolation=cv2.INTER_NEAREST)
        self.__bgr_full = origin
        self.__bgr_bottom = origin[128:256, :, :]
        img_hsv = cv2.cvtColor(origin[128:256, :, :], cv2.COLOR_BGR2HSV)
        self.img_h, self.img_s, self.img_v = img_hsv[:, :, 0], img_hsv[:, :, 1], img_hsv[:, :, 2]
    def marker_callback(self, data):
        pass