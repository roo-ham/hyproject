#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2
from basement import Basement
from submodule import Submodule
from sensor_msgs.msg import CompressedImage
from ar_track_alvar_msgs.msg import AlvarMarkers
from cv_bridge import CvBridge

class VisionImage(Submodule):
    def __init__(self, base:Basement):
        super().__init__(base, "VisionImage")
        self.sub_image_raw = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.callback)
    def get_yellow(self):
        under_yellow = self.basement.img_h < 15
        over_yellow = self.basement.img_h > 35
        return ~(under_yellow | over_yellow)
    def get_thick_h(self, original):
        thick_h = np.zeros((128,256), bool) | original
        n = 2
        for k in range(n + 1):
            thick_h[:, k:256-n+k] |= original[:, n-k:256-k]
            thick_h[:, n-k:256-k] |= original[:, k:256-n+k]
        return thick_h
    def get_thick_v(self, original):
        thick_v = np.zeros((128,256), bool) | original
        n = 2
        for k in range(n + 1):
            thick_v[k:128-n+k, :] |= original[n-k:128-k, :]
            thick_v[n-k:128-k, :] |= original[k:128-n+k, :]
        return thick_v
    def get_yellow_point(self, yellow):
        yellow_thick_h = self.get_thick_h(yellow)
        horizonal = np.zeros((128,256), bool) | yellow
        horizonal[0:127, :] &= ~yellow_thick_h[1:128, :]

        yellow_thick_v = self.get_thick_v(horizonal)
        vertical = np.zeros((128,256), bool) | horizonal
        vertical[:, 0:255] &= ~yellow_thick_v[:, 1:256]

        return vertical
    
    def get_white(self):
        over_sat = self.basement.img_s < 64
        over_bri = self.basement.img_v > 150
        return (over_sat & over_bri)
    def get_black(self):
        over_sat = self.basement.img_s < 128
        over_bri = self.basement.img_v < 150
        return (over_sat & over_bri)
    def update(self):
        super().update()
        img = self.basement.get_bgr_bottom()
        img[:, :, :] = 0

        black = self.get_black()
        white = self.get_white()
        y1 = self.get_yellow()&(~black)&(~white)
        y1[0:124, 0:252] &= y1[4:128, 4:256]
        #y1[4:128, 4:256] |= y1[0:124, 0:252]
        #y1[0:127, 0:255] &= ~y1[1:128, 1:256]
        y1[0:8, :], y1[120:128, :], y1[:, 0:8], y1[:, 248:256] = False, False, False, False

        a = self.get_yellow_point(y1)

        img[:, :, 0] = np.where(y1, 0, img[:, :, 0])
        img[:, :, 1] = np.where(y1, 255, img[:, :, 1])
        img[:, :, 2] = np.where(y1, 255, img[:, :, 2])

        img[:, :, 0] = np.where(a, 0, img[:, :, 0])
        img[:, :, 1] = np.where(a, 0, img[:, :, 1])
        img[:, :, 2] = np.where(a, 255, img[:, :, 2])
        cv2.namedWindow("hyproject", cv2.WINDOW_NORMAL)
        cv2.imshow("hyproject", img)
        cv2.waitKey(1)
    def callback(self, data):
        super().callback(data)
        bridge = CvBridge()
        origin = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        origin = cv2.resize(origin, (256, 256))
        self.basement.set_bgr(origin, origin[128:256, :, :])

class VisionMarker(Submodule):
    def __init__(self, base:Basement):
        super().__init__(base, "VisionMarker")
        self.sub_marker = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)