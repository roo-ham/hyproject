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
    def get_white(self):
        over_sat = self.basement.img_s > 230
        over_bri = self.basement.img_v > 230
        return (over_sat & over_bri)
    def get_black(self):
        over_sat = self.basement.img_s > 230
        over_bri = self.basement.img_v < 50
        return (over_sat & over_bri)
    def update(self):
        super().update()
        img = self.basement.get_bgr_bottom()
        yellow = self.get_yellow()
        white = self.get_white()
        black = self.get_black()
        img[:, :, 1] = 0

        img[:, :, 0] = np.where(yellow, 20, img[:, :, 0])
        img[:, :, 1] = np.where(yellow, 255, img[:, :, 1])
        img[:, :, 2] = np.where(yellow, 255, img[:, :, 2])

        img[:, :, 0] = np.where(white, 40, img[:, :, 0])
        img[:, :, 1] = np.where(white, 255, img[:, :, 1])
        img[:, :, 2] = np.where(white, 255, img[:, :, 2])

        img[:, :, 0] = np.where(black, 80, img[:, :, 0])
        img[:, :, 1] = np.where(black, 255, img[:, :, 1])
        img[:, :, 2] = np.where(black, 255, img[:, :, 2])
        cv2.namedWindow("hyproject", cv2.WINDOW_NORMAL)
        cv2.imshow("hyproject", img)
        cv2.waitKey(1)
    def callback(self, data):
        super().callback(data)
        bridge = CvBridge()
        origin = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        origin = cv2.resize(origin, (256, 256), interpolation=cv2.INTER_NEAREST)
        self.basement.set_bgr(origin, origin[128:256, :, :])

class VisionMarker(Submodule):
    def __init__(self, base:Basement):
        super().__init__(base, "VisionMarker")
        self.sub_marker = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)