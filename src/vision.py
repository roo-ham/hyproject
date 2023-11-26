#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2
import storage
from basement import Basement
from submodule import Submodule
from sensor_msgs.msg import CompressedImage
from ar_track_alvar_msgs.msg import AlvarMarkers
from cv_bridge import CvBridge

class VisionImage(Submodule):
    def __init__(self, base:Basement):
        super().__init__(base, "VisionImage")
        self.sub_image_raw = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.callback)
        self.lane_storage:storage.Lane = base.storages["lane"]

    def get_yellow(self):
        under_yellow = self.basement.img_h < 13
        over_yellow = self.basement.img_h > 37
        return ~(under_yellow | over_yellow)
    def get_white(self):
        over_sat = self.basement.img_s < 64
        over_bri = self.basement.img_v > 150
        return (over_sat & over_bri)
    def get_black(self):
        over_sat = self.basement.img_s < 128
        over_bri = self.basement.img_v < 150
        return (over_sat & over_bri)
    def get_high_saturation(self):
        over_sat = self.basement.img_s > np.mean(self.basement.img_s)
        return over_sat
    def update(self):
        super().update()

        yellow = self.get_yellow()
        black = self.get_black()
        white = self.get_white()
        yellow = self.get_yellow_border(white, black, yellow)

        #self.display_s(self.get_high_saturation())
        self.display_lane(white, black, yellow)

        identity_size = np.sum(yellow)
        self.lane_storage.update((self.basement.real_speed_x, self.basement.real_speed_z), identity_size, yellow)

    def get_yellow_border(self, white, black, yellow):
        b_height = self.basement.bottom_height
        bw = black | white
        bw[:, 0:254] &= bw[:, 2:256]
        yellow = yellow & ~bw & self.get_high_saturation()
        y2 = np.zeros_like(yellow)
        y2[0:b_height, 0:255] |= yellow[0:b_height, 0:255] ^ yellow[0:b_height, 1:256]
        horizonal = yellow[0:b_height-1, 0:256] ^ yellow[1:b_height, 0:256]
        horizonal[0:b_height, 0:255] &= horizonal[0:b_height, 1:256]
        y2[0:b_height-1, 0:256] |= horizonal
        y2[:, 0:8] = False
        y2[:, 248:256] = False
        y2[0:8, :] = False
        y2[b_height-8:b_height, :] = False
        return y2

    def display_s(self, s):
        img = np.zeros_like(self.basement.get_bgr_bottom())

        img[:, :, 0] = np.where(s, 255, img[:, :, 0])
        img[:, :, 1] = np.where(s, 255, img[:, :, 1])
        img[:, :, 2] = np.where(s, 255, img[:, :, 2])

        cv2.namedWindow("hyproject", cv2.WINDOW_GUI_EXPANDED)
        cv2.imshow("hyproject", img)
        cv2.waitKey(1)

    def display_lane(self, white, black, yellow):
        img = np.zeros_like(self.basement.get_bgr_bottom())

        img[:, :, 0] = np.where(white, 255, img[:, :, 0])
        img[:, :, 1] = np.where(white, 255, img[:, :, 1])
        img[:, :, 2] = np.where(white, 0, img[:, :, 2])

        img[:, :, 0] = np.where(black, 255, img[:, :, 0])
        img[:, :, 1] = np.where(black, 0, img[:, :, 1])
        img[:, :, 2] = np.where(black, 0, img[:, :, 2])

        img[:, :, 0] = np.where(yellow, 0, img[:, :, 0])
        img[:, :, 1] = np.where(yellow, 255, img[:, :, 1])
        img[:, :, 2] = np.where(yellow, 255, img[:, :, 2])

        cv2.namedWindow("hyproject", cv2.WINDOW_GUI_EXPANDED)
        cv2.imshow("hyproject", img)
        cv2.waitKey(1)

    def callback(self, data):
        super().callback(data)
        bridge = CvBridge()
        origin = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        origin = cv2.resize(origin, (256, 256))
        self.basement.set_bgr(origin, origin[256-self.basement.bottom_height:256, :, :])
        self.basement.roslaunch("marker.launch")

class VisionMarker(Submodule):
    def __init__(self, base:Basement):
        super().__init__(base, "VisionMarker")
        self.sub_marker = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)

    def callback(self, data):
        super().callback(data)
        
    def update(self):
        super().update()