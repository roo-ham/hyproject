#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from cv_bridge import CvBridge
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from ar_track_alvar_msgs.msg import AlvarMarkers

from basement import Basement

class VisionImage:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.callback)
        self.origin_top = np.zeros((128,256,3))
        self.origin_bottom = np.zeros((128,256,3))
        print("I'm VisionImage")
    def callback(self, data):
        origin = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        origin = cv2.resize(origin, (256, 256), interpolation=cv2.INTER_NEAREST)
        self.origin_top = origin[128:256, :, :]
        self.origin_bottom = origin[0:128, :, :]
    def get_yellow(self):
        img = cv2.cvtColor(self.origin_bottom, cv2.COLOR_BGR2HSV)
        under_yellow = img[:, :, 0] < 15
        over_yellow = img[:, :, 0] > 35
        yellow = over_yellow & under_yellow
        img[:, :, 0] = np.where(under_yellow, 0, self.img[:, :, 0])
        img[:, :, 1] = np.where(under_yellow, 50, 255)
        self.img[:, :, 0] = np.where(over_yellow, 128, self.img[:, :, 0])
        self.img[:, :, 2] = np.where(over_yellow, 50, 255)
        self.img = cv2.cvtColor(self.img, cv2.COLOR_HSV2BGR)
        cv2.namedWindow("hyproject", cv2.WINDOW_NORMAL)
        cv2.imshow("hyproject", self.img)
        cv2.waitKey(1)

class VisionMarker:
    def __init__(self):
        print("I'm VisionMarker")
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)
    def callback(self, data):
        pass


if __name__ == "__main__":
    VisionImage()
    VisionMarker()
    try:
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass