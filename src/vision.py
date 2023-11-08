#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from cv_bridge import CvBridge
import numpy, cv2
from sensor_msgs.msg import CompressedImage
from ar_track_alvar_msgs.msg import AlvarMarkers

from basement import Basement

class VisionImage:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.callback)
        self.img = None
        print("I'm VisionImage")
    def callback(self, data):
        self.img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        self.img = cv2.resize(self.img, (128, 128), interpolation=cv2.INTER_NEAREST)
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        self.img[:, :, 1:3] = 255
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