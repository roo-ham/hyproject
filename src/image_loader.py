#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt8MultiArray

rospy.init_node("hyproject_image_loader")
pub_1 = rospy.Publisher("/hyproject/image_loader/bgr_top", UInt8MultiArray, queue_size=10)
pub_2 = rospy.Publisher("/hyproject/image_loader/bgr_bottom", UInt8MultiArray, queue_size=10)
def callback(data):
    bridge = CvBridge()
    origin = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    origin = cv2.resize(origin, (256, 256), interpolation=cv2.INTER_NEAREST)
    bgr_top = origin[128:256, :, :]
    bgr_bottom = origin[0:128, :, :]
    pub_1.publish(bgr_top)
    pub_1.publish(bgr_bottom)
rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, callback)
    

if __name__ == "__main__":
    try:
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass