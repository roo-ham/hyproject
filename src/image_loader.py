#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int8MultiArray
from rospy.numpy_msg import numpy_msg
import numpy as np

pub_1 = rospy.Publisher("/hyproject/image_loader/bgr_top", numpy_msg(Int8MultiArray), queue_size=10)
pub_2 = rospy.Publisher("/hyproject/image_loader/bgr_bottom", numpy_msg(Int8MultiArray), queue_size=10)
def callback(_data):
    
    print(list(_data.data)[0])
if __name__ == "__main__":
    rospy.init_node("hyproject_image_loader")
    rospy.Subscriber("/camera/rgb/image_raw", Image, callback)
    try:
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass