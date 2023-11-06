#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from cv_bridge import CvBridge
import numpy, cv2
import os
'''
from sensor_msgs.msg import Image
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
'''
from basement import Basement

# 클래스 생성
class Main:
    def __init__(self):
        rospy.init_node("hyproject_node")
        #self.bridge = CvBridge()
        #rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_CB)
        #rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.marker_CB)
        #rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        #self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #self.drive_data = Twist()
        #self.basic_speed = 0.3
        os.system("clear")
        print("Hello, Hanyang!")
        print("Ctrl+C to exit.")


if __name__ == "__main__":
    Main()
    try:
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass
