#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
'''
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
'''

from vision import *
from basement import Basement

# 클래스 생성
class Main:
    def __init__(self):
        rospy.init_node("hyproject_node")
        self.vision_image = VisionImage()
        self.vision_marker = VisionMarker()
        #rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        #self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #self.drive_data = Twist()


if __name__ == "__main__":
    Main()
    try:
        while not rospy.is_shutdown():
            os.system("clear")
            print("Hello, Hanyang!")
            print("Ctrl+C to exit.")
            rospy.sleep(0.01)
    except rospy.ROSInterruptException:
        pass
