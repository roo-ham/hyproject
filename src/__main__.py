#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
'''
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
'''

from vision import VisionImage, VisionMarker
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
        self.rate = rospy.Rate(10)
    def update(self):
        self.rate.sleep()


if __name__ == "__main__":
    os.system("clear")
    print("Hello, Hanyang!")
    print("Ctrl+C to exit.")
    main_object = Main()
    try:
        while not rospy.is_shutdown():
            main_object.update()
    except rospy.ROSInterruptException:
        pass
