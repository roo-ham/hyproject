#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
from math import sin
'''
from sensor_msgs.msg import LaserScan

'''
from geometry_msgs.msg import Twist
from vision import VisionImage, VisionMarker
from basement import Basement

# 클래스 생성
class Main:
    def __init__(self, base:Basement):
        self.basement = base
        self.vision_image = VisionImage(base)
        self.vision_marker = VisionMarker(base)
        #rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.drive_data = Twist()
        self.rate = rospy.Rate(60)
        self.t = 0.0
    def update(self):
        self.basement.update()
        self.vision_image.update()
        self.t += 0.1
        self.drive_data.linear.x = sin(self.t)
        self.pub.publish(self.drive_data)
        self.rate.sleep()


if __name__ == "__main__":
    os.system("clear")
    print("Hello, Hanyang!")
    print("Ctrl+C to exit.")
    base = Basement()
    try:
        while True:
            main_object = Main(base)
            rospy.init_node("hyproject_main")
            base.start()
            while not rospy.is_shutdown():
                main_object.update()
    except KeyboardInterrupt:
        exit()
