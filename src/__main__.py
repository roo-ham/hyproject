#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
import time

from vision import VisionImage, VisionMarker
from lidar import Lidar
from motor import Motor
from basement import Basement

# 클래스 생성
class Main:
    def __init__(self, base:Basement):
        self.basement = base
        self.basement.roslaunch("camera.launch")
        self.rate = rospy.Rate(30)
        time.sleep(2)
        self.vision_image = VisionImage(base)
        self.vision_marker = VisionMarker(base)
        self.lidar = Lidar(base)
        self.motor = Motor(base)
    def update(self):
        if self.vision_image.timeout < 0 or self.vision_marker.timeout < 0 :
            self.restart()
        self.vision_image.update()
        self.vision_marker.update()
        self.motor.update()
        self.basement.tick += 1
        self.rate.sleep()
    def end(self):
        for l in self.basement.launch.values():
            l.shutdown()
    def restart(self):
        print("restarting...")
        rospy.signal_shutdown("restarting hyproject...")

base = Basement()
try:
    rospy.init_node("hyproject_main")
    main_object = Main(base)
    os.system("clear")
    print("Hello, Hanyang!")
    print("Ctrl+C to exit.")
    while not rospy.is_shutdown():
        main_object.update()
    main_object.end()
except KeyboardInterrupt:
    pass


