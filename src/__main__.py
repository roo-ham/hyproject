#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os

from rooham.flag import debug_flags
from rooham.timer import debug_timers
from basement import Basement
from iomodules import *
from taskmodules import *

# 클래스 생성
class Main:
    def __init__(self, base:Basement):
        self.basement = base
        self.basement.roslaunch("camera.launch")
        self.rate = rospy.Rate(30)
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
        if self.basement.tick % 10 == 0:
            lane:Lane = self.basement.taskmodules["Lane"]

            os.system("clear")
            debug_text = ""
            debug_text += self.vision_marker.debug_markers()
            debug_text += debug_timers()
            debug_text += debug_flags()
            debug_text += lane.debug_junction()
            print(debug_text)

    def restart(self):
        print("restarting...")
        for l in self.basement.launch.values():
            l.shutdown()
        rospy.signal_shutdown("restarting hyproject...")

base = Basement()
try:
    rospy.init_node("hyproject_main")
    Lane(base)
    Wall(base)
    TPark(base)
    main_object = Main(base)
    os.system("clear")
    print("Hello, Hanyang!")
    print("Ctrl+C to exit.")
    while not rospy.is_shutdown():
        main_object.update()
except KeyboardInterrupt:
    pass


