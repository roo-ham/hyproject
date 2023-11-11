#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, roslaunch
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
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, ["./launch/camera.launch"])
        self.launch.start()
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
    def end(self):
        self.launch.shutdown()

base = Basement()
try:
    rospy.init_node("hyproject_main")
    os.system("clear")
    print("Hello, Hanyang!")
    print("Ctrl+C to exit.")
    main_object = Main(base)
    while not rospy.is_shutdown():
        main_object.update()
    main_object.end()
except KeyboardInterrupt:
    pass
