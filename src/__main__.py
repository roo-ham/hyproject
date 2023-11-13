#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, roslaunch
import os
import time
from math import sin
'''
from sensor_msgs.msg import LaserScan

'''
from geometry_msgs.msg import Twist
from vision import VisionImage, VisionMarker
from basement import Basement
from pathlib import Path

# 클래스 생성
class Main:
    def __init__(self, base:Basement):
        self.basement = base
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        path = Path(os.path.abspath(__file__)).parent.parent.joinpath("launch/camera.launch")
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [str(path)])
        self.launch.start()
        self.rate = rospy.Rate(30)
        time.sleep(5)
        
        self.vision_image = VisionImage(base)
        self.vision_marker = VisionMarker(base)
        #rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.drive_data = Twist()
        self.t = 0.0
    def update(self):
        if self.vision_image.timeout < 0 or self.vision_marker.timeout < 0 :
            self.restart()
        self.vision_image.update()
        self.drive_data.linear.x = 0.3
        self.drive_data.angular.z = 0.0
        
        gtan = self.basement.global_tan
        ltan = self.basement.local_tan
        ltan2 = self.basement.local_tan_sqaured
        OFFSET_CONSTANT = 0
        
        self.drive_data.linear.x *= ltan2 + 0.5
        self.drive_data.angular.z += gtan / ((ltan**2) + 0.1) + OFFSET_CONSTANT
        self.pub.publish(self.drive_data)
        print(self.drive_data.angular.z)
        self.rate.sleep()
        self.t += 1/60
    def end(self):
        self.launch.shutdown()
    def restart(self):
        print("restarting...")
        rospy.signal_shutdown("restarting hyproject...")

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


