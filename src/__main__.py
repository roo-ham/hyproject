#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from basement import Basement

# 클래스 생성
class Main:
    def __init__(self):
        print("hi")


if __name__ == "__main__":
    # 클래스 객체 생성
    Main()
    try:
        # 함수를 호출하여 ROS 노드를 실행하고, 메시지를 수신하는 동안 무한 루프
        rospy.spin()
    # 예외가 발생하면, 예외를 무시하고 프로그램 종료
    except rospy.ROSInterruptException:
        pass
