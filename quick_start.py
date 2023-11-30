#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
os.system("bash -c 'pip3 install defusedxml netifaces pathlib'")
os.system("bash -c 'rosnode kill -a'")
os.system("bash -c 'source ../../devel/setup.bash && roslaunch launch/start.launch'")
print("quick_start.py가 종료되었습니다.")
