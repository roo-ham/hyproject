#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, time
os.system("cd ~/1273_ws")
time.sleep(1)
os.system("source devel/setup.bash")
os.system("roslaunch hyproject start.launch")