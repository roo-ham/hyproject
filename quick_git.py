#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
os.system("git pull")
os.system("git add .")
print("\n"*5)
print("LISTEN!!!")
print("if you didn't edit anything, just press Ctrl+C")
print("What have you done? : ", end="")
commit_reason = input()
os.system("git commit -m \"%s\""%commit_reason)
os.system("git push")