#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from rooham.timer import *
from basement import Basement
from module import TaskModule

class TPark(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "TPark")
        pass
    def update(self):
        pass