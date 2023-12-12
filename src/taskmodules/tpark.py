#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from rooham.flag import *
from rooham.timer import *
from basement import Basement
from module import TaskModule

class TPark(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "TPark")
        self.phase = "ready"
    def update(self):
        if is_not_flag("tpark"):
            self.weight_x = 0.0
            self.weight_z = 0.0
        else:
            self.weight_x = 10.0
            self.weight_z = 10.0