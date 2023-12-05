#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from basement import Basement
from module import TaskModule

from .lane import *
from .wall import *

class Sign(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "Sign")

    def update(self):
        pass


class Arrow(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "Arrow")

class TPark(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "TPark")
        self.enabled = False

    def update(self):
        pass