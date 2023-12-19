#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from rooham.flag import *
from rooham.timer import *
from basement import Basement
from module import TaskModule

class TPark(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "TPark")
        self.phase = (0, "ready")
        self.phase_list = [("ready", True),
                           ("approach", True),
                           ("align", False, 2.3, -0.8, 0),
                           ("enter_1", False, 2, 0, -0.78),
                           ("enter_2", False, 2.5, 0.8, 0),
                           ("done", False, 100, 0, 0),
                           ]
    def set_phase_from_id(self, phase_id):
        tpl = self.phase_list[phase_id]
        if not tpl[1]:
            set_timer("tpark/action", tpl[2])
        self.phase = (phase_id, tpl[0])
    def debug_tpark(self):
        return "tpark_phase : %d, %s\n"%self.phase
    def update(self):
        if is_timer_on("marker/stop/phase1"):
            self.weight_x = 0
            self.weight_z = 0
            return

        tpl = self.phase_list[self.phase[0]]
        if tpl[1]:
            pass
        elif is_timer_off("tpark/action"):
            self.set_phase_from_id(self.phase[0]+1)
        else:
            self.x = tpl[3]
            self.z = tpl[4]
            return
        
        phase_name = self.phase[1]
        if phase_name == "ready":
            pass
        elif phase_name == "approach":
            self.x = 0.8
            self.z = 0
            if is_flag("tpark/approach/end"):
                self.set_phase_from_id(2)

        if is_not_flag("tpark"):
            self.weight_x = 0.0
            self.weight_z = 0.0
        else:
            self.weight_x = 1.0
            self.weight_z = 1.0