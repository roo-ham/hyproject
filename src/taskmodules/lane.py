#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt

from rooham.timer import *
from rooham.mathtools import get_global_tangent, get_local_tangent
from basement import Basement
from module import TaskModule

class Lane(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "Lane")
        b_height = base.bottom_height
        self.mask_global_x = np.arange(-128, 128)
        self.mask_global_y = np.arange(128-b_height, 128)
        self.mask_local = np.arange(-2, 3)
        self.timescale_dataset = np.zeros((60,3), np.float32)
        self.x_data = range(60)
        self.junction_curve_direction = ""
        self.on_waiting_curve = False

        self.fig, self.axes = plt.subplots()
        styles = ['r-', 'g-', 'y-']
        labels = ['G tan', 'L tan', 'L tan (absolute)']
        self.lines = []
        for style, label in zip(styles, labels):
            plot = self.axes.plot(self.x_data, self.timescale_dataset[:, 0], style, animated=True, label=label)[0]
            self.axes.set_xlim(0, 59)
            self.axes.set_ylim(-1.6, 1.6)
            self.axes.legend()
            self.lines.append(plot)
        self.fig.show()
        self.fig.canvas.draw()
        self.backgrounds = self.fig.canvas.copy_from_bbox(self.axes.bbox)

    def append_latest_data(self, *data_tuple):
        for key, value in enumerate(data_tuple):
            if type(value) != type(None):
                self.timescale_dataset[1:60, key] = self.timescale_dataset[0:59, key]
                self.timescale_dataset[0, key] = value

    def show_dataset_graph(self):
        items = enumerate(self.lines)
        self.fig.canvas.restore_region(self.backgrounds)
        for j, line in items:
            line.set_ydata(self.timescale_dataset[:, j])
            self.axes.draw_artist(line)
        self.fig.canvas.blit(self.axes.bbox)
    
    def on_curve_transition(self, gtan):
        if gtan == None:
            gtan = 0
        if self.timescale_dataset[0, 0] * gtan > 0:
            return False
        elif abs(self.timescale_dataset[0, 0]) < 1.0:
            return False
        return True

    def clean_junction_curve(self):
        if not self.on_waiting_curve:
            self.junction_curve_direction = ""

    def do_junction_curve(self, offset) -> float:
        angle = 0
        if self.junction_curve_direction == "":
            return angle

        self.on_waiting_curve = False
        if self.junction_curve_direction == "left":
            angle = -1.0
        elif self.junction_curve_direction == "right":
            angle = 1.0

        return angle

    def update(self, identity_size, yellow:np.ndarray):
        if is_timer_running("marker/stop/phase1"):
            self.weight_x = 0
            self.weight_z = 0
            return
        
        # 차선의 형태를 계산한다, 그리고 하나의 데이터로 만든다.
        # 어떤 조건을 불만족 하는 경우 이전(previous) 데이터를 계속 사용한다.
        # 데이터베이스에 데이터들을 나열한다.
        gtan, ltan, ltan_abs = None, None, None
        if (identity_size > 0):
            gtan = get_global_tangent(self.mask_global_x, self.mask_global_y, identity_size, yellow)
            ltan, ltan_abs = get_local_tangent(self.mask_local, identity_size, yellow)
        if not self.on_curve_transition(gtan):
            pass
        else:
            gtan = None
        if is_timer_running("lane/lane_exception/left"):
            if gtan == None:
                gtan = -np.pi/2
            gtan = -np.pi/2 if gtan > 0 else gtan
        elif is_timer_running("lane/lane_exception/right"):
            if gtan == None:
                gtan = np.pi/2
            gtan = np.pi/2 if gtan < 0 else gtan
        elif gtan == None and ltan == None:
            set_timer("lane/ramp", 1.5, True)
        self.append_latest_data(gtan, ltan, ltan_abs)
        is_none = (gtan == None, ltan == None, ltan_abs == None)
        gtan, ltan, ltan_abs = self.timescale_dataset[0, :]

        # 실시간으로 데이터베이스를 그래프로 보여준다.
        self.show_dataset_graph()
        
        delta_x = 1.0
        delta_z = gtan - ltan
        
        # 급커브 처리
        if abs(gtan) < 0.1:
            delta_x = 2.0
            delta_z = gtan
            self.clean_junction_curve()
        elif abs(delta_z) > 0.2 and abs(gtan) < 1.1:
            delta_z = self.do_junction_curve(gtan - ltan)
        else:
            delta_x = 1.5
            delta_z = (gtan - (ltan*0.5))/1.25
            if is_none[1]:
                delta_z = gtan/1.1

        if self.junction_curve_direction != "" and (not self.on_waiting_curve):
            pass
        elif is_timer_running("lane/ramp"):
            delta_x = 0.8
            delta_z = 0

        self.weight_x = 1.0
        self.weight_z = delta_z**2

        if not is_timer_running("wall/obstacle_ignore"):
            self.weight_x = 0.0

        self.x, self.z = delta_x, delta_z