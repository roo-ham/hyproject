#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt

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
        self.timescale_dataset = np.zeros((60,4), np.float32)
        self.x_data = range(60)
        self.timer = 0.0

        self.fig, self.axes = plt.subplots()
        styles = ['r-', 'g-', 'y-', 'b-']
        labels = ['G tan', 'L tan', 'L tan (absolute)', 'Integral Timer']
        self.lines = []
        for style, label in zip(styles, labels):
            plot = self.axes.plot(self.x_data, self.timescale_dataset[:, 0], style, animated=True, label=label)
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
        items = enumerate(self.axes)
        for j, line in items:
            self.fig.canvas.restore_region(self.backgrounds)
            line.set_ydata(self.timescale_dataset[:, j])
            self.axes.draw_artist(line)
            self.fig.canvas.blit(self.axes.bbox)

    def pause_until(self, t):
        self.timer = t

    def resume(self):
        self.timer = -1.0
    
    def on_curve_transition(self, gtan):
        if gtan == None:
            return False
        elif self.timescale_dataset[0, 0] * gtan >= 0:
            return False
        elif abs(gtan) < 0.4:
            return False
        return True
    
    def found_junction(self, gtan):
        ltan = np.mean(self.timescale_dataset[0:20, 1])
        prev_ltan = np.mean(self.timescale_dataset[20:40, 1])
        not_center = abs(gtan) > 1.2
        lane_as_one = abs(gtan-ltan) < 0.3
        diff_ltan_high = abs(ltan-prev_ltan) > 0.3
        if not_center & lane_as_one & diff_ltan_high:
            return True
        return False

    def update(self, real_speed, identity_size, yellow:np.ndarray):
        # 차선의 형태를 계산한다, 그리고 하나의 데이터로 만든다.
        # 어떤 조건을 불만족 하는 경우 이전(previous) 데이터를 계속 사용한다.
        # 데이터베이스에 데이터들을 나열한다.
        gtan, ltan, ltan_abs = None, None, None
        if (identity_size > 0):
            gtan = get_global_tangent(self.mask_global_x, self.mask_global_y, identity_size, yellow)
            ltan, ltan_abs = get_local_tangent(self.mask_local, identity_size, yellow)
        if self.on_curve_transition(gtan):
            gtan = None
        self.append_latest_data(gtan, ltan, ltan_abs, self.timer)
        gtan, ltan, ltan_abs = self.timescale_dataset[0, 0:3]

        #if self.timer > 0:
            # 타이머가 작동하는 경우 적분을 이용하여 현재 속력만큼 타이머 숫자를 줄인다.
            # self.timer -= real_speed[0] / 30

        # 실시간으로 데이터베이스를 그래프로 보여준다.
        self.show_dataset_graph()

        # 커브를 발견하면 2.0m 타이머 시작
        #if self.found_junction(gtan) and self.timer <= 0:
        #    self.pause_until(1.5)
        
        # 차선이 수평하면 (휘어있으면) 속도 줄임
        # 그렇지 않으면 (곧으면) 속도 늘림
        delta_x = 1.0

        # 차선이 한쪽으로 치우쳐져 있어 global_tan이 0이 아니면 회전
        delta_z = gtan - ltan
        
        # 급커브 처리
        if abs(delta_z) < 0.5:
            delta_z = 0

        self.weight_x = 1.0
        self.weight_z = delta_z**2

        self.x, self.z = delta_x, delta_z