import numpy as np
import matplotlib.pyplot as plt
import random

from ..basement import Basement
from ..module import TaskModule

class Sign(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "Sign")

class Arrow(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "Arrow")

class TPark(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "TPark")

class Ramp(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "Ramp")
    
class Wall(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "Wall")

    def update(self, orthogonal_pos, polar_pos):
        self.weight_x = 0.0
        self.weight_z = 0.0
        self.x = 0.0
        self.z = 0.0

        in_range_pos = []
        for p1, p2 in zip(polar_pos, orthogonal_pos):
            if p1[0] > 0.35 or abs(p1[1]) > 0.05:
                continue
            in_range_pos.append(p2)

        if len(in_range_pos) == 0:
            return

        number_of_point = len(in_range_pos)
        mean_pos = sum(in_range_pos)/number_of_point
        sum_tangent = 0.0

        for p2 in in_range_pos:
            relative_point = p2 - mean_pos
            if relative_point[1] == 0:
                number_of_point -= 1
                continue
            sum_tangent += relative_point[0]/relative_point[1]
        
        if number_of_point == 0:
            return
        atan = np.arctan(sum_tangent / number_of_point)

        delta_x = -1.5
        delta_z = atan
        
        k = np.pi * 0.5
        self.weight_x = 2.0
        self.weight_z = 1 - np.sqrt(abs(atan / k))

        self.x, self.z = delta_x, delta_z

class Lane(TaskModule):
    def __init__(self, base:Basement) -> None:
        super().__init__(base, "Wall")
        b_height = base.bottom_height
        self.mask_global_x = np.arange(-128, 128)
        self.mask_global_y = np.arange(128-b_height, 128)
        self.mask_local = np.arange(-2, 3)
        self.timescale_dataset = np.zeros((60,4), np.float32)
        self.weight_x = 1.0
        self.weight_z = 1.0
        self.x_data = range(60)
        self.timer = 0.0

        self.fig, self.axes = plt.subplots(nrows=4)
        styles = ['r-', 'g-', 'y-', 'b-']
        labels = ['G tan', 'L tan', 'L tan (absolute)', 'Integral Timer']
        def plot(ax, style, label):
            plot = ax.plot(self.x_data, self.timescale_dataset[:, 0], style, animated=True, label=label)[0]
            ax.set_xlim(0, 59)
            ax.set_ylim(-1.6, 1.6)
            ax.legend()
            return plot
        self.lines = [plot(ax, style, label) for ax, style, label in zip(self.axes, styles, labels)]
        self.fig.show()
        self.fig.canvas.draw()
        self.backgrounds = [self.fig.canvas.copy_from_bbox(ax.bbox) for ax in self.axes]

    def append_latest_data(self, *data_tuple):
        for key, value in enumerate(data_tuple):
            if type(value) != type(None):
                self.timescale_dataset[1:60, key] = self.timescale_dataset[0:59, key]
                self.timescale_dataset[0, key] = value

    def show_dataset_graph(self):
        items = enumerate(zip(self.lines, self.axes, self.backgrounds), start=0)
        for j, (line, ax, background) in items:
            self.fig.canvas.restore_region(background)
            line.set_ydata(self.timescale_dataset[:, j])
            ax.draw_artist(line)
            self.fig.canvas.blit(ax.bbox)

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
        ltan = np.mean(self.timescale_dataset[0:15, 1])
        prev_ltan = np.mean(self.timescale_dataset[15:30, 1])
        not_center = abs(gtan) > 0.3
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
            gtan = self.get_global_tangent(identity_size, yellow)
            ltan, ltan_abs = self.get_local_tangent(identity_size, yellow)
        if self.on_curve_transition(gtan):
            gtan = None
        self.append_latest_data(gtan, ltan, ltan_abs, self.timer)
        gtan, ltan, ltan_abs = self.timescale_dataset[0, 0:3]

        if self.timer > 0:
            # 타이머가 작동하는 경우 적분을 이용하여 현재 속력만큼 타이머 숫자를 줄인다.
            self.timer -= real_speed[0] / 30

        # 실시간으로 데이터베이스를 그래프로 보여준다.
        self.show_dataset_graph()

        # 커브를 발견하면 2.2m 타이머 시작
        if self.found_junction(gtan) and self.timer <= 0:
            self.pause_until(2.2)
        
        # 차선이 수평하면 (휘어있으면) 속도 줄임
        # 그렇지 않으면 (곧으면) 속도 늘림
        delta_x = 1.0

        # 차선이 한쪽으로 치우쳐져 있어 global_tan이 0이 아니면 회전
        delta_z = gtan * 0.5
        
        # 급커브 처리
        arc_offset = 0.0
        if self.timer > 0.5 :
            delta_x += 1.0
            arc_offset = 0.4
        elif self.timer > 0.0:
            arc_offset = 0.2
        else:
            if identity_size == 0:
                delta_z = 0
        
        if (gtan > 0):
            delta_z -= arc_offset
        elif (gtan < 0):
            delta_z += arc_offset

        self.x, self.z = delta_x, delta_z
        
    def get_global_tangent(self, identity_size, yellow:np.ndarray) -> float:
        x_set = yellow * self.mask_global_x
        y_set = ((yellow.T) * self.mask_global_y).T
        x_set, y_set = np.where(x_set, x_set, 1), np.where(x_set, y_set, 1000*y_set)
        return np.arctan(np.sum(y_set/x_set) / identity_size)

    def get_local_tangent(self, identity_size, yellow:np.ndarray) -> tuple:
        l_tan = 0.0
        l_tan_abs = 0.0
        arange = self.mask_local
        for x, y in np.argwhere(yellow):
            if random.randint(1, identity_size) > 32:
                continue
            base = yellow[-2+x:3+x, -2+y:3+y].copy()
            base[:, 2] = 0
            x_set = base * arange
            y_set = (base.T * arange).T
            x_set, y_set = np.where(x_set, x_set, 1), np.where(x_set, y_set, 1000*y_set)
            identity_size_local = np.sum(base)
            if identity_size_local == 0:
                continue
            atan = np.arctan(y_set/x_set)
            atan_abs = np.abs(atan)
            l_tan += np.sum(atan) / identity_size_local
            l_tan_abs += np.sum(atan_abs) / identity_size_local
        l_tan /= identity_size
        l_tan_abs /= identity_size
        return l_tan, l_tan_abs