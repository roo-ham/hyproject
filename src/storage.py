import numpy as np
import matplotlib.pyplot as plt
import random

class Storage:
    def __init__(self) -> None:
        self.x = 0.0
        self.z = 0.0
        self.weight_x = 0.0
        self.weight_z = 0.0
    def getDataset(self):
        return self.x, self.z, self.weight_x, self.weight_z

class Sign(Storage):
    def __init__(self) -> None:
        super().__init__()

class Arrow(Storage):
    def __init__(self) -> None:
        super().__init__()

class TPark(Storage):
    def __init__(self) -> None:
        super().__init__()

class Ramp(Storage):
    def __init__(self) -> None:
        super().__init__()
    
class Wall(Storage):
    def __init__(self) -> None:
        super().__init__()

class Lane(Storage):
    def __init__(self, b_height) -> None:
        super().__init__()
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
        self.timescale_dataset[1:60, :] = self.timescale_dataset[0:59, :]
        self.timescale_dataset[0, :] = data_tuple

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
    
    def on_curve_transition(self):
        if self.timescale_dataset[0, 0] * self.timescale_dataset[1, 0] >= 0:
            return False
        elif abs(self.timescale_dataset[0, 0]) < 0.4:
            return False
        return True
    
    def found_junction(self, gtan):
        ltan = np.mean(self.timescale_dataset[0:15, 1])
        prev_ltan = np.mean(self.timescale_dataset[15:30, 1])
        not_center = abs(gtan) > 0.2
        lane_as_one = abs(gtan-ltan) < 0.1
        diff_ltan_high = abs(ltan-prev_ltan) > 0.2
        if not_center & lane_as_one & diff_ltan_high:
            return True
        return False

    def update(self, real_speed, identity_size, yellow:np.ndarray):
        # 차선의 형태를 계산한다, 그리고 하나의 데이터로 만든다.
        # 데이터베이스에 데이터들을 나열한다.
        self.append_latest_data(self.get_global_tangent(identity_size, yellow),\
                                *self.get_local_tangent(identity_size, yellow), self.timer)

        if self.timer > 0:
            # 타이머가 작동하는 경우 적분을 이용하여 현재 속력만큼 타이머 숫자를 줄인다.
            self.timer -= real_speed[0] / 30

            # 아래 조건을 불만족 하는 경우 이전(previous) 데이터를 계속 사용한다.
            prev_gtan = self.timescale_dataset[1, 0]
            if self.on_curve_transition():
                self.timescale_dataset[0, 0] = prev_gtan

        # 실시간으로 데이터베이스를 그래프로 보여준다.
        self.show_dataset_graph()

        gtan, ltan_abs = self.timescale_dataset[0, 0], self.timescale_dataset[0, 2]

        # 커브를 발견하면 1.0m 타이머 시작
        if self.found_junction(gtan) and self.timer <= 0:
            self.pause_until(1.0)
        
        # 차선이 수평하면 (휘어있으면) 속도 줄임
        # 그렇지 않으면 (곧으면) 속도 늘림
        delta_x = ltan_abs + 0.5

        # 차선이 한쪽으로 치우쳐져 있어 global_tan이 0이 아니면 회전
        delta_z = np.tan(gtan/2)*2

        # 급커브 처리
        if self.timer > 0:
            arc_offset = 0.5
            if ltan_abs < 0.1 or abs(gtan) > 0.8:
                arc_offset = 0
                delta_x /= delta_z if delta_z != 0 else 1
                delta_z /= 2
            if (gtan > 0):
                delta_z -= arc_offset
            elif (gtan < 0):
                delta_z += arc_offset

        # 새 속도는 바로 적용되는 것이 아니라 이전속도를 절반만큼 반영함
        # 주행이 부드러워지는 효과를 낼 수 있음
        self.x = (self.x + delta_x) / 2
        self.z = (self.z + delta_z) / 2
        
    def get_global_tangent(self, identity_size, yellow:np.ndarray) -> float:
        if identity_size <= 0:
            return 0.0
        x_set = yellow * self.mask_global_x
        y_set = ((yellow.T) * self.mask_global_y).T
        x_set, y_set = np.where(x_set, x_set, 1), np.where(x_set, y_set, 1000*y_set)
        return np.arctan(np.sum(y_set/x_set) / identity_size)

    def get_local_tangent(self, identity_size, yellow:np.ndarray) -> tuple:
        if identity_size == 0:
            return 0.0, 0.0
        l_tan = 0.0
        l_tan_abs = 0.0
        arange = self.mask_local
        identity_size_local = 0
        for x, y in np.argwhere(yellow):
            if random.randint(1, identity_size) > 32:
                continue
            base = yellow[-2+x:3+x, -2+y:3+y].copy()
            base[:, 2] = 0
            x_set = base * arange
            y_set = (base.T * arange).T
            x_set, y_set = np.where(x_set, x_set, 1), np.where(x_set, y_set, 1000*y_set)
            identity_size_local += np.sum(base)
            l_tan += np.sum(y_set/x_set)
            l_tan_abs += np.sum(np.abs(y_set/x_set))
        if identity_size_local == 0:
            return 0.0, 0.0
        l_tan = np.arctan(l_tan / identity_size_local)
        l_tan_abs = np.arctan(l_tan_abs / identity_size_local)
        return l_tan, l_tan_abs