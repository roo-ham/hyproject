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
        self.global_tan = 0.0
        self.local_tan = 0.0
        self.local_tan_abs = 0.0
        self.timescale_dataset = np.zeros((60,3), np.float32)
        self.weight_x = 1.0
        self.weight_z = 1.0
        self.x_data = range(60)
        self.tick = 0

        self.fig, self.axes = plt.subplots(nrows=3)
        styles = ['r-', 'g-', 'y-']
        labels = ['G tan', 'L tan', 'L tan (absolute)']
        def plot(ax, style, label):
            plot = ax.plot(self.x_data, self.timescale_dataset[:, 0], style, animated=True, label=label)[0]
            ax.set_xlim(0, 59)
            ax.set_ylim(-3, 3)
            ax.legend()
            return plot
        self.lines = [plot(ax, style, label) for ax, style, label in zip(self.axes, styles, labels)]
        self.fig.show()
        self.fig.canvas.draw()
        self.backgrounds = [self.fig.canvas.copy_from_bbox(ax.bbox) for ax in self.axes]

    def append_latest_data(self):
        self.timescale_dataset[1:60, :] = self.timescale_dataset[0:59, :]
        self.timescale_dataset[0, :] = (self.global_tan, self.local_tan, self.local_tan_abs)

    def show_dataset_graph(self):
        items = enumerate(zip(self.lines, self.axes, self.backgrounds), start=0)
        for j, (line, ax, background) in items:
            self.fig.canvas.restore_region(background)
            line.set_ydata(self.timescale_dataset[:, j])
            ax.draw_artist(line)
            self.fig.canvas.blit(ax.bbox)

    def pause_until(self, t):
        self.tick = t

    def on_pause(self, t) -> bool:
        return self.tick >= t
    
    def on_curve_transition(self, tick, gtan):
        if not self.on_pause(tick):
            return False
        elif self.global_tan * gtan >= 0:
            return False
        elif abs(self.global_tan) < 0.4:
            return False
        return True
            

    def update(self, tick, identity_size, yellow:np.ndarray):
        gtan = self.get_global_tangent(identity_size, yellow)
        ltan = self.get_local_tangent(identity_size, yellow)

        # to-do
        # self.tick - 15 >= tick 일 경우 tan을 저장만 하고 x, z는 변경 없음
        # on_curve_transition 일 경우 데이터 수집을 중단함
        # self.tick - 15 < tick 일 경우 tan을 x, z에 반영함
        
        # 차선의 형태를 계산한다, 그리고 하나의 데이터로 만든다.
        if not self.on_curve_transition(tick, gtan) :
            self.global_tan = gtan
            self.local_tan, self.local_tan_abs = ltan

        # 데이터베이스에 데이터들을 실시간으로 나열한다.
        self.append_latest_data()

        # 0.1초마다 데이터베이스를 그래프로 보여준다.
        if tick % 3 == 0:
            self.show_dataset_graph()

        # 급경사를 발견하면 2초 대기 시작
        if abs(self.global_tan) >= 0.4 and abs(self.global_tan-self.local_tan) >= 0.2 and (not self.on_pause(tick)) :
            self.pause_until(tick + 60)

        # 급경사를 발견 후 1.5초 까지는 직진을 함
        if self.on_pause(tick + 45):
            self.weight_z = 0.1
            return
        else:
            self.weight_z = 1.0
        
        # 차선이 수평하면 (휘어있으면) 속도 줄임
        # 그렇지 않으면 (곧으면) 속도 늘림
        delta_x = self.local_tan_abs + 0.25

        # 차선이 한쪽으로 치우쳐져 있어 global_tan의 값이 0이 아니면 회전
        # 회전 속도는 차선이 수평할 수록 (휘어있으면) 커짐 (local_tan의 절댓값에 반비례)
        delta_z = self.global_tan / ((self.local_tan**2) + 1)

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