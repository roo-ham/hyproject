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
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.axis((0, 60, -3, 3))
        plt.legend()
        plt.show()

    def update(self, tick, identity_size, yellow:np.ndarray):
        self.global_tan = self.get_global_tangent(identity_size, yellow)
        self.local_tan, self.local_tan_abs = self.get_local_tangent(identity_size, yellow)
        self.timescale_dataset[1:60, :] = self.timescale_dataset[0:59, :]
        self.timescale_dataset[0, :] = (self.global_tan, self.local_tan, self.local_tan_abs)
        if tick % 30 == 0:
            self.ax.cla()
            self.ax.plot(self.x_data, self.timescale_dataset[:, 0], label="gTan")
            self.ax.plot(self.x_data, self.timescale_dataset[:, 1], label="lTan")
            self.ax.plot(self.x_data, self.timescale_dataset[:, 2], label="lTan2")
            plt.pause(0.01)

        # 차선이 한쪽으로 치우쳐져 있어 global_tan의 값이 0이 아니면 회전
        # 회전 속도는 차선이 수평할 수록 커짐 (local_tan의 절댓값에 반비례)
        delta_z = self.global_tan / ((self.local_tan**2) + 1)

        self.x = (self.x + self.local_tan_abs + 0.25) / 2
        self.z = (self.z + delta_z) / 2

    def get_global_tangent(self, identity_size, yellow:np.ndarray) -> float:
        if identity_size <= 0:
            return 0.0
        x_set = yellow * self.mask_global_x
        y_set = ((yellow.T) * self.mask_global_y).T
        x_set, y_set = np.where(x_set != 0, x_set, 1), np.where(x_set != 0, y_set, 0)
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
            base = yellow[-2+x:3+x, -2+y:3+y]
            base[:, 2] = 0
            x_set = base * arange
            y_set = (base.T * arange).T
            x_set, y_set = np.where(x_set, x_set, 1), np.where(x_set, y_set, 0)
            identity_size_local += np.sum(base)
            l_tan += np.sum(y_set/x_set)
            l_tan_abs += np.sum(np.abs(y_set/x_set))
        if identity_size_local == 0:
            return 0.0, 0.0
        l_tan = np.arctan(l_tan / identity_size_local)
        l_tan_abs = np.arctan(l_tan_abs / identity_size_local)
        return l_tan, l_tan_abs