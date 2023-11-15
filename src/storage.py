import numpy as np
import os

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
        self.local_tan_squared = 0.0
        self.weight_x = 1.0
        self.weight_z = 1.0

    def update(self, identity_size, yellow:np.ndarray):
        self.global_tan = self.get_global_tangent(identity_size, yellow)
        self.local_tan, self.local_tan_squared = self.get_local_tangent(identity_size, yellow)
        self.x *= 0.5
        self.z *= 0.5
        self.x += (self.local_tan_squared + 0.25) * 0.5
        self.z += (self.global_tan / ((self.local_tan**2) + 1)) * 0.5
        os.system("clear")
        print("%f, %f, %f"%(self.global_tan, self.local_tan,\
                                        self.local_tan_squared))

    def get_global_tangent(self, identity_size, yellow:np.ndarray) -> float:
        if identity_size <= 0:
            return 0.0
        x_set = yellow * self.mask_global_x
        y_set = ((yellow.T) * self.mask_global_y).T
        x_set, y_set = np.where(x_set != 0, x_set, 1), np.where(x_set != 0, y_set, 0)
        return np.arctan(np.sum(y_set/x_set) / identity_size)

    def get_local_tangent(self, identity_size, yellow:np.ndarray) -> tuple:
        if identity_size <= 0:
            return 0.0, 0.0
        l_tan = 0.0
        l_tan_squared = 0.0
        arange = self.mask_local
        for x, y in np.argwhere(yellow):
            if x%2 != 0:
                continue
            elif y%8 != 0:
                continue
            base = yellow[-2+x:3+x, -2+y:3+y]
            x_set = base * arange
            y_set = (base.T * arange).T
            x_set_zero = x_set != 0
            x_set, y_set = np.where(x_set_zero, x_set, 1), np.where(x_set_zero, y_set, 0)
            tan0 = np.sum(y_set/x_set)
            identity_size_local = np.sum(base)
            l_tan += tan0 / (identity_size*identity_size_local)
            l_tan_squared += tan0**2 / (identity_size*identity_size_local)
        l_tan = np.arctan(l_tan)
        l_tan_squared = np.arctan(l_tan_squared)
        return l_tan, l_tan_squared