import numpy as np
import random

def get_wall_angle(positions:list):
    sum1 = 0
    sum2 = 0

    if len(positions) == 0:
        return 0

    mean_pos = sum(positions)/len(positions)

    for pos in positions:
        relative_point:np.ndarray = pos - mean_pos
        if relative_point[1] == 0:
            continue
        arctan = np.arctan(relative_point[0]/relative_point[1])
        length2 = np.sum(relative_point**2)
        sum1 += arctan*length2
        sum2 += length2
    
    return 0 if sum2 == 0 else sum1/sum2

def get_global_tangent(mask_x, mask_y, identity_size, yellow:np.ndarray) -> float:
    x_set = yellow * mask_x
    y_set = ((yellow.T) * mask_y).T
    x_set, y_set = np.where(x_set, x_set, 1), np.where(x_set, y_set, 1000*y_set)
    return np.arctan(np.sum(y_set/x_set) / identity_size)

def get_local_tangent(mask, identity_size, yellow:np.ndarray) -> tuple:
    l_tan = 0.0
    l_tan_abs = 0.0
    for x, y in np.argwhere(yellow):
        if random.randint(1, identity_size) > 32:
            continue
        base = yellow[-2+x:3+x, -2+y:3+y].copy()
        base[:, 2] = 0
        x_set = base * mask
        y_set = (base.T * mask).T
        x_set, y_set = np.where(x_set, x_set, 1), np.where(x_set, y_set, 1000*y_set)

        base2 = (x_set**2)+(y_set**2)
        identity_size_local = np.sum(base2)

        if identity_size_local == 0:
            continue
        atan = np.arctan(y_set/x_set) * base2
        atan_abs = np.abs(atan)
        l_tan += np.sum(atan) / identity_size_local
        l_tan_abs += np.sum(atan_abs) / identity_size_local
    l_tan /= identity_size
    l_tan_abs /= identity_size
    return l_tan, l_tan_abs