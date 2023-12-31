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
    n = 0
    for x, y in np.argwhere(yellow):
        if random.randint(1, identity_size) > 16:
            continue
        base = yellow[-2+x:3+x, -2+y:3+y].copy()
        x_set = base * mask
        y_set = (base.T * mask).T
        base2 = (x_set**2)+(y_set**2)
        x_set, y_set = np.where(x_set, x_set, 1), np.where(x_set, y_set, 1000*y_set)
        identity_size_local = np.sum(base2)
        if identity_size_local == 0:
            continue
        atan = np.arctan(y_set/x_set) * base2
        atan_abs = np.abs(atan)
        l_tan += np.sum(atan) / identity_size_local
        l_tan_abs += np.sum(atan_abs) / identity_size_local
        n += 1
    if (n == 0):
        return None, None
    l_tan /= n
    l_tan_abs /= n
    return l_tan, l_tan_abs

def get_local_cotangent(mask, identity_size, yellow:np.ndarray) -> tuple:
    l_tan = 0.0
    n = 0
    for x, y in np.argwhere(yellow):
        if random.randint(1, identity_size) > 16:
            continue
        base = yellow[-2+x:3+x, -2+y:3+y].copy()
        x_set = (base.T * mask).T
        y_set = base * mask
        base2 = (x_set**2)+(y_set**2)
        x_set, y_set = np.where(x_set, x_set, 1), np.where(x_set, y_set, 1000*y_set)
        identity_size_local = np.sum(base2)
        if identity_size_local == 0:
            continue
        atan = np.arctan(y_set/x_set) * base2
        l_tan += np.sum(atan) / identity_size_local
        n += 1
    if (n == 0):
        return None
    l_tan /= n
    return l_tan

def get_yellow_distribution(yellow, b_height):
    y2 = yellow.T @ np.ones(b_height)
    y2 = np.where(y2 != 0, 1, 0)
    return np.mean(y2)