import math
import numpy as np
from scipy.spatial.transform import Rotation as Rot


def rot_mat_2d(angle):
    return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]


def global_to_local(x, y, origin, dir_vec):
    # shift the origin
    tx = [ix - origin[0] for ix in x]
    ty = [iy - origin[1] for iy in y]
    th = math.atan2(dir_vec[1], dir_vec[0])
    # rotate frame
    converted_xy = np.stack([tx, ty]).T @ rot_mat_2d(th)
    return converted_xy[:, 0], converted_xy[:, 1]


def local_to_global(x, y, origin, dir_vec):
    th = math.atan2(dir_vec[1], dir_vec[0])
    converted_xy = np.stack([x, y]).T @ rot_mat_2d(-th)
    rx = [ix + origin[0] for ix in converted_xy[:, 0]]
    ry = [iy + origin[1] for iy in converted_xy[:, 1]]
    return rx, ry
