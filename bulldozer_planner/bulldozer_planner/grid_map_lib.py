import matplotlib.pyplot as plt
import numpy as np


class GridMap:
    """
    GridMap class
    """

    def __init__(self, width, height, resolution,
                 center_x, center_y, init_val=0.0):
        """__init__

        :param width: number of grid for width
        :param height: number of grid for height
        :param resolution: grid resolution [m]
        :param center_x: center x position  [m]
        :param center_y: center y position [m]
        :param init_val: initial value for all grid
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        self.center_x = center_x
        self.center_y = center_y

        self.left_lower_x = self.center_x - self.width / 2.0 * self.resolution
        self.left_lower_y = self.center_y - self.height / 2.0 * self.resolution

        self.n_data = self.width * self.height
        self.data = [init_val] * self.n_data
        self.data_type = type(init_val)

    def __getitem__(self, item):
        x_ind, y_ind = item

        grid_ind = self.xy2index(x_ind, y_ind)

        if 0 <= grid_ind < self.n_data:
            return self.data[grid_ind]
        else:
            return None

    def __setitem__(self, index, value):
        x_ind, y_ind = index
        if (x_ind is None) or (y_ind is None):
            return False, False

        grid_ind = self.xy2index(x_ind, y_ind)

        if 0 <= grid_ind < self.n_data and isinstance(value, self.data_type):
            self.data[grid_ind] = value
            return True  # OK
        else:
            return False  # NG

    def set_value_from_polygon(self, pol_x, pol_y, val, inside=True):
        """set_value_from_polygon

        Setting value inside or outside polygon

        :param pol_x: x position list for a polygon
        :param pol_y: y position list for a polygon
        :param val: grid value
        :param inside: setting data inside or outside
        """

        # making ring polygon
        if (pol_x[0] != pol_x[-1]) or (pol_y[0] != pol_y[-1]):
            np.append(pol_x, pol_x[0])
            np.append(pol_y, pol_y[0])

        # setting value for all grid
        for x_ind in range(self.width):
            for y_ind in range(self.height):
                x_pos, y_pos = self.xy2pos(
                    x_ind, y_ind)

                flag = self.check_inside_polygon(x_pos, y_pos, pol_x, pol_y)

                if flag is inside:
                    self[x_ind, y_ind] = val

    def xy2index(self, x_ind, y_ind):
        grid_ind = int(y_ind * self.width + x_ind)
        return grid_ind

    def xy2pos(self, x_ind, y_ind):
        x_pos = self.index2pos(
            x_ind, self.left_lower_x)
        y_pos = self.index2pos(
            y_ind, self.left_lower_y)

        return x_pos, y_pos

    def index2pos(self, index, lower_pos):
        return lower_pos + index * self.resolution + self.resolution / 2.0

    def check_occupied(self, x_ind, y_ind, occupied_val):

        val = self[x_ind, y_ind]

        if val is None or val >= occupied_val:
            return True
        else:
            return False

    def expand_grid(self, occupied_val=1.0):
        x_inds, y_inds, values = [], [], []

        for ix in range(self.width):
            for iy in range(self.height):
                if self.check_occupied(ix, iy, occupied_val):
                    x_inds.append(ix)
                    y_inds.append(iy)
                    values.append(self[ix, iy])

        for (ix, iy, value) in zip(x_inds, y_inds, values):
            self[ix + 1, iy] = value
            self[ix, iy + 1] = value
            self[ix + 1, iy + 1] = value
            self[ix - 1, iy] = value
            self[ix, iy - 1] = value
            self[ix - 1, iy - 1] = value

    @staticmethod
    def check_inside_polygon(iox, ioy, x, y):

        n_point = len(x) - 1
        inside = False
        for i1 in range(n_point):
            i2 = (i1 + 1) % (n_point + 1)

            if x[i1] >= x[i2]:
                min_x, max_x = x[i2], x[i1]
            else:
                min_x, max_x = x[i1], x[i2]
            if not min_x <= iox < max_x:
                continue

            tmp1 = (y[i2] - y[i1]) / (x[i2] - x[i1])
            if (y[i1] + tmp1 * (iox - x[i1]) - ioy) > 0.0:
                inside = not inside

        return inside

    def plot(self, ax=None):
        float_data_array = np.array([d.get_float_data() for d in self.data])
        grid_data = np.reshape(float_data_array, (self.height, self.width))
        if not ax:
            fig, ax = plt.subplots()
        heat_map = ax.pcolor(grid_data, cmap="Blues", vmin=0.0, vmax=1.0)
        plt.axis("equal")
        plt.show()

        return heat_map
