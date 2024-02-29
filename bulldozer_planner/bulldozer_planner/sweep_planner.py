from sweep_searcher import SweepSearcher
import math
from .utils import global_to_local, local_to_global
import numpy as np
from ..bulldozer_planner.grid_map_lib import GridMap, FloatGrid


class SweepPlanner:
    def planning(self, ox, oy, resolution,
                 moving_direction=SweepSearcher.MovingDirection.RIGHT,
                 sweeping_direction=SweepSearcher.SweepDirection.DOWN,
                 ):
        # find start pos and sweep direction
        sweep_vec, sweep_start_position = self.find_sweep_direction_and_start_position(
            ox, oy)

        # transform to local coord frame
        rox, roy = global_to_local(ox, oy, sweep_vec, sweep_start_position)

        # set up occupancy grid
        grid_map, x_inds_goal_y, goal_y = self.setup_grid_map(rox, roy, resolution,
                                                              sweeping_direction)

        sweep_searcher = SweepSearcher(moving_direction, sweeping_direction,
                                       x_inds_goal_y, goal_y)
        # search for path
        px, py = self.sweep_path_search(sweep_searcher, grid_map)

        # return to global coord frame
        rx, ry = local_to_global(px, py, sweep_vec, sweep_start_position)

        print("Path length:", len(rx))

        return rx, ry

    def find_sweep_direction_and_start_position(self, ox, oy):
        i = 0
        dx = ox[i + 1] - ox[i]
        dy = oy[i + 1] - oy[i]
        vec = [dx, dy]
        sweep_start_pos = [ox[i], oy[i]]
        return vec, sweep_start_pos

    def setup_grid_map(self, ox, oy, resolution, sweep_direction, offset_grid=10):
        width = math.ceil((max(ox) - min(ox)) / resolution) + offset_grid
        height = math.ceil((max(oy) - min(oy)) / resolution) + offset_grid
        center_x = (np.max(ox) + np.min(ox)) / 2.0
        center_y = (np.max(oy) + np.min(oy)) / 2.0

        grid_map = GridMap(width, height, resolution, center_x, center_y)
        # grid_map.print_grid_map_info()
        grid_map.set_value_from_polygon(ox, oy, FloatGrid(1.0), inside=False)
        grid_map.expand_grid()

        x_inds_goal_y = []
        goal_y = 0
        if sweep_direction == SweepSearcher.SweepDirection.UP:
            x_inds_goal_y, goal_y = self.search_free_grid_index_at_edge_y(
                grid_map, from_upper=True)
        elif sweep_direction == SweepSearcher.SweepDirection.DOWN:
            x_inds_goal_y, goal_y = self.search_free_grid_index_at_edge_y(
                grid_map, from_upper=False)

        return grid_map, x_inds_goal_y, goal_y

    def search_free_grid_index_at_edge_y(self, grid_map, from_upper=False):
        y_index = None
        x_indexes = []

        if from_upper:
            x_range = range(grid_map.height)[::-1]
            y_range = range(grid_map.width)[::-1]
        else:
            x_range = range(grid_map.height)
            y_range = range(grid_map.width)

        for iy in x_range:
            for ix in y_range:
                if not SweepSearcher.check_occupied(ix, iy, grid_map):
                    y_index = iy
                    x_indexes.append(ix)
            if y_index:
                break

        return x_indexes, y_index

    def sweep_path_search(self, sweep_searcher, grid_map, grid_search_animation=False):
        # search start grid
        c_x_index, c_y_index = sweep_searcher.search_start_grid(grid_map)
        if not grid_map.set_value_from_xy_index(c_x_index, c_y_index, FloatGrid(0.5)):
            print("Cannot find start grid")
            return [], []

        x, y = grid_map.calc_grid_central_xy_position_from_xy_index(c_x_index,
                                                                    c_y_index)
        px, py = [x], [y]

        while True:
            c_x_index, c_y_index = sweep_searcher.move_target_grid(c_x_index,
                                                                   c_y_index,
                                                                   grid_map)

            if sweep_searcher.is_search_done(grid_map) or (
                    c_x_index is None or c_y_index is None):
                print("Done")
                break

            x, y = grid_map.calc_grid_central_xy_position_from_xy_index(
                c_x_index, c_y_index)

            px.append(x)
            py.append(y)

            grid_map.set_value_from_xy_index(
                c_x_index, c_y_index, FloatGrid(0.5))

        return px, py
