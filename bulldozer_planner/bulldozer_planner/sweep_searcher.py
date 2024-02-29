from enum import IntEnum
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))


def search_free_grid_index_at_edge_y(grid_map, from_upper=False):
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


class SweepSearcher:
    class SweepDirection(IntEnum):
        UP = 1
        DOWN = -1

    class MovingDirection(IntEnum):
        RIGHT = 1
        LEFT = -1

    def __init__(self,
                 moving_direction, sweep_direction, x_inds_goal_y, goal_y):
        self.moving_direction = moving_direction
        self.sweep_direction = sweep_direction
        self.moves = []
        self.update_moves()
        self.x_indexes_goal_y = x_inds_goal_y
        self.goal_y = goal_y

    def move_target(self, c_x_index, c_y_index, grid_map):
        n_x_index = self.moving_direction + c_x_index
        n_y_index = c_y_index

        # found safe grid
        if not self.check_occupied(n_x_index, n_y_index, grid_map):
            return n_x_index, n_y_index
        else:  # occupied
            next_c_x_index, next_c_y_index = self.find_safe_move(
                c_x_index, c_y_index, grid_map)
            if (next_c_x_index is None) and (next_c_y_index is None):
                # moving backward
                next_c_x_index = -self.moving_direction + c_x_index
                next_c_y_index = c_y_index
                if self.check_occupied(next_c_x_index, next_c_y_index, grid_map, 1.0):
                    # moved backward, but the grid is occupied by obstacle
                    return None, None
            else:
                # keep moving until end
                while not self.check_occupied(next_c_x_index + self.moving_direction, next_c_y_index, grid_map):
                    next_c_x_index += self.moving_direction
                self.swap_direction()
            return next_c_x_index, next_c_y_index

    @staticmethod
    def check_occupied(c_x_index, c_y_index, grid_map, occupied_val=0.5):
        return grid_map.check_occupied(c_x_index, c_y_index, occupied_val)

    def find_safe_move(self, c_x_index, c_y_index, grid_map):

        for (d_x_ind, d_y_ind) in self.moves:

            next_x_ind = d_x_ind + c_x_index
            next_y_ind = d_y_ind + c_y_index

            # found safe grid
            if not self.check_occupied(next_x_ind, next_y_ind, grid_map):
                return next_x_ind, next_y_ind

        return None, None

    def is_done(self, grid_map):
        for ix in self.x_indexes_goal_y:
            if not self.check_occupied(ix, self.goal_y, grid_map):
                return False

        # all lower grid is occupied
        return True

    def update_moves(self):
        # turning window definition
        # robot can move grid based on it.
        self.moves = [
            (self.moving_direction, 0.0),
            (self.moving_direction, self.sweep_direction),
            (0, self.sweep_direction),
            (-self.moving_direction, self.sweep_direction),
        ]

    def swap_direction(self):
        self.moving_direction *= -1
        self.update_moves()

    def search_start_grid(self, grid_map):
        x_inds = []
        y_ind = 0
        if self.sweep_direction == self.SweepDirection.DOWN:
            x_inds, y_ind = search_free_grid_index_at_edge_y(
                grid_map, from_upper=True)
        elif self.sweep_direction == self.SweepDirection.UP:
            x_inds, y_ind = search_free_grid_index_at_edge_y(
                grid_map, from_upper=False)

        if self.moving_direction == self.MovingDirection.RIGHT:
            return min(x_inds), y_ind
        elif self.moving_direction == self.MovingDirection.LEFT:
            return max(x_inds), y_ind

        raise ValueError("self.moving direction is invalid ")
