import numpy as np

GRID_RESOLUTION = 0.06
MAP_ORIGIN_X = -0.66
MAP_ORIGIN_Y = -0.57


class KnownMap:

    def __init__(self):
        self.grid = np.zeros((19, 22), dtype=int)
        #paredes
        self.grid[0, :] = 1
        self.grid[-1, :] = 1
        self.grid[:, 0] = 1
        self.grid[:, -1] = 1

        self.resolution = GRID_RESOLUTION

    def is_free(self, x, y):
        return self.grid[y][x] == 0

    def in_bounds(self, x, y):
        return (
            0 <= x < self.grid.shape[1] and
            0 <= y < self.grid.shape[0]
        )

    def world_to_grid(self, x, y):
        col = int((x - MAP_ORIGIN_X) / GRID_RESOLUTION)
        row = int((y - MAP_ORIGIN_Y) / GRID_RESOLUTION)
        return row, col

    def grid_to_world(self, row, col):
        x = MAP_ORIGIN_X + col * GRID_RESOLUTION
        y = MAP_ORIGIN_Y + row * GRID_RESOLUTION
        return x, y