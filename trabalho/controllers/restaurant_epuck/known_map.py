import math, sys
from pathlib import Path

CURRENT_DIR = Path(__file__).resolve().parent
CONTROLLERS_DIR = CURRENT_DIR.parent
COMMON_DIR = CONTROLLERS_DIR / "common"

sys.path.insert(0, str(COMMON_DIR))

from config_tables import get_map_config

FREE = 0
OCCUPIED = 1


class KnownMap:
    """Known occupancy grid built from the selected map configuration.

    The previous version hard-coded Map 1 coordinates directly in this file.
    This version reads tables, plants, walls, counter and floor shape from
    config_tables.py, so the same navigation code can run on map1 or map2.
    """

    def __init__(self, map_config=None):
        self.map_config = map_config or get_map_config()
        km = self.map_config["known_map"]

        self.resolution = km.get("resolution", 0.03)
        self.origin_x = km["origin_x"]
        self.origin_y = km["origin_y"]
        self.width = km["width"]
        self.height = km["height"]

        # If floor_rectangles are provided, non-floor cells start as occupied.
        # This is important for Map 2 because the environment is L-shaped.
        has_floor_mask = bool(km.get("floor_rectangles"))
        initial_value = OCCUPIED if has_floor_mask else FREE

        self.grid = [
            [initial_value for _ in range(self.width)]
            for _ in range(self.height)
        ]

        if has_floor_mask:
            self.add_floor(km["floor_rectangles"])

        self.add_walls()
        self.add_config_obstacles(km)
        self.inflate_obstacles(km.get("inflate_radius_cells", 1))

    def mark_rectangle(self, x, y, w, h, value=OCCUPIED):
        min_x = x - w / 2
        max_x = x + w / 2
        min_y = y - h / 2
        max_y = y + h / 2

        min_cell = self.world_to_grid(min_x, min_y)
        max_cell = self.world_to_grid(max_x, max_y)

        if min_cell is None and max_cell is None:
            return

        # Clamp to map limits so rectangles partially outside the grid still work.
        if min_cell is None:
            min_row, min_col = 0, 0
        else:
            min_row, min_col = min_cell

        if max_cell is None:
            max_row, max_col = self.height - 1, self.width - 1
        else:
            max_row, max_col = max_cell

        min_row, max_row = sorted([min_row, max_row])
        min_col, max_col = sorted([min_col, max_col])

        min_row = max(0, min(self.height - 1, min_row))
        max_row = max(0, min(self.height - 1, max_row))
        min_col = max(0, min(self.width - 1, min_col))
        max_col = max(0, min(self.width - 1, max_col))

        for row in range(min_row, max_row + 1):
            for col in range(min_col, max_col + 1):
                self.grid[row][col] = value

    def mark_circle(self, x, y, r, value=OCCUPIED):
        for row in range(self.height):
            for col in range(self.width):
                wx, wy = self.grid_to_world(row, col)
                if math.hypot(wx - x, wy - y) <= r:
                    self.grid[row][col] = value

    def add_floor(self, floor_rectangles):
        for x, y, w, h in floor_rectangles:
            self.mark_rectangle(x, y, w, h, value=FREE)

    def add_config_obstacles(self, known_map_config):
        for x, y, w, h in known_map_config.get("rectangles", []):
            self.mark_rectangle(x, y, w, h, value=OCCUPIED)

        for x, y, r in known_map_config.get("circles", []):
            self.mark_circle(x, y, r, value=OCCUPIED)

    def world_to_grid(self, x, y):
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)

        if row < 0 or row >= self.height:
            return None

        if col < 0 or col >= self.width:
            return None

        return row, col

    def grid_to_world(self, row, col):
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def add_walls(self):
        # Grid limits are always non-walkable.
        for col in range(self.width):
            self.grid[0][col] = OCCUPIED
            self.grid[self.height - 1][col] = OCCUPIED

        for row in range(self.height):
            self.grid[row][0] = OCCUPIED
            self.grid[row][self.width - 1] = OCCUPIED

    def inflate_obstacles(self, radius_cells=1):
        if radius_cells <= 0:
            return

        inflated = [row[:] for row in self.grid]

        for row in range(self.height):
            for col in range(self.width):
                if self.grid[row][col] != OCCUPIED:
                    continue

                for d_row in range(-radius_cells, radius_cells + 1):
                    for d_col in range(-radius_cells, radius_cells + 1):
                        n_row = row + d_row
                        n_col = col + d_col

                        if 0 <= n_row < self.height and 0 <= n_col < self.width:
                            if d_row * d_row + d_col * d_col <= radius_cells * radius_cells:
                                inflated[n_row][n_col] = OCCUPIED

        self.grid = inflated

    def print_grid(self, robot=None, goal=None, path=None):
        path_set = set(path) if path else set()

        for row in range(self.height):
            line = ""

            for col in range(self.width):
                if robot == (row, col):
                    line += "R"
                elif goal == (row, col):
                    line += "G"
                elif (row, col) in path_set:
                    line += "*"
                elif self.grid[row][col] == OCCUPIED:
                    line += "#"
                else:
                    line += "."

            print(line)
        print()
