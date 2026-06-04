import math

FREE = 0; OCCUPIED = 1

class KnownMap:
    
    def __init__(self):
        self.resolution = 0.03

        self.origin_x = -0.66
        self.origin_y = -0.57

        self.width = 44
        self.height = 38

        self.grid = [
            [FREE for _ in range(self.width)]
            for _ in range(self.height)
        ]

        self.add_walls()
        self.add_tables()
        self.add_plants()
        self.inflate_obstacles()


    def mark_rectangle(self, x, y, w, h):
        min_x = x - w/2
        max_x = x + w/2
        min_y = y - h/2
        max_y = y + h/2

        min_row, min_col = self.world_to_grid(min_x, min_y)
        max_row, max_col = self.world_to_grid(max_x, max_y)

        min_row, max_row = sorted([min_row, max_row])
        min_col, max_col = sorted([min_col, max_col])

        for r in range(min_row, max_row + 1):
            for c in range(min_col, max_col + 1):
                if 0 <= r < self.height and 0 <= c < self.width:
                    self.grid[r][c] = OCCUPIED


    def mark_circle(self, x, y, r):
        for row in range(self.height):
            for col in range(self.width):
                wx, wy = self.grid_to_world(row, col)
                if math.hypot(wx - x, wy - y) <= r:
                    self.grid[row][col] = OCCUPIED


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
        # bottom/top
        for col in range(self.width):
            self.grid[0][col] = 1
            self.grid[self.height - 1][col] = OCCUPIED

        # left/right
        for row in range(self.height):
            self.grid[row][0] = 1
            self.grid[row][self.width - 1] = OCCUPIED


    def add_tables(self):
        self.mark_rectangle(-0.432, -0.312, 0.126, 0.126) #table1
        self.mark_rectangle(-0.168, -0.120, 0.126, 0.126) #table2
        self.mark_rectangle(-0.408, 0.204, 0.126, 0.126) #table3
        self.mark_rectangle(0.396, -0.252, 0.126, 0.126) #taable4
        self.mark_rectangle(0.144, 0.084, 0.126, 0.126) #table5
        self.mark_rectangle(0.432, 0.336, 0.126, 0.126) #table6
        self.mark_rectangle(0, -0.486, 0.384, 0.066) #counter


    def add_plants(self):
        self.mark_circle(-0.576, 0.48, 0.042) #corner_plant1
        self.mark_circle(0.576, 0.48, 0.042) #corner_plant2
        self.mark_circle(-0.576, -0.48, 0.042) #corner_plant3
        self.mark_circle(0.576, -0.48, 0.042) #corner_plant4

    
    def inflate_obstacles(self, radius_cells=1):

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

                            if d_row*d_row + d_col*d_col <= radius_cells*radius_cells:
                                inflated[n_row][n_col] = OCCUPIED

        self.grid = inflated

    def print_grid(self, robot=None, goal=None, path=None):
        path_set = set(path) if path else set()

        for r in range(self.height):
            line = ""

            for c in range(self.width):

                if robot == (r, c):
                    line += "R"
                elif goal == (r, c):
                    line += "G"
                elif (r, c) in path_set:
                    line += "*"
                else:
                    if self.grid[r][c] == 1:
                        line += "#"
                    else:
                        line += "."

            print(line)
        print()