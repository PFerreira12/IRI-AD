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


def bresenham(self, start_cell, end_cell):
    start_row, start_col = start_cell
    end_row, end_col = end_cell

    cells = []

    delta_row = abs(end_row - start_row)
    delta_col = abs(end_col - start_col)

    step_row = 1 if start_row < end_row else -1
    step_col = 1 if start_col < end_col else -1

    row = start_row
    col = start_col

    if delta_col > delta_row:
        error = delta_col / 2

        while col != end_col:
            cells.append((row, col))
            error -= delta_row

            if error < 0:
                row += step_row
                error += delta_col

            col += step_col

    else:
        error = delta_row / 2

        while row != end_row:
            cells.append((row, col))
            error -= delta_col

            if error < 0:
                col += step_col
                error += delta_row

            row += step_row

    cells.append((end_row, end_col))

    return cells

FREE = 0

def find_nearest_free(nav, goal_cell, radius=4):

    gr, gc = goal_cell

    print("\n[FIND NEAREST FREE]")
    print("goal:", goal_cell)
    print("goal occupancy:", nav.map.grid[gr][gc])

    for rad in range(1, radius + 1):
        print(f"\n[RADIUS {rad}]")

        for r in range(gr - rad, gr + rad + 1):
            for c in range(gc - rad, gc + rad + 1):

                if 0 <= r < nav.map.height and 0 <= c < nav.map.width:
                    value = nav.map.grid[r][c]
                    print(f"checking {(r,c)} -> {value}")

                    if value == FREE:
                        print("FOUND FREE:", (r,c))
                        return (r, c)

    print("NO FREE CELL FOUND")
    return None

import numpy as np, matplotlib.pyplot as plt

def save_grid_png(grid, robot_pos=None, goal_pos=None, path=None, filename="grid_debug.png"):
    grid_np = np.array(grid)

    plt.figure(figsize=(6, 6))
    plt.imshow(grid_np, cmap="gray_r", origin="lower")

    # robot
    if robot_pos is not None:
        r, c = robot_pos
        plt.scatter(c, r, c="red", label="robot")

    # goal
    if goal_pos is not None:
        r, c = goal_pos
        plt.scatter(c, r, c="green", s=60, label="goal")

    # A* path
    if path is not None and len(path)>0:
        xs = [cell[1] for cell in path]
        ys = [cell[0] for cell in path]
        plt.plot(xs, ys, c="blue", linewidth=2, label="path")

    plt.legend()
    plt.title("Occupancy Grid Debug")
    plt.savefig(filename)
    plt.close()