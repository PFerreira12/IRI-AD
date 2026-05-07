import heapq
import math

FREE = 0.0
UNKNOWN = 0.5
OCCUPIED = 1.0


def heuristic(a, b):
    row_a, col_a = a
    row_b, col_b = b
    return math.hypot(row_b - row_a, col_b - col_a)


def get_neighbors(cell, rows, cols, allow_diagonal=True):
    row, col = cell

    if allow_diagonal:
        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        ]
    else:
        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
        ]

    for d_row, d_col in directions:
        n_row = row + d_row
        n_col = col + d_col

        if 0 <= n_row < rows and 0 <= n_col < cols:
            yield n_row, n_col


def movement_cost(current, neighbor):
    row_a, col_a = current
    row_b, col_b = neighbor

    if row_a != row_b and col_a != col_b:
        return math.sqrt(2)

    return 1.0


def is_walkable(grid, cell):
    row, col = cell
    return grid[row][col] == FREE


def reconstruct_path(came_from, current):
    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path


def astar_grid(grid, start, goal, allow_diagonal=True):
    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0

    if rows == 0 or cols == 0:
        return []

    start_row, start_col = start
    goal_row, goal_col = goal

    if not (0 <= start_row < rows and 0 <= start_col < cols):
        return []

    if not (0 <= goal_row < rows and 0 <= goal_col < cols):
        return []

    if not is_walkable(grid, start):
        return []

    if not is_walkable(grid, goal):
        return []

    open_heap = []
    heapq.heappush(open_heap, (0.0, start))

    came_from = {}
    g_score = {start: 0.0}
    closed = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current in closed:
            continue

        if current == goal:
            return reconstruct_path(came_from, current)

        closed.add(current)

        for neighbor in get_neighbors(current, rows, cols, allow_diagonal):
            if neighbor in closed:
                continue

            if not is_walkable(grid, neighbor):
                continue

            tentative_g = g_score[current] + movement_cost(current, neighbor)

            if tentative_g < g_score.get(neighbor, math.inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f_score, neighbor))

    return []