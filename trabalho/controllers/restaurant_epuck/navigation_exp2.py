"""Navegacao da Experiencia 2.

Mantem uma grelha de ocupacao incremental construida com Lidar.
Calcula A* sobre uma grelha de planeamento com obstaculos inflados.
A exploracao/frontiers ainda fica para fase seguinte.
"""

import math
from astar_grid import astar_grid


FREE = 0.0
UNKNOWN = 0.5
OCCUPIED = 1.0

MAP_RESOLUTION = 0.03
MAP_WIDTH = 100
MAP_HEIGHT = 100

LIDAR_SAMPLE_STEP = 6
LIDAR_MAX_RANGE = 2.0
LIDAR_MIN_RANGE = 0.02
DEFAULT_LIDAR_FOV = 6.28

PATH_DEBUG_INTERVAL = 2.0

# Margem de segurança em torno de obstáculos para o A*.
# 2 células * 0.03 m = cerca de 6 cm.
OBSTACLE_INFLATION_RADIUS = 2

PATH_REPLAN_INTERVAL = 2.0


class NavigationExp2:
    def __init__(self, manager):
        self.manager = manager

        self.occupancy_grid = [
            [UNKNOWN for _ in range(MAP_WIDTH)]
            for _ in range(MAP_HEIGHT)
        ]

        self.origin_x = -MAP_WIDTH * MAP_RESOLUTION / 2
        self.origin_y = -MAP_HEIGHT * MAP_RESOLUTION / 2

        self.last_summary_time = -999.0
        self.last_path_debug_time = -999.0
        self.last_path_plan_time = -999.0
        self.cached_target_pos = None
        self.last_planned_path = []
        self.last_selected_target_pos = None

        print(
            "[navigation_exp2] initialized "
            f"grid={MAP_WIDTH}x{MAP_HEIGHT}, "
            f"resolution={MAP_RESOLUTION}, "
            f"origin=({self.origin_x:.2f}, {self.origin_y:.2f}), "
            f"inflation_radius={OBSTACLE_INFLATION_RADIUS}"
        )

    def is_valid_number(self, value):
        return (
            value is not None
            and not math.isnan(value)
            and not math.isinf(value)
        )

    def world_to_grid(self, x, y):
        if not self.is_valid_number(x) or not self.is_valid_number(y):
            return None

        col = int((x - self.origin_x) / MAP_RESOLUTION)
        row = int((y - self.origin_y) / MAP_RESOLUTION)

        return row, col

    def grid_to_world(self, row, col):
        x = self.origin_x + (col + 0.5) * MAP_RESOLUTION
        y = self.origin_y + (row + 0.5) * MAP_RESOLUTION

        return x, y

    def is_inside_grid(self, row, col):
        return 0 <= row < MAP_HEIGHT and 0 <= col < MAP_WIDTH

    def mark_cell(self, row, col, value):
        if self.is_inside_grid(row, col):
            self.occupancy_grid[row][col] = value

    def mark_robot_cell_free(self):
        position = self.manager.epuck.get_robot_position()

        if position is None:
            return None

        x, y = position
        cell = self.world_to_grid(x, y)

        if cell is None:
            print(f"[navigation_exp2] invalid robot position: ({x}, {y})")
            return None

        row, col = cell

        if not self.is_inside_grid(row, col):
            print(
                "[navigation_exp2] robot outside grid: "
                f"pos=({x:.3f}, {y:.3f}) -> cell=({row}, {col})"
            )
            return None

        self.mark_cell(row, col, FREE)

        return row, col

    def get_safe_heading(self):
        try:
            theta = self.manager.epuck.get_robot_heading()
        except Exception:
            theta = getattr(self.manager.epuck, "estimated_theta", 0.0)

        if not self.is_valid_number(theta):
            theta = 0.0

        return theta

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

    def get_lidar_fov(self):
        try:
            fov = self.manager.epuck.lidar.getFov()
        except Exception:
            fov = DEFAULT_LIDAR_FOV

        if not self.is_valid_number(fov) or fov <= 0:
            fov = DEFAULT_LIDAR_FOV

        return fov

    def update_map_from_lidar(self):
        robot_cell = self.mark_robot_cell_free()

        if self.manager.epuck.lidar is None:
            return {
                "source": "none",
                "robot_cell": robot_cell,
                "lidar_rays_used": 0,
                "occupied_marked": 0,
                "free_marked": 0,
                "theta": self.get_safe_heading(),
            }

        position = self.manager.epuck.get_robot_position()
        theta = self.get_safe_heading()

        if robot_cell is None or position is None:
            return {
                "source": "lidar",
                "robot_cell": robot_cell,
                "lidar_rays_used": 0,
                "occupied_marked": 0,
                "free_marked": 0,
                "theta": theta,
            }

        robot_x, robot_y = position

        if not self.is_valid_number(robot_x) or not self.is_valid_number(robot_y):
            return {
                "source": "lidar",
                "robot_cell": robot_cell,
                "lidar_rays_used": 0,
                "occupied_marked": 0,
                "free_marked": 0,
                "theta": theta,
            }

        ranges = self.manager.epuck.lidar.getRangeImage()
        n = len(ranges)

        if n == 0:
            return {
                "source": "lidar",
                "robot_cell": robot_cell,
                "lidar_rays_used": 0,
                "occupied_marked": 0,
                "free_marked": 0,
                "theta": theta,
            }

        fov = self.get_lidar_fov()

        lidar_rays_used = 0
        occupied_marked = 0
        free_marked = 0

        for index in range(0, n, LIDAR_SAMPLE_STEP):
            distance = ranges[index]

            if not self.is_valid_number(distance):
                continue

            if distance < LIDAR_MIN_RANGE:
                continue

            ray_distance = min(distance, LIDAR_MAX_RANGE)

            if n > 1:
                rel_angle = -fov / 2 + index * fov / (n - 1)
            else:
                rel_angle = 0.0

            # Sinal calibrado para o referencial usado no teu mapa.
            global_angle = theta - rel_angle

            if not self.is_valid_number(global_angle):
                continue

            end_x = robot_x + ray_distance * math.cos(global_angle)
            end_y = robot_y + ray_distance * math.sin(global_angle)

            if not self.is_valid_number(end_x) or not self.is_valid_number(end_y):
                continue

            end_cell = self.world_to_grid(end_x, end_y)

            if end_cell is None:
                continue

            ray_cells = self.bresenham(robot_cell, end_cell)

            for row, col in ray_cells[:-1]:
                if self.is_inside_grid(row, col):
                    self.mark_cell(row, col, FREE)
                    free_marked += 1

            end_row, end_col = end_cell

            if self.is_inside_grid(end_row, end_col):
                if distance < LIDAR_MAX_RANGE * 0.98:
                    self.mark_cell(end_row, end_col, OCCUPIED)
                    occupied_marked += 1
                else:
                    self.mark_cell(end_row, end_col, FREE)
                    free_marked += 1

            lidar_rays_used += 1

        return {
            "source": "lidar",
            "robot_cell": robot_cell,
            "lidar_rays_used": lidar_rays_used,
            "occupied_marked": occupied_marked,
            "free_marked": free_marked,
            "theta": theta,
        }

    def build_planning_grid(self):
        """Cria uma grelha para A* com obstáculos inflados.

        A occupancy_grid guarda o mapa observado.
        A planning_grid é uma cópia onde células perto de OCCUPIED também ficam bloqueadas.
        """
        planning_grid = [
            [self.occupancy_grid[row][col] for col in range(MAP_WIDTH)]
            for row in range(MAP_HEIGHT)
        ]

        radius = OBSTACLE_INFLATION_RADIUS

        for row in range(MAP_HEIGHT):
            for col in range(MAP_WIDTH):
                if self.occupancy_grid[row][col] != OCCUPIED:
                    continue

                for d_row in range(-radius, radius + 1):
                    for d_col in range(-radius, radius + 1):
                        n_row = row + d_row
                        n_col = col + d_col

                        if not self.is_inside_grid(n_row, n_col):
                            continue

                        # Inflacao circular aproximada.
                        if d_row * d_row + d_col * d_col <= radius * radius:
                            if planning_grid[n_row][n_col] == FREE:
                                planning_grid[n_row][n_col] = OCCUPIED

        return planning_grid

    def find_nearest_free_cell_in_grid(self, grid, center_cell, max_radius=10):
        if center_cell is None:
            return None

        center_row, center_col = center_cell

        if not self.is_inside_grid(center_row, center_col):
            return None

        if grid[center_row][center_col] == FREE:
            return center_cell

        for radius in range(1, max_radius + 1):
            for row in range(center_row - radius, center_row + radius + 1):
                for col in range(center_col - radius, center_col + radius + 1):
                    if not self.is_inside_grid(row, col):
                        continue

                    if grid[row][col] == FREE:
                        return row, col

        return None

    def plan_path_to_target(self, target_pos):
        if target_pos is None:
            self.last_planned_path = []
            return []

        start_cell = self.mark_robot_cell_free()
        goal_cell = self.world_to_grid(target_pos[0], target_pos[1])

        if start_cell is None or goal_cell is None:
            self.last_planned_path = []
            return []

        if not self.is_inside_grid(goal_cell[0], goal_cell[1]):
            self.last_planned_path = []
            return []

        planning_grid = self.build_planning_grid()

        # A célula atual do robô deve continuar navegável,
        # mesmo que esteja próxima de um obstáculo inflado.
        start_row, start_col = start_cell
        planning_grid[start_row][start_col] = FREE

        reachable_goal = self.find_nearest_free_cell_in_grid(
            planning_grid,
            goal_cell,
            max_radius=12,
        )

        if reachable_goal is None:
            self.last_planned_path = []
            return []

        path = astar_grid(
            planning_grid,
            start_cell,
            reachable_goal,
            allow_diagonal=True,
        )

        self.last_planned_path = path

        return path

    def plan_path_to_candidates(self, target_candidates):
        if not target_candidates:
            self.last_planned_path = []
            self.last_selected_target_pos = None
            return None, []

        if self.last_selected_target_pos in target_candidates:
            path = self.plan_path_to_target(self.last_selected_target_pos)

            if path:
                self.last_planned_path = path
                return self.last_selected_target_pos, path

        best_target = None
        best_path = []

        for target_pos in target_candidates:
            path = self.plan_path_to_target(target_pos)

            if not path:
                continue

            if not best_path or len(path) < len(best_path):
                best_target = target_pos
                best_path = path

        self.last_selected_target_pos = best_target
        self.last_planned_path = best_path

        return best_target, best_path

    def print_path_debug(self, target_pos, path):
        now = 0.0

        if hasattr(self.manager.epuck, "robot") and self.manager.epuck is not None:
            now = self.manager.epuck.robot.getTime()

        if now - self.last_path_debug_time < PATH_DEBUG_INTERVAL:
            return

        start_cell = self.mark_robot_cell_free()
        goal_cell = None

        if target_pos is not None:
            goal_cell = self.world_to_grid(target_pos[0], target_pos[1])

        print(
            "[navigation_exp2] A* debug: "
            f"start={start_cell}, "
            f"goal={goal_cell}, "
            f"path_length={len(path)}"
        )

        self.last_path_debug_time = now

    def update(self):
        map_update = self.update_map_from_lidar()
        self.print_map_summary()

        return map_update

    def step(self, target_pos):
        map_update = self.update()

        now = 0.0
        if hasattr(self.manager.epuck, "robot") and self.manager.epuck is not None:
            now = self.manager.epuck.robot.getTime()

        should_replan = (
            now - self.last_path_plan_time >= PATH_REPLAN_INTERVAL
            or target_pos != self.cached_target_pos
            or not self.last_planned_path
        )

        if should_replan:
            path = self.plan_path_to_target(target_pos)
            self.cached_target_pos = target_pos
            self.last_path_plan_time = now
        else:
            path = self.last_planned_path

        self.print_path_debug(target_pos, path)

        return {
            "target_pos": target_pos,
            "map_update": map_update,
            "path_length": len(path),
            "path": path,
        }

    def step_to_candidates(self, target_candidates):
        map_update = self.update()
        target_key = tuple(target_candidates) if target_candidates else None

        now = 0.0
        if hasattr(self.manager.epuck, "robot") and self.manager.epuck is not None:
            now = self.manager.epuck.robot.getTime()

        should_replan = (
            now - self.last_path_plan_time >= PATH_REPLAN_INTERVAL
            or target_key != self.cached_target_pos
            or not self.last_planned_path
        )

        if should_replan:
            selected_target, path = self.plan_path_to_candidates(target_candidates)
            self.cached_target_pos = target_key
            self.last_path_plan_time = now
        else:
            selected_target = self.last_selected_target_pos
            path = self.last_planned_path

        self.print_path_debug(selected_target, path)

        return {
            "target_pos": selected_target,
            "map_update": map_update,
            "path_length": len(path),
            "path": path,
        }

    def print_map_summary(self):
        now = 0.0

        if hasattr(self.manager.epuck, "robot") and self.manager.epuck is not None:
            now = self.manager.epuck.robot.getTime()

        if now - self.last_summary_time < 2.0:
            return

        free_count = 0
        unknown_count = 0
        occupied_count = 0

        for row in self.occupancy_grid:
            for cell in row:
                if cell == FREE:
                    free_count += 1
                elif cell == OCCUPIED:
                    occupied_count += 1
                else:
                    unknown_count += 1

        print(
            "[navigation_exp2] map summary: "
            f"free={free_count}, unknown={unknown_count}, "
            f"occupied={occupied_count}"
        )

        self.print_local_map()

        self.last_summary_time = now

    def print_local_map(self, radius=8):
        robot_cell = self.mark_robot_cell_free()

        if robot_cell is None:
            print("[navigation_exp2] cannot print local map: robot cell unavailable")
            return

        robot_row, robot_col = robot_cell

        print("[navigation_exp2] local map:")

        path_cells = set(self.last_planned_path)

        for row in range(robot_row + radius, robot_row - radius - 1, -1):
            line = ""

            for col in range(robot_col - radius, robot_col + radius + 1):
                if not self.is_inside_grid(row, col):
                    line += " "
                    continue

                value = self.occupancy_grid[row][col]

                if row == robot_row and col == robot_col:
                    line += "R"
                elif (row, col) in path_cells:
                    line += "*"
                elif value == FREE:
                    line += "."
                elif value == OCCUPIED:
                    line += "#"
                else:
                    line += "?"

            print(line)
