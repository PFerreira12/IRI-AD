from astar_grid import astar_grid

FREE = 0

class NavigationExp1:

    def __init__(self, controller, known_map):
        self.controller = controller
        self.map = known_map

        self.cached_target_pos = None
        self.last_planned_path = []
        self.current_waypoint_index = 1

    def is_free_cell(self, cell):
        if cell is None:
            return False

        row, col = cell

        if row < 0 or row >= self.map.height:
            return False

        if col < 0 or col >= self.map.width:
            return False

        return self.map.grid[row][col] == FREE

    def nearest_free_cell(self, center_cell, max_radius=6):
        if center_cell is None:
            return None

        if self.is_free_cell(center_cell):
            return center_cell

        center_row, center_col = center_cell

        for radius in range(1, max_radius + 1):
            for row in range(center_row - radius, center_row + radius + 1):
                for col in range(center_col - radius, center_col + radius + 1):
                    cell = (row, col)

                    if self.is_free_cell(cell):
                        return cell

        return None

    def plan_path_to_target(self, target_candidates):

        robot_pos = self.controller.get_robot_position()

        if robot_pos is None:
            return []

        start_cell = self.map.world_to_grid(*robot_pos)
        start_cell = self.nearest_free_cell(start_cell, max_radius=3)

        if start_cell is None:
            print("[EXP1] no free start cell available")
            return []

        best_path = []
        best_goal_cell = None

        for candidate in target_candidates:

            goal_cell = self.map.world_to_grid(*candidate)
            reachable_goal = self.nearest_free_cell(goal_cell)

            print("\n[TRY GOAL]", candidate)
            print("goal cell:", goal_cell, "reachable:", reachable_goal)

            if reachable_goal is None:
                continue

            path = astar_grid(
                self.map.grid,
                start_cell,
                reachable_goal,
                allow_diagonal=True
            )

            if path:
                if not best_path or len(path) < len(best_path):
                    best_path = path
                    best_goal_cell = reachable_goal

        if best_path:
            print("[SUCCESS] path found to", best_goal_cell)
            return best_path

        print("[FAIL] no candidate worked")
        return []
    
    
    def step(self, target_candidates):

        # sem candidatos
        if not target_candidates:
            self.cached_target_pos = None
            self.last_planned_path = []

            return {
                "path": [],
                "path_length": 0
            }

        current_target = tuple(target_candidates)

        should_replan = (
            self.cached_target_pos != current_target
            or not self.last_planned_path
        )

        if should_replan:

            print("\n[EXP1] replanning path...")
            print("[EXP1] candidates:", target_candidates)

            path = self.plan_path_to_target(target_candidates)
            print("[EXP1] path len:", len(path))

            self.cached_target_pos = current_target
            self.last_planned_path = path
            self.current_waypoint_index = 1

        else:
            path = self.last_planned_path

        return {
            "path_length": len(path),
            "path": path,
    }
