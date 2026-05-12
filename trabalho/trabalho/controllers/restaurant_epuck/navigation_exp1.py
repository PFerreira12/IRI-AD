from astar_grid import astar_grid

FREE=0

class NavigationExp1:

    def __init__(self, controller, known_map):
        self.controller = controller
        self.map = known_map

        self.cached_target_pos = None
        self.last_planned_path = []


    def plan_path_to_target(self, target_candidates):

        robot_pos = self.controller.get_robot_position()
        start_cell = self.map.world_to_grid(*robot_pos)

        best_path = []

        for candidate in target_candidates:

            goal_cell = self.map.world_to_grid(*candidate)

            print("\n[TRY GOAL]", candidate)
            print("goal cell:", goal_cell)

            path = astar_grid(
                self.map.grid,
                start_cell,
                goal_cell,
                allow_diagonal=True
            )

            if path:
                print("[SUCCESS] path found")
                return path

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

        # usa o primeiro candidato como cache key
        current_target = tuple(target_candidates[0])

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