class NavigationEXP1:

    def __init__(self, known_map, planner):
        self.map = known_map
        self.planner = planner

        self.path = []
        self.index = 0


    def set_mission(self, start_world, goal_world):
        start = self.map.world_to_grid(*start_world)
        print("start grid: ", start)
        goal = self.map.world_to_grid(*goal_world)
        print("goal grid: ", goal)

        self.path = self.planner.plan(start, goal)
        print("path: ", self.path)
        self.index = 0


    def get_next(self):
        if self.index >= len(self.path):
            return None
        return self.path[self.index]


    def update(self, current_grid, reached_fn):
        if self.index >= len(self.path):
            return True

        target = self.path[self.index]

        if reached_fn(current_grid, target):
            self.index += 1

        return self.index >= len(self.path)


    def is_finished(self):
        return self.index >= len(self.path)