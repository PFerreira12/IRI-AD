from controller import Supervisor
import math
import os, sys
from pathlib import Path
CURRENT_DIR = Path(__file__).resolve().parent
CONTROLLERS_DIR = CURRENT_DIR.parent
COMMON_DIR = CONTROLLERS_DIR / "common"

sys.path.insert(0, str(COMMON_DIR))
sys.path.append(str(CONTROLLERS_DIR))


from config_tables import get_map_config

"""Configurable dynamic obstacles manager.

The previous controller contained one fixed set of circuits for Map 1. This
version reads the dynamic-obstacle routes from config_tables.py, so the same
controller can be used with MAP_ID=map1 or MAP_ID=map2.
"""

TIME_STEP = 32


def env_bool(name, default=False):
    raw_value = os.environ.get(name)
    if raw_value is None:
        return default
    return str(raw_value).strip().lower() in {"1", "true", "yes", "on"}


DYNAMIC_ENVIRONMENT = env_bool("DYNAMIC_ENVIRONMENT", True)


def distance_2d(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


class Circuit:
    def __init__(self, name, points_xy):
        self.name = name
        self.points_xy = points_xy
        self.segment_lengths = []
        self.total_length = 0.0

        for index in range(len(points_xy)):
            point_a = points_xy[index]
            point_b = points_xy[(index + 1) % len(points_xy)]
            length = distance_2d(point_a, point_b)
            self.segment_lengths.append(length)
            self.total_length += length

    def position_xy_at(self, progress):
        """progress in [0, 1). Returns an (x, y) point along the closed circuit."""
        if not self.points_xy:
            return 0.0, 0.0

        if len(self.points_xy) == 1 or self.total_length <= 0:
            return self.points_xy[0]

        target_distance = (progress % 1.0) * self.total_length
        accumulated = 0.0

        for index, segment_length in enumerate(self.segment_lengths):
            if accumulated + segment_length >= target_distance:
                point_a = self.points_xy[index]
                point_b = self.points_xy[(index + 1) % len(self.points_xy)]

                local_distance = target_distance - accumulated
                alpha = local_distance / segment_length if segment_length > 0 else 0.0

                x = point_a[0] + (point_b[0] - point_a[0]) * alpha
                y = point_a[1] + (point_b[1] - point_a[1]) * alpha
                return x, y

            accumulated += segment_length

        return self.points_xy[-1]


class MovingObstacle:
    def __init__(
        self,
        supervisor,
        def_name,
        circuits,
        active_windows,
        lap_time,
        z_height,
        circuit_offset=0,
        hidden_pos=(0.0, 0.0, -2.0),
    ):
        self.supervisor = supervisor
        self.def_name = def_name
        self.circuits = circuits
        self.active_windows = active_windows
        self.lap_time = lap_time
        self.z_height = z_height
        self.circuit_offset = circuit_offset
        self.hidden_pos = hidden_pos

        self.node = self.supervisor.getFromDef(def_name)

        if self.node is None:
            print(f"[dynamic_obstacles] ERROR: {def_name} not found.")
            self.translation_field = None
        else:
            print(f"[dynamic_obstacles] {def_name} found.")
            self.translation_field = self.node.getField("translation")

    def is_active(self, scenario_time):
        for start, end in self.active_windows:
            if start <= scenario_time < end:
                return True
        return False

    def get_circuit_for_cycle(self, cycle_index):
        if not self.circuits:
            return None

        index = (cycle_index + self.circuit_offset) % len(self.circuits)
        return self.circuits[index]

    def update(self, current_time, scenario_time, cycle_index):
        if self.translation_field is None:
            return

        if not DYNAMIC_ENVIRONMENT:
            self.translation_field.setSFVec3f(list(self.hidden_pos))
            return

        if not self.is_active(scenario_time):
            self.translation_field.setSFVec3f(list(self.hidden_pos))
            return

        circuit = self.get_circuit_for_cycle(cycle_index)
        if circuit is None:
            self.translation_field.setSFVec3f(list(self.hidden_pos))
            return

        progress = (current_time % self.lap_time) / self.lap_time
        x, y = circuit.position_xy_at(progress)
        self.translation_field.setSFVec3f([x, y, self.z_height])


class DynamicObstaclesManager:
    def __init__(self):
        self.robot = Supervisor()
        self.time_step = int(self.robot.getBasicTimeStep()) or TIME_STEP

        self.map_config = get_map_config()
        dyn_config = self.map_config["dynamic_obstacles"]
        self.scenario_period = dyn_config.get("scenario_period", 95.0)

        self.circuits = [
            Circuit(name, points)
            for name, points in dyn_config.get("circuits", {}).items()
        ]

        self.obstacles = []
        for obstacle_config in dyn_config.get("obstacles", []):
            self.obstacles.append(
                MovingObstacle(
                    supervisor=self.robot,
                    def_name=obstacle_config["def_name"],
                    circuits=self.circuits,
                    active_windows=obstacle_config["active_windows"],
                    lap_time=obstacle_config["lap_time"],
                    z_height=obstacle_config["z_height"],
                    circuit_offset=obstacle_config.get("circuit_offset", 0),
                    hidden_pos=obstacle_config.get("hidden_pos", (0.0, 0.0, -2.0)),
                )
            )

        circuit_names = [circuit.name for circuit in self.circuits]
        print(
            "[dynamic_obstacles] configuration "
            f"map={self.map_config['id']} "
            f"enabled={DYNAMIC_ENVIRONMENT} "
            f"scenario_period={self.scenario_period:.1f}s "
            f"circuits={circuit_names}"
        )

    def run(self):
        while self.robot.step(self.time_step) != -1:
            current_time = self.robot.getTime()
            cycle_index = int(current_time // self.scenario_period)
            scenario_time = current_time % self.scenario_period

            for obstacle in self.obstacles:
                obstacle.update(current_time, scenario_time, cycle_index)


if __name__ == "__main__":
    DynamicObstaclesManager().run()
