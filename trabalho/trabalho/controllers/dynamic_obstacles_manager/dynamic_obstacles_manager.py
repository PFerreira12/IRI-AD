from controller import Supervisor
import math

"""Dynamic obstacles manager for the restaurant environment.

This controller creates a reproducible dynamic scenario with three moving
obstacles that represent people or temporary obstacles inside the restaurant.

Main idea:
- The scenario runs in a 95-second repeating cycle.
- Each obstacle is active only during specific time windows.
- When inactive, the obstacle is moved below the floor so it does not affect
  the robot or the simulation.
- Each obstacle follows one predefined circuit at a time.
- At the end of each 95-second cycle, the circuits rotate between obstacles,
  so the same obstacle does not always follow the same route.
- The circuits are defined only with (x, y) coordinates; each obstacle keeps
  its own z_height according to its cylinder height.

Scenario timing:
    0–20s      MOVING_PERSON_1 active
    20–40s     MOVING_PERSON_2 active
    40–60s     MOVING_PERSON_3 active
    60–85s     all three obstacles active
    85–95s     no dynamic obstacles active
    then the cycle repeats

Obstacle roles:
    MOVING_PERSON_1:
        beige/yellow cylinder, slow speed, taller and thinner

    MOVING_PERSON_2:
        blue/grey cylinder, medium speed, medium size

    MOVING_PERSON_3:
        red/orange cylinder, fast speed, shorter obstacle

Circuits:
- Top corridor route: moves along the upper corridor without crossing plants,
  chairs or the counter.
- Around-table route: moves around the central table area instead of crossing
  through the table/chairs.
- Lower corridor route: moves between the lower plants, passing around tables
  and keeping a safety distance from the counter.

Purpose:
This setup allows testing how the robot reacts to dynamic obstacles while
keeping the experiment controlled and reproducible. The robot does not receive
direct messages from this controller; it detects the moving obstacles through
its LiDAR and proximity sensors.
"""

TIME_STEP = 32
DYNAMIC_ENVIRONMENT = False

SCENARIO_PERIOD = 95.0

WINDOW_PERSON_1 = [(0.0, 20.0), (60.0, 85.0)]
WINDOW_PERSON_2 = [(20.0, 40.0), (60.0, 85.0)]
WINDOW_PERSON_3 = [(40.0, 60.0), (60.0, 85.0)]


def distance_2d(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


class Circuit:
    def __init__(self, name, points_xy):
        self.name = name
        self.points_xy = points_xy
        self.segment_lengths = []
        self.total_length = 0.0

        for i in range(len(points_xy)):
            a = points_xy[i]
            b = points_xy[(i + 1) % len(points_xy)]

            length = distance_2d(a, b)
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

        for i, segment_length in enumerate(self.segment_lengths):
            if accumulated + segment_length >= target_distance:
                a = self.points_xy[i]
                b = self.points_xy[(i + 1) % len(self.points_xy)]

                local_distance = target_distance - accumulated
                alpha = local_distance / segment_length if segment_length > 0 else 0.0

                x = a[0] + (b[0] - a[0]) * alpha
                y = a[1] + (b[1] - a[1]) * alpha

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

        circuit_top = Circuit(
            "top_corridor",
            [
                (-0.46, 0.40),
                (-0.25, 0.40),
                (-0.05, 0.40),
                (0.15, 0.40),
                (0.28, 0.40),
            ],
        )

        # Circuito 2: contorna a mesa central, sem atravessar mesa nem cadeiras.
        circuit_around_table = Circuit(
            "around_table",
            [
                (-0.04, 0.25),
                (0.33, 0.25),
                (0.33, -0.10),
                (-0.02, -0.08),
            ],
        )

        circuit_base = Circuit(
            "base_area",
            [
                 # começa perto da planta direita, sem tocar nela
                (0.52, -0.41),
                (0.50, -0.43),

                # desvio PARA FORA da mesa 4: por baixo da chair_4_2
                (0.44, -0.43),
                (0.36, -0.43),
                (0.28, -0.42),

                # sobe ligeiramente depois de ultrapassar a zona da mesa 4
                (0.20, -0.40),
                (0.10, -0.395),
                (0.00, -0.395),

                # segue junto ao balcão, mas sem entrar nele
                (-0.12, -0.395),
                (-0.20, -0.400),

                # desvio forte ANTES da cadeira da mesa 1
                (-0.26, -0.430),
                (-0.32, -0.465),

                # passa claramente por baixo da chair_1_2
                (-0.40, -0.480),
                (-0.48, -0.480),

                # sobe só depois de ultrapassar a cadeira
                (-0.54, -0.455),
                (-0.55, -0.425),

                # regresso pelo mesmo caminho, para não cortar por dentro
                (-0.54, -0.455),
                (-0.48, -0.480),
                (-0.40, -0.480),
                (-0.32, -0.465),
                (-0.26, -0.430),
                (-0.20, -0.400),
                (-0.12, -0.395),
                (0.00, -0.395),
                (0.10, -0.395),
                (0.20, -0.40),
                (0.28, -0.42),
                (0.36, -0.43),
                (0.44, -0.43),
                (0.50, -0.43),
            ],
        )

        self.circuits = [
            circuit_top,
            circuit_around_table,
            circuit_base,
        ]

        self.obstacles = [
            MovingObstacle(
                supervisor=self.robot,
                def_name="MOVING_PERSON_1",
                circuits=self.circuits,
                active_windows=WINDOW_PERSON_1,
                lap_time=14.0,
                z_height=0.07,
                circuit_offset=0,
            ),
            MovingObstacle(
                supervisor=self.robot,
                def_name="MOVING_PERSON_2",
                circuits=self.circuits,
                active_windows=WINDOW_PERSON_2,
                lap_time=9.0,
                z_height=0.055,
                circuit_offset=1,
            ),
            MovingObstacle(
                supervisor=self.robot,
                def_name="MOVING_PERSON_3",
                circuits=self.circuits,
                active_windows=WINDOW_PERSON_3,
                lap_time=6.0,
                z_height=0.045,
                circuit_offset=2,
            ),
        ]

    def run(self):
        while self.robot.step(self.time_step) != -1:
            current_time = self.robot.getTime()
            cycle_index = int(current_time // SCENARIO_PERIOD)
            scenario_time = current_time % SCENARIO_PERIOD

            for obstacle in self.obstacles:
                obstacle.update(current_time, scenario_time, cycle_index)


if __name__ == "__main__":
    DynamicObstaclesManager().run()
