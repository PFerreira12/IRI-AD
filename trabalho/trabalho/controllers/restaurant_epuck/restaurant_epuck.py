"""Controller comum para o e-puck do restaurante.

Este ficheiro prepara a gestao de pedidos e estados do robo. A navegacao real
por A* (EXP1) e por mapeamento incremental/SLAM simplificado (EXP2) fica para
uma fase seguinte.
"""
import math
from controller import Supervisor, Receiver, Emitter


TIME_STEP = 32
MAX_SPEED = 6.28
CRUISE_SPEED = 3.2
TURN_SPEED = 2.4
OBSTACLE_THRESHOLD = 80.0

WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.052

EXPERIMENT_MODE = "EXP1"
SERVICE_TIME = 2.0
REQUEST_CHANNEL = 1
DONE_CHANNEL = 2

STATE_IDLE = "IDLE"
STATE_GOING_TO_TABLE = "GOING_TO_TABLE"
STATE_SERVING = "SERVING"
STATE_RETURNING_TO_BASE = "RETURNING_TO_BASE"


class RestaurantEpuck:
    def __init__(self):
        self.robot = Supervisor()
        self.time_step = int(self.robot.getBasicTimeStep()) or TIME_STEP
        self.experiment_mode = EXPERIMENT_MODE

        self.left_motor = self._required_device("left wheel motor")
        self.right_motor = self._required_device("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.stop()

        self.left_encoder = self._optional_sensor("left wheel sensor")
        self.right_encoder = self._optional_sensor("right wheel sensor")

        self.proximity_sensors = [
            self._required_sensor(f"ps{index}") for index in range(8)
        ]

        self.lidar = self._optional_sensor("lidar")
        self.compass = self._optional_sensor("compass")
        self.camera = self._optional_sensor("camera")
        self.accelerometer = self._optional_sensor("accelerometer")

        self.leds = [
            led
            for index in range(10)
            if (led := self._optional_device(f"led{index}")) is not None
        ]

        self.receiver = self._optional_device("receiver")
        if self.receiver is not None:
            self.receiver.setChannel(REQUEST_CHANNEL)
            self.receiver.enable(self.time_step)

        self.emitter = self._optional_device("emitter")
        if self.emitter is not None:
            self.emitter.setChannel(DONE_CHANNEL)

        # Centros reais das mesas no mundo Webots.
        self.TABLES = {
            "T1": (-0.432, -0.312),
            "T2": (-0.168, -0.120),
            "T3": (-0.408, 0.204),
            "T4": (0.396, -0.252),
            "T5": (0.144, 0.084),
            "T6": (0.432, 0.336),
        }

        # Área de chegada de cada mesa.
        # Cada mesa é representada pelo centro da mesa + coordenadas das cadeiras.
        # O robô não precisa de atingir exatamente o centro da mesa.
        self.TABLE_REACH_POINTS = {
            "T1": [
                (-0.432, -0.312),
                (-0.432, -0.2136),
                (-0.432, -0.4104),
                (-0.3336, -0.312),
                (-0.5304, -0.312),
            ],
            "T2": [
                (-0.168, -0.120),
                (-0.168, -0.0216),
                (-0.168, -0.2184),
                (-0.0696, -0.120),
                (-0.2664, -0.120),
            ],
            "T3": [
                (-0.408, 0.204),
                (-0.408, 0.3024),
                (-0.408, 0.1056),
                (-0.3096, 0.204),
                (-0.5064, 0.204),
            ],
            "T4": [
                (0.396, -0.252),
                (0.396, -0.1536),
                (0.396, -0.3504),
                (0.4944, -0.252),
                (0.2976, -0.252),
            ],
            "T5": [
                (0.144, 0.084),
                (0.144, 0.1824),
                (0.144, -0.0144),
                (0.2424, 0.084),
                (0.0456, 0.084),
            ],
            "T6": [
                (0.432, 0.336),
                (0.432, 0.4344),
                (0.432, 0.2376),
                (0.5304, 0.336),
                (0.3336, 0.336),
            ],
        }

        # Posição inicial/base do robô.
        # O balcão está em (0.0, -0.486), mas o robô começa à frente dele.
        self.base_pos = (0.0, -0.39)

        # Raios para considerar chegada.
        # Para mesas usamos uma área maior, porque o robô pode chegar por vários lados.
        self.table_arrival_radius = 0.14
        self.base_arrival_radius = 0.08

        self.state = STATE_IDLE
        self.target_id = None
        self.target_pos = None
        self.last_served_id = None
        self.service_start_time = None

        # Fila FIFO de pedidos recebidos enquanto o robô está ocupado.
        self.request_queue = []

        # Pose estimada para EXP2.
        self.estimated_x = self.base_pos[0]
        self.estimated_y = self.base_pos[1]
        self.estimated_theta = 0.0
        self.previous_left_encoder = None
        self.previous_right_encoder = None

        self.last_encoder_warning_time = -999.0
        self.last_report_time = -1.0

        self._print_configuration_summary()

    def _required_device(self, name):
        device = self.robot.getDevice(name)
        if device is None:
            raise RuntimeError(f"Required Webots device not found: {name}")
        return device

    def _optional_device(self, name):
        try:
            return self.robot.getDevice(name)
        except Exception:
            return None

    def _required_sensor(self, name, sampling_period=None):
        sensor = self._required_device(name)
        sensor.enable(sampling_period or self.time_step)
        return sensor

    def _optional_sensor(self, name, sampling_period=None):
        sensor = self._optional_device(name)
        if sensor is not None:
            sensor.enable(sampling_period or self.time_step)
        return sensor

    def _print_configuration_summary(self):
        optional_devices = {
            "lidar": self.lidar is not None,
            "compass": self.compass is not None,
            "camera": self.camera is not None,
            "accelerometer": self.accelerometer is not None,
            "wheel_encoders": self.left_encoder is not None and self.right_encoder is not None,
            "receiver": self.receiver is not None,
            "emitter": self.emitter is not None,
        }

        enabled = ", ".join(
            name for name, available in optional_devices.items() if available
        )

        missing = ", ".join(
            name for name, available in optional_devices.items() if not available
        )

        print("[restaurant_epuck] Configured actuators: left/right wheel motors, LEDs")
        print(
            "[restaurant_epuck] Configured sensors: ps0..ps7"
            + (f", {enabled}" if enabled else "")
        )
        print(f"[restaurant_epuck] Experiment mode: {self.experiment_mode}")
        print(
            "[restaurant_epuck] communication channels: "
            f"REQ<-{REQUEST_CHANNEL}, DONE->{DONE_CHANNEL}"
        )

        if missing:
            print(f"[restaurant_epuck] Optional devices not present in this robot: {missing}")

    def set_wheel_speeds(self, left_speed, right_speed):
        left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
        right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))

        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def stop(self):
        self.set_wheel_speeds(0.0, 0.0)

    def set_status_leds(self, active):
        for index, led in enumerate(self.leds):
            led.set(1 if active or index % 2 == 0 else 0)

    def proximity_values(self):
        return [sensor.getValue() for sensor in self.proximity_sensors]

    def front_obstacle_levels(self):
        values = self.proximity_values()

        # Sensores frontais do e-puck.
        left_front = max(values[5], values[6], values[7])
        right_front = max(values[0], values[1], values[2])

        return left_front, right_front

    def update_odometry(self):
        if self.left_encoder is None or self.right_encoder is None:
            now = self.robot.getTime()

            if now - self.last_encoder_warning_time >= 5.0:
                print("[restaurant_epuck] wheel encoders unavailable; keeping last estimated pose")
                self.last_encoder_warning_time = now

            return

        left_value = self.left_encoder.getValue()
        right_value = self.right_encoder.getValue()

        if self.previous_left_encoder is None or self.previous_right_encoder is None:
            self.previous_left_encoder = left_value
            self.previous_right_encoder = right_value
            return

        delta_left = (left_value - self.previous_left_encoder) * WHEEL_RADIUS
        delta_right = (right_value - self.previous_right_encoder) * WHEEL_RADIUS

        self.previous_left_encoder = left_value
        self.previous_right_encoder = right_value

        delta_center = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / AXLE_LENGTH

        self.estimated_theta += delta_theta
        self.estimated_x += delta_center * math.cos(self.estimated_theta)
        self.estimated_y += delta_center * math.sin(self.estimated_theta)

    def get_robot_position(self):
        if self.experiment_mode == "EXP1":
            return self.get_ground_truth_position()

        if self.experiment_mode == "EXP2":
            return self.get_estimated_position()

        raise ValueError(f"Unsupported experiment mode: {self.experiment_mode}")

    def get_ground_truth_position(self):
        # Ground truth do simulador: útil para validação/EXP1.
        # Em EXP2, a fonte principal deve ser get_estimated_position().
        try:
            node = self.robot.getSelf()

            if node is None:
                return None

            position = node.getPosition()

            # Neste mundo, o plano horizontal usa X e Y.
            return position[0], position[1]

        except Exception as exc:
            now = self.robot.getTime()

            if now - self.last_encoder_warning_time >= 5.0:
                print(f"[restaurant_epuck] ground truth unavailable: {exc}")
                self.last_encoder_warning_time = now

            return None

    def get_estimated_position(self):
        return self.estimated_x, self.estimated_y

    def has_arrived(self, target_pos, radius):
        current_pos = self.get_robot_position()

        if current_pos is None or target_pos is None:
            return False

        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]

        return math.hypot(dx, dy) <= radius

    def distance_to_point(self, point):
        current_pos = self.get_robot_position()

        if current_pos is None or point is None:
            return None

        return math.hypot(
            point[0] - current_pos[0],
            point[1] - current_pos[1],
        )

    def distance_to_table_area(self, table_id):
        current_pos = self.get_robot_position()

        if current_pos is None or table_id not in self.TABLE_REACH_POINTS:
            return None

        robot_x, robot_y = current_pos

        distances = [
            math.hypot(point_x - robot_x, point_y - robot_y)
            for point_x, point_y in self.TABLE_REACH_POINTS[table_id]
        ]

        return min(distances)

    def has_reached_table_area(self, table_id):
        distance = self.distance_to_table_area(table_id)

        if distance is None:
            return False

        return distance <= self.table_arrival_radius

    def notify_request_done(self, table_id):
        if table_id is None:
            return

        msg = f"DONE {table_id}"

        if self.emitter is None:
            print(f"[restaurant_epuck] cannot send {msg}: emitter unavailable")
            return

        self.emitter.send(msg.encode("utf-8"))
        print(f"[restaurant_epuck] sent: {msg}")

    def start_request(self, table_id):
        if table_id not in self.TABLES:
            print(f"[restaurant_epuck] cannot start unknown table: {table_id}")
            return False

        self.target_id = table_id

        # O target_pos continua a ser o centro da mesa por agora.
        # A deteção de chegada usa a área da mesa e cadeiras.
        self.target_pos = self.TABLES[table_id]

        self.state = STATE_GOING_TO_TABLE

        print(
            f"[restaurant_epuck] NEW REQUEST: {table_id} -> "
            f"table_center={self.target_pos}"
        )

        return True

    def navigation_step(self, target_pos):
        # TODO EXP1: substituir por A* sobre mapa/grelha conhecido.
        # TODO EXP2: substituir por mapeamento incremental + A* no mapa conhecido.
        #
        # Este comportamento ainda é apenas reativo:
        # anda em frente e roda quando deteta obstáculo.

        left_front, right_front = self.front_obstacle_levels()
        blocked = max(left_front, right_front) > OBSTACLE_THRESHOLD

        if blocked and left_front >= right_front:
            left_speed, right_speed = TURN_SPEED, -TURN_SPEED

        elif blocked:
            left_speed, right_speed = -TURN_SPEED, TURN_SPEED

        else:
            left_speed, right_speed = CRUISE_SPEED, CRUISE_SPEED

        self.set_status_leds(blocked)
        self.set_wheel_speeds(left_speed, right_speed)

    def process_requests(self):
        if self.receiver is None:
            return

        while self.receiver.getQueueLength() > 0:
            msg = self.receiver.getString().strip()
            self.receiver.nextPacket()

            parts = msg.split()

            if len(parts) != 2 or parts[0] != "REQ":
                print(f"[restaurant_epuck] invalid message ignored: {msg}")
                continue

            table_id = parts[1]

            if table_id not in self.TABLES:
                print(f"[restaurant_epuck] unknown table id ignored: {table_id}")
                continue

            if self.state == STATE_IDLE:
                self.start_request(table_id)
                continue

            if table_id == self.target_id:
                print(
                    "[restaurant_epuck] duplicate request ignored; already serving "
                    f"{table_id}"
                )
                continue

            if table_id in self.request_queue:
                print(
                    "[restaurant_epuck] duplicate request ignored; already queued "
                    f"{table_id}"
                )
                continue

            self.request_queue.append(table_id)

            print(
                f"[restaurant_epuck] queued request: {table_id} | "
                f"queue={self.request_queue}"
            )

    def report_debug(self):
        now = self.robot.getTime()

        if now - self.last_report_time < 1.0:
            return

        position = self.get_robot_position()

        table_distance = None
        if self.target_id is not None:
            table_distance = self.distance_to_table_area(self.target_id)

        base_distance = None
        if self.state == STATE_RETURNING_TO_BASE:
            base_distance = self.distance_to_point(self.base_pos)

        left_front, right_front = self.front_obstacle_levels()

        print(
            "[restaurant_epuck] status "
            f"mode={self.experiment_mode} "
            f"state={self.state} "
            f"target={self.target_id} "
            f"last_served={self.last_served_id} "
            f"queue={self.request_queue} "
            f"pos={position} "
            f"table_dist={table_distance} "
            f"base_dist={base_distance} "
            f"front=({left_front:.1f}, {right_front:.1f})"
        )

        self.last_report_time = now

    def run_state_machine(self):
        while self.robot.step(self.time_step) != -1:
            self.process_requests()
            self.update_odometry()

            now = self.robot.getTime()

            if self.state == STATE_IDLE:
                self.stop()
                self.set_status_leds(False)

            elif self.state == STATE_GOING_TO_TABLE:
                self.navigation_step(self.target_pos)

                if self.has_reached_table_area(self.target_id):
                    self.stop()
                    self.service_start_time = now
                    self.state = STATE_SERVING

                    print(
                        f"[restaurant_epuck] arrived at service area for "
                        f"{self.target_id}; serving"
                    )

            elif self.state == STATE_SERVING:
                self.stop()
                self.set_status_leds(True)

                if (
                    self.service_start_time is not None
                    and now - self.service_start_time >= SERVICE_TIME
                ):
                    completed_table = self.target_id

                    self.notify_request_done(completed_table)

                    self.last_served_id = completed_table
                    self.target_id = None
                    self.target_pos = self.base_pos
                    self.state = STATE_RETURNING_TO_BASE

                    print(f"[restaurant_epuck] returning to base from {completed_table}")

            elif self.state == STATE_RETURNING_TO_BASE:
                self.navigation_step(self.base_pos)

                if self.has_arrived(self.base_pos, self.base_arrival_radius):
                    self.stop()

                    print("[restaurant_epuck] arrived at base; ready for new requests")

                    self.target_id = None
                    self.target_pos = None
                    self.service_start_time = None

                    if self.request_queue:
                        next_table = self.request_queue.pop(0)
                        print(f"[restaurant_epuck] next FIFO request: {next_table}")
                        self.start_request(next_table)

                    else:
                        self.state = STATE_IDLE
                        self.stop()

            else:
                print(f"[restaurant_epuck] unknown state {self.state}; stopping")
                self.stop()
                self.state = STATE_IDLE

            self.report_debug()


if __name__ == "__main__":
    RestaurantEpuck().run_state_machine()