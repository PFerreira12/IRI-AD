"""Common controller for the restaurant's e-puck.

This file manages orders, robot states, estimated pose, and movement.

In EXP2, it uses incremental map + A* when a path exists, and reactive navigation only as a fallback/emergency.
"""

import math
import os, sys
from controller import Supervisor, Receiver, Emitter
from map_utils import save_grid_png
from pathlib import Path

CURRENT_DIR = Path(__file__).resolve().parent
CONTROLLERS_DIR = CURRENT_DIR.parent
COMMON_DIR = CONTROLLERS_DIR / "common"

sys.path.insert(0, str(COMMON_DIR))

from config_tables import get_map_config

TIME_STEP = 32
MAX_SPEED = 6.28

CRUISE_SPEED = 3.2
TURN_SPEED = 2.4

# Navegação reativa fallback.
NAV_WARNING_THRESHOLD = 78.0
NAV_OBSTACLE_THRESHOLD = 85.0

# Calibracao inicial dos sensores.
SENSOR_CALIBRATION_SAMPLES = 20
PROXIMITY_BASELINE_MARGIN = 2.0
LIDAR_SMOOTHING_ALPHA = 0.35
LIDAR_SECTOR_PERCENTILE = 0.20

# Proteção de odometria.
ODOMETRY_BLOCK_THRESHOLD = 78.0

# Lidar para fallback reativo.
LIDAR_NAV_MIN_VALID_DISTANCE = 0.06
LIDAR_CLEAR_DISTANCE = 0.18
LIDAR_CAUTION_DISTANCE = 0.25
LIDAR_FRONT_DEGREES = 30
LIDAR_SIDE_DEGREES = 75
DEFAULT_LIDAR_FOV = 6.28

# Seguimento de A*.
WAYPOINT_LOOKAHEAD_INDEX = 4
WAYPOINT_REACHED_RADIUS = 0.04
ANGLE_TOLERANCE = 0.25
PATH_FORWARD_SPEED = 1.8
PATH_TURN_SPEED = 1.2
PATH_MIN_FORWARD_SPEED = 0.35

# Durante path following, só abandona o caminho se houver emergência real.
PATH_EMERGENCY_LIDAR_DISTANCE = 0.11
PATH_HARD_MIN_LIDAR_DISTANCE = 0.07
RECOVERY_BACKUP_TIME = 0.5
RECOVERY_TOTAL_TIME = 1.05
RECOVERY_BACKUP_SPEED = -1.2
RECOVERY_TURN_SPEED = 1.8

# Detecao simples de bloqueio: se o robo nao reduz a distancia ao alvo
# durante alguns segundos, assume que ficou preso junto a uma cadeira/mesa.
STUCK_TIMEOUT = 6.0
STUCK_MIN_PROGRESS = 0.01
STUCK_RECOVERY_TOTAL_TIME = 1.8

WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.052

EXP1_MODE = "EXP1"
EXP2_MODE = "EXP2"
EXPERIMENT_MODE = EXP2_MODE
SERVICE_TIME = 2.0
REQUEST_CHANNEL = 1
DONE_CHANNEL = 2

STATE_IDLE = "IDLE"
STATE_GOING_TO_TABLE = "GOING_TO_TABLE"
STATE_SERVING = "SERVING"
STATE_RETURNING_TO_BASE = "RETURNING_TO_BASE"

POLICY_FIFO = "FIFO"
POLICY_NEAREST = "NEAREST"
POLICY_HYBRID = "HYBRID"
VALID_REQUEST_POLICIES = {POLICY_FIFO, POLICY_NEAREST, POLICY_HYBRID}


def env_float(name, default):
    raw_value = os.environ.get(name)
    if raw_value is None:
        return default

    try:
        return float(raw_value)
    except ValueError:
        print(f"[restaurant_epuck] invalid {name}={raw_value!r}; using {default}")
        return default


class RestaurantEpuck:
    def __init__(self):
        self.robot = Supervisor()
        self.epuck = self
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

        self.map_config = get_map_config()
        self.map_id = self.map_config["id"]
        self.TABLES = self.map_config["tables"]
        self.TABLE_REACH_POINTS = self.map_config["table_reach_points"]
        self.base_pos = self.map_config["base_pos"]
        self.table_arrival_radius = self.map_config.get("table_arrival_radius", 0.15)
        self.base_arrival_radius = self.map_config.get("base_arrival_radius", 0.05)

        self.state = STATE_IDLE
        self.target_id = None
        self.target_pos = None
        self.target_candidates = []
        
        self.last_served_id = None
        self.service_start_time = None
        self.request_queue = []
        self.request_created_times = {}
        self.current_request_created_time = None
        self.total_requests = 0
        self.completed_requests = 0

        self.run_id = os.environ.get("SIM_RUN_ID", "manual")
        self.request_policy = os.environ.get("REQUEST_POLICY", POLICY_NEAREST).upper()
        if self.request_policy not in VALID_REQUEST_POLICIES:
            print(
                "[restaurant_epuck] unsupported REQUEST_POLICY="
                f"{self.request_policy!r}; using FIFO"
            )
            self.request_policy = POLICY_FIFO

        self.hybrid_wait_weight = env_float("HYBRID_WAIT_WEIGHT", 1.0)
        self.hybrid_distance_weight = env_float("HYBRID_DISTANCE_WEIGHT", 30.0)

        self.estimated_x = self.base_pos[0]
        self.estimated_y = self.base_pos[1]
        self.estimated_theta = 0.0

        self.proximity_baseline = [0.0 for _ in range(8)]
        self.lidar_sector_filtered = {
            "front": None,
            "left": None,
            "right": None,
        }

        self.initial_compass_heading = None

        self.previous_left_encoder = None
        self.previous_right_encoder = None

        self.last_encoder_warning_time = -999.0
        self.last_report_time = -1.0
        self.obstacle_recovery_start_time = None
        self.obstacle_recovery_until = 0.0
        self.obstacle_recovery_direction = 1
        self.best_navigation_distance = None
        self.last_navigation_progress_time = None

        self.known_map = None
        self.navigation_exp1 = None
        self.navigation_exp2 = None
        self.configure_experiment()
        self._print_configuration_summary()
        self.calibrate_sensors()

        self.mission_start_time = None
        self.total_distance = 0.0
        self.prev_pos = self.get_robot_position()
        self.delivery_start_time = None
        self.wait_times = []
        self.delivery_times = []
        self.return_start_time = None
        self.return_times = []
        self.near_collision_count = 0
        self.in_safety_recovery = False

    def configure_experiment(self):
        if self.experiment_mode == EXP1_MODE:
            from known_map import KnownMap
            from navigation_exp1 import NavigationExp1

            self.known_map = KnownMap(self.map_config)
            self.navigation_exp1 = NavigationExp1(self, self.known_map)
            return

        if self.experiment_mode == EXP2_MODE:
            from navigation_exp2 import NavigationExp2

            self.navigation_exp2 = NavigationExp2(self)
            return

        raise ValueError(f"Unsupported experiment mode: {self.experiment_mode}")

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
        print(f"[restaurant_epuck] Map selected: {self.map_id} ({self.map_config.get('name', 'unnamed map')})")
        print(
            "[restaurant_epuck] request selection: "
            f"run={self.run_id} "
            f"policy={self.request_policy} "
            f"hybrid_wait_weight={self.hybrid_wait_weight:.2f} "
            f"hybrid_distance_weight={self.hybrid_distance_weight:.2f}"
        )
        print(
            "[restaurant_epuck] communication channels: "
            f"REQ<-{REQUEST_CHANNEL}, DONE->{DONE_CHANNEL}"
        )

        if missing:
            print(f"[restaurant_epuck] Optional devices not present in this robot: {missing}")

    def is_valid_number(self, value):
        return (
            value is not None
            and not math.isnan(value)
            and not math.isinf(value)
        )

    def median(self, values):
        clean_values = [
            value for value in values
            if self.is_valid_number(value)
        ]

        if not clean_values:
            return None

        clean_values.sort()
        mid = len(clean_values) // 2

        if len(clean_values) % 2 == 1:
            return clean_values[mid]

        return (clean_values[mid - 1] + clean_values[mid]) / 2.0

    def percentile(self, values, fraction):
        clean_values = [
            value for value in values
            if self.is_valid_number(value)
        ]

        if not clean_values:
            return None

        clean_values.sort()
        index = int(round((len(clean_values) - 1) * fraction))
        index = max(0, min(len(clean_values) - 1, index))

        return clean_values[index]

    def average_angles(self, angles):
        clean_angles = [
            angle for angle in angles
            if self.is_valid_number(angle)
        ]

        if not clean_angles:
            return None

        sin_sum = sum(math.sin(angle) for angle in clean_angles)
        cos_sum = sum(math.cos(angle) for angle in clean_angles)

        return math.atan2(sin_sum, cos_sum)

    def calibrate_sensors(self):
        print("[restaurant_epuck] calibrating sensors...")
        self.stop()

        proximity_samples = [[] for _ in range(8)]
        compass_samples = []

        for _ in range(SENSOR_CALIBRATION_SAMPLES):
            if self.robot.step(self.time_step) == -1:
                break

            raw_values = self.raw_proximity_values()
            for index, value in enumerate(raw_values):
                proximity_samples[index].append(value)

            raw_heading = self.get_raw_compass_heading()
            if raw_heading is not None:
                compass_samples.append(raw_heading)

        for index, samples in enumerate(proximity_samples):
            baseline = self.median(samples)
            self.proximity_baseline[index] = baseline if baseline is not None else 0.0

        compass_heading = self.average_angles(compass_samples)
        if compass_heading is not None:
            self.initial_compass_heading = compass_heading
            self.estimated_theta = 0.0

        if self.left_encoder is not None and self.right_encoder is not None:
            self.previous_left_encoder = self.left_encoder.getValue()
            self.previous_right_encoder = self.right_encoder.getValue()

        print(
            "[restaurant_epuck] sensor calibration "
            f"ps_baseline={[round(value, 1) for value in self.proximity_baseline]} "
            f"compass_zero={self.initial_compass_heading}"
        )

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

    def raw_proximity_values(self):
        return [sensor.getValue() for sensor in self.proximity_sensors]

    def proximity_values(self):
        return self.raw_proximity_values()

    def calibrated_proximity_values(self):
        raw_values = self.raw_proximity_values()

        return [
            max(
                0.0,
                raw_values[index]
                - self.proximity_baseline[index]
                - PROXIMITY_BASELINE_MARGIN,
            )
            for index in range(8)
        ]

    def front_obstacle_levels(self):
        values = self.proximity_values()

        left_front = max(values[5], values[6], values[7])
        right_front = max(values[0], values[1], values[2])

        return left_front, right_front

    def obstacle_level(self):
        return max(self.proximity_values())

    def get_lidar_ranges(self):
        if self.lidar is None:
            return None

        ranges = self.lidar.getRangeImage()

        if ranges is None or len(ranges) == 0:
            return None

        return ranges

    def get_lidar_fov(self):
        if self.lidar is None:
            return DEFAULT_LIDAR_FOV

        try:
            fov = self.lidar.getFov()
        except Exception:
            fov = DEFAULT_LIDAR_FOV

        if not self.is_valid_number(fov) or fov <= 0:
            fov = DEFAULT_LIDAR_FOV

        return fov

    def smooth_lidar_sector(self, sector, value):
        if value is None:
            return None

        previous = self.lidar_sector_filtered.get(sector)

        if previous is None:
            filtered = value
        else:
            filtered = (
                LIDAR_SMOOTHING_ALPHA * value
                + (1.0 - LIDAR_SMOOTHING_ALPHA) * previous
            )

        self.lidar_sector_filtered[sector] = filtered
        return filtered

    def lidar_sector_distances(self, start_angle_deg, end_angle_deg):
        ranges = self.get_lidar_ranges()

        if ranges is None:
            return None, None

        n = len(ranges)
        fov = self.get_lidar_fov()

        start_rad = math.radians(start_angle_deg)
        end_rad = math.radians(end_angle_deg)

        if start_rad > end_rad:
            start_rad, end_rad = end_rad, start_rad

        sector_distances = []

        for index, distance in enumerate(ranges):
            if not self.is_valid_number(distance):
                continue

            if distance <= LIDAR_NAV_MIN_VALID_DISTANCE:
                continue

            rel_angle = -fov / 2.0 + index * fov / max(1, n - 1)

            if start_rad <= rel_angle <= end_rad:
                sector_distances.append(distance)

        if not sector_distances:
            return None, None

        min_distance = min(sector_distances)
        robust_distance = self.percentile(
            sector_distances,
            LIDAR_SECTOR_PERCENTILE,
        )

        return min_distance, robust_distance

    def lidar_navigation_info(self):
        if self.lidar is None:
            return None

        front_min, front_robust = self.lidar_sector_distances(
            -LIDAR_FRONT_DEGREES,
            LIDAR_FRONT_DEGREES,
        )

        left_min, left_robust = self.lidar_sector_distances(
            -LIDAR_SIDE_DEGREES,
            -LIDAR_FRONT_DEGREES,
        )

        right_min, right_robust = self.lidar_sector_distances(
            LIDAR_FRONT_DEGREES,
            LIDAR_SIDE_DEGREES,
        )

        return {
            "front": self.smooth_lidar_sector("front", front_robust),
            "left": self.smooth_lidar_sector("left", left_robust),
            "right": self.smooth_lidar_sector("right", right_robust),
            "front_min": front_min,
            "left_min": left_min,
            "right_min": right_min,
        }

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def get_raw_compass_heading(self):
        if self.compass is None:
            return None

        values = self.compass.getValues()

        if values is None or len(values) < 2:
            return None

        if (
            math.isnan(values[0])
            or math.isnan(values[1])
            or math.isinf(values[0])
            or math.isinf(values[1])
        ):
            return None

        return math.atan2(values[0], values[1])

    def get_robot_heading(self):
        if self.compass is None:
            return self.estimated_theta

        raw_heading = self.get_raw_compass_heading()

        if raw_heading is None:
            return self.estimated_theta

        if self.initial_compass_heading is None:
            self.initial_compass_heading = raw_heading

        return self.normalize_angle(raw_heading - self.initial_compass_heading)

    def update_odometry(self):
        if self.left_encoder is None or self.right_encoder is None:
            now = self.robot.getTime()

            if now - self.last_encoder_warning_time >= 5.0:
                print("[restaurant_epuck] wheel encoders unavailable; keeping last estimated pose")
                self.last_encoder_warning_time = now

            if self.compass is not None:
                self.estimated_theta = self.get_robot_heading()

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
        delta_theta_wheels = (delta_right - delta_left) / AXLE_LENGTH

        if self.compass is not None:
            self.estimated_theta = self.get_robot_heading()
        else:
            self.estimated_theta += delta_theta_wheels

        left_front, right_front = self.front_obstacle_levels()
        front_blocked = max(left_front, right_front) > ODOMETRY_BLOCK_THRESHOLD
        any_obstacle_blocked = self.obstacle_level() > ODOMETRY_BLOCK_THRESHOLD

        is_forward_motion = delta_center > 0.0001

        if (front_blocked or any_obstacle_blocked) and is_forward_motion:
            delta_center = 0.0

        self.estimated_x += delta_center * math.cos(self.estimated_theta)
        self.estimated_y += delta_center * math.sin(self.estimated_theta)

    def get_robot_position(self):
        if self.experiment_mode == EXP1_MODE:
            return self.get_ground_truth_position()

        if self.experiment_mode == EXP2_MODE:
            return self.get_estimated_position()

        raise ValueError(f"Unsupported experiment mode: {self.experiment_mode}")

    """def get_robot_position(self):
        if self.experiment_mode == EXP1_MODE:
            return self.get_ground_truth_position()

        if self.experiment_mode == EXP2_MODE:
            ground_truth = self.get_ground_truth_position()
            if ground_truth is not None:
                return ground_truth

            return self.get_estimated_position()

        raise ValueError(f"Unsupported experiment mode: {self.experiment_mode}")
    """

    def get_ground_truth_position(self):
        try:
            node = self.robot.getSelf()

            if node is None:
                return None

            position = node.getPosition()

            return position[0], position[1]

        except Exception as exc:
            now = self.robot.getTime()

            if now - self.last_encoder_warning_time >= 5.0:
                print(f"[restaurant_epuck] ground truth unavailable: {exc}")
                self.last_encoder_warning_time = now

            return None

    def get_estimated_position(self):
        return self.estimated_x, self.estimated_y

    def reset_estimated_pose_to_base(self):
        self.estimated_x = self.base_pos[0]
        self.estimated_y = self.base_pos[1]
        self.estimated_theta = self.get_robot_heading()
        self.previous_left_encoder = None
        self.previous_right_encoder = None
        print("[restaurant_epuck] EXP2 odometry reset at base")

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

    def reset_navigation_progress(self):
        self.best_navigation_distance = None
        self.last_navigation_progress_time = None

    def current_navigation_distance(self):
        if self.state == STATE_GOING_TO_TABLE and self.target_id is not None:
            return self.distance_to_table_area(self.target_id)

        if self.state == STATE_RETURNING_TO_BASE:
            return self.distance_to_point(self.base_pos)

        return None

    def invalidate_navigation_plan(self):
        if self.navigation_exp1 is not None:
            self.navigation_exp1.last_planned_path = []
            self.navigation_exp1.cached_target_pos = None
            self.navigation_exp1.current_waypoint_index = 1

        if self.navigation_exp2 is not None:
            self.navigation_exp2.last_planned_path = []
            self.navigation_exp2.cached_target_pos = None
            self.navigation_exp2.last_selected_target_pos = None
            self.navigation_exp2.last_path_plan_time = -999.0

    def detect_navigation_stuck(self):
        if self.state not in (STATE_GOING_TO_TABLE, STATE_RETURNING_TO_BASE):
            self.reset_navigation_progress()
            return False

        if self.robot.getTime() < self.obstacle_recovery_until:
            return False

        distance = self.current_navigation_distance()
        if distance is None:
            self.reset_navigation_progress()
            return False

        now = self.robot.getTime()

        if self.best_navigation_distance is None:
            self.best_navigation_distance = distance
            self.last_navigation_progress_time = now
            return False

        if distance < self.best_navigation_distance - STUCK_MIN_PROGRESS:
            self.best_navigation_distance = distance
            self.last_navigation_progress_time = now
            return False

        if self.last_navigation_progress_time is None:
            self.last_navigation_progress_time = now
            return False

        if now - self.last_navigation_progress_time < STUCK_TIMEOUT:
            return False

        print(
            "[restaurant_epuck] navigation stuck detected "
            f"state={self.state} "
            f"target={self.target_id} "
            f"distance={distance:.3f} "
            f"best={self.best_navigation_distance:.3f}"
        )
        self.best_navigation_distance = distance
        self.last_navigation_progress_time = now
        return True

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
    
    def record_delivery_time(self, table_id, arrived_at):
        
        if self.delivery_start_time is None:
            print(f"[METRIC delivery_time] table={table_id} unavailable")
            return

        delivery_time = max(0.0, arrived_at - self.delivery_start_time)
        self.delivery_times.append(delivery_time)

        avg_delivery = sum(self.delivery_times) / len(self.delivery_times)

        print(
            "[METRIC delivery_time] "
            f"run={self.run_id} "
            f"policy={self.request_policy} "
            f"table={table_id} "
            f"delivery={delivery_time:.2f}s "
            f"avg={avg_delivery:.2f}s "
            f"count={len(self.delivery_times)}"
        )

    def notify_request_done(self, table_id):
        if table_id is None:
            return

        msg = f"DONE {table_id}"

        if self.emitter is None:
            print(f"[restaurant_epuck] cannot send {msg}: emitter unavailable")
            return

        self.emitter.send(msg.encode("utf-8"))
        print(f"[restaurant_epuck] sent: {msg}")

    def start_request(self, table_id, requested_at=None):
        if table_id not in self.TABLES:
            print(f"[restaurant_epuck] cannot start unknown table: {table_id}")
            return False

        if requested_at is None:
            requested_at = self.request_created_times.get(table_id)

        if requested_at is None:
            requested_at = self.robot.getTime()

        self.target_id = table_id
        self.current_request_created_time = requested_at

        self.mission_start_time =self.robot.getTime()
        self.delivery_start_time = self.mission_start_time

        #resetar distância total por missão
        self.total_distance = 0.0
        self.prev_pos = self.get_robot_position()

        #self.target_pos = self.TABLES[table_id]
        self.target_candidates = self.TABLE_REACH_POINTS[table_id]
        self.target_pos = self.target_candidates[0]
        
        self.reset_navigation_progress()
        print("[REQUEST DEBUG] table_id:", table_id)
        print("[DEBUG REQUEST] TABLE_REACH_POINTS:", self.target_candidates)
        print("[REQUEST DEBUG] target_pos chosen:", self.target_pos)
        
        self.state = STATE_GOING_TO_TABLE
        self.total_requests += 1

        print(
            f"[restaurant_epuck] NEW REQUEST: {table_id} -> "
            f"table_center={self.target_pos} requested_at={requested_at:.2f}"
        )

        return True

    def distance_to_request(self, table_id):
        current_pos = self.get_robot_position()
        if current_pos is None:
            current_pos = self.base_pos

        candidates = self.TABLE_REACH_POINTS.get(table_id)
        if not candidates:
            candidates = [self.TABLES[table_id]]

        current_x, current_y = current_pos

        return min(
            math.hypot(point_x - current_x, point_y - current_y)
            for point_x, point_y in candidates
        )

    def wait_for_request(self, table_id, now):
        requested_at = self.request_created_times.get(table_id)
        if requested_at is None:
            return 0.0

        return max(0.0, now - requested_at)

    def select_next_request(self):
        if not self.request_queue:
            return None

        now = self.robot.getTime()

        if self.request_policy == POLICY_FIFO:
            selected_index = 0
        elif self.request_policy == POLICY_NEAREST:
            selected_index = min(
                range(len(self.request_queue)),
                key=lambda index: (
                    self.distance_to_request(self.request_queue[index]),
                    index,
                ),
            )
        else:
            selected_index = max(
                range(len(self.request_queue)),
                key=lambda index: (
                    self.hybrid_wait_weight
                    * self.wait_for_request(self.request_queue[index], now)
                    - self.hybrid_distance_weight
                    * self.distance_to_request(self.request_queue[index]),
                    self.wait_for_request(self.request_queue[index], now),
                    -index,
                ),
            )

        table_id = self.request_queue.pop(selected_index)
        distance = self.distance_to_request(table_id)
        wait_time = self.wait_for_request(table_id, now)
        score = (
            self.hybrid_wait_weight * wait_time
            - self.hybrid_distance_weight * distance
        )

        print(
            "[restaurant_epuck] selected next request "
            f"policy={self.request_policy} "
            f"table={table_id} "
            f"wait={wait_time:.2f}s "
            f"distance={distance:.3f} "
            f"score={score:.2f} "
            f"remaining_queue={self.request_queue}"
        )

        return table_id

    def record_wait_time(self, table_id, served_at):
        requested_at = self.current_request_created_time

        if requested_at is None:
            requested_at = self.request_created_times.get(table_id)

        if requested_at is None:
            print(f"[METRIC wait_time] table={table_id} unavailable")
            return

        wait_time = max(0.0, served_at - requested_at)
        self.wait_times.append(wait_time)

        average_wait = sum(self.wait_times) / len(self.wait_times)

        print(
            "[METRIC wait_time] "
            f"run={self.run_id} "
            f"policy={self.request_policy} "
            f"table={table_id} "
            f"requested_at={requested_at:.2f} "
            f"served_at={served_at:.2f} "
            f"wait={wait_time:.2f}s "
            f"count={len(self.wait_times)} "
            f"avg={average_wait:.2f}s "
            f"max={max(self.wait_times):.2f}s"
        )

    def record_return_time(self, table_id, returned_at):
        if self.return_start_time is None:
            print(f"[METRIC return_time] table={table_id} unavailable")
            return

        return_time = max(0.0, returned_at - self.return_start_time)
        self.return_times.append(return_time)

        average_return = sum(self.return_times) / len(self.return_times)

        print(
            "[METRIC return_time] "
            f"run={self.run_id} "
            f"policy={self.request_policy} "
            f"table={table_id} "
            f"done_at={self.return_start_time:.2f} "
            f"base_at={returned_at:.2f} "
            f"return={return_time:.2f}s "
            f"count={len(self.return_times)} "
            f"avg={average_return:.2f}s "
            f"max={max(self.return_times):.2f}s"
        )

    def navigation_step(self, target_pos):
        """Fallback reativo quando não há caminho A* utilizável."""
        left_front, right_front = self.front_obstacle_levels()
        ps_max_front = max(left_front, right_front)

        lidar_info = self.lidar_navigation_info()

        if ps_max_front > NAV_OBSTACLE_THRESHOLD:
            if left_front >= right_front:
                left_speed = TURN_SPEED
                right_speed = -TURN_SPEED
            else:
                left_speed = -TURN_SPEED
                right_speed = TURN_SPEED

            self.set_status_leds(True)
            self.set_wheel_speeds(left_speed, right_speed)
            return

        if lidar_info is not None:
            front = lidar_info["front"]
            left = lidar_info["left"]
            right = lidar_info["right"]

            front_blocked = front is not None and front < LIDAR_CLEAR_DISTANCE
            front_caution = front is not None and front < LIDAR_CAUTION_DISTANCE

            if front_blocked:
                left_space = left if left is not None else 0.0
                right_space = right if right is not None else 0.0

                if left_space >= right_space:
                    left_speed = -TURN_SPEED
                    right_speed = TURN_SPEED
                else:
                    left_speed = TURN_SPEED
                    right_speed = -TURN_SPEED

                self.set_status_leds(True)
                self.set_wheel_speeds(left_speed, right_speed)
                return

            if front_caution:
                left_space = left if left is not None else 0.0
                right_space = right if right is not None else 0.0

                slow_speed = CRUISE_SPEED * 0.45
                steer = 0.8

                if left_space >= right_space:
                    left_speed = slow_speed - steer
                    right_speed = slow_speed + steer
                else:
                    left_speed = slow_speed + steer
                    right_speed = slow_speed - steer

                self.set_status_leds(True)
                self.set_wheel_speeds(left_speed, right_speed)
                return

        if ps_max_front > NAV_WARNING_THRESHOLD:
            slow_speed = CRUISE_SPEED * 0.55
            steer = 0.7

            if left_front >= right_front:
                left_speed = slow_speed + steer
                right_speed = slow_speed - steer
            else:
                left_speed = slow_speed - steer
                right_speed = slow_speed + steer

            self.set_status_leds(True)
            self.set_wheel_speeds(left_speed, right_speed)
            return

        self.set_status_leds(False)
        self.set_wheel_speeds(CRUISE_SPEED, CRUISE_SPEED)

    def start_obstacle_recovery(
        self,
        lidar_info=None,
        total_time=RECOVERY_TOTAL_TIME,
        reason="obstacle",
    ):
        now = self.robot.getTime()

        if now < self.obstacle_recovery_until:
            return
        
        if not self.in_safety_recovery:
            self.near_collision_count += 1
            self.in_safety_recovery = True

        left_space = 0.0
        right_space = 0.0

        if lidar_info is not None:
            left = lidar_info["left"]
            right = lidar_info["right"]
            left_space = left if left is not None else 0.0
            right_space = right if right is not None else 0.0

        if left_space >= right_space:
            self.obstacle_recovery_direction = 1
        else:
            self.obstacle_recovery_direction = -1

        self.obstacle_recovery_start_time = now
        self.obstacle_recovery_until = now + total_time
        self.reset_navigation_progress()

        if self.navigation_exp1 is not None:
            self.navigation_exp1.current_waypoint_index = min(
                self.navigation_exp1.current_waypoint_index + 1,
                max(1, len(self.navigation_exp1.last_planned_path) - 1),
            )

        self.invalidate_navigation_plan()

        print(
            "[restaurant_epuck] obstacle recovery started "
            f"reason={reason} "
            f"duration={total_time:.2f}s"
        )

    def run_obstacle_recovery(self):
        now = self.robot.getTime()

        if now >= self.obstacle_recovery_until:
            self.obstacle_recovery_start_time = None
            self.in_safety_recovery = False
            return False

        start_time = self.obstacle_recovery_start_time
        if start_time is None:
            start_time = now
            self.obstacle_recovery_start_time = now

        self.set_status_leds(True)

        if now - start_time < RECOVERY_BACKUP_TIME:
            self.set_wheel_speeds(RECOVERY_BACKUP_SPEED, RECOVERY_BACKUP_SPEED)
            return True

        turn = RECOVERY_TURN_SPEED * self.obstacle_recovery_direction
        self.set_wheel_speeds(-turn, turn)
        return True

    def path_obstacle_emergency(self):
        left_front, right_front = self.front_obstacle_levels()
        calibrated_proximity = self.calibrated_proximity_values()
        calibrated_left_front = max(
            calibrated_proximity[5],
            calibrated_proximity[6],
            calibrated_proximity[7],
        )
        calibrated_right_front = max(
            calibrated_proximity[0],
            calibrated_proximity[1],
            calibrated_proximity[2],
        )

        if max(left_front, right_front) > NAV_OBSTACLE_THRESHOLD:
            return True, self.lidar_navigation_info()

        lidar_info = self.lidar_navigation_info()
        if lidar_info is None:
            return False, None

        front = lidar_info["front"]
        front_min = lidar_info.get("front_min")

        if front is not None and front < PATH_EMERGENCY_LIDAR_DISTANCE:
            return True, lidar_info

        if front_min is not None and front_min < PATH_HARD_MIN_LIDAR_DISTANCE:
            return True, lidar_info

        return False, lidar_info


    def follow_path_step(self, path, fallback_target):
        if not path or len(path) < 2:
            self.navigation_step(fallback_target)
            return

        if self.navigation_exp2 is None:
            self.navigation_step(fallback_target)
            return

        waypoint_index = min(WAYPOINT_LOOKAHEAD_INDEX, len(path) - 1)
        waypoint_cell = path[waypoint_index]

        waypoint_pos = self.navigation_exp2.grid_to_world(
            waypoint_cell[0],
            waypoint_cell[1],
        )

        current_pos = self.get_robot_position()

        if current_pos is None or waypoint_pos is None:
            self.navigation_step(fallback_target)
            return

        robot_x, robot_y = current_pos
        waypoint_x, waypoint_y = waypoint_pos

        dx = waypoint_x - robot_x
        dy = waypoint_y - robot_y

        distance = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)
        heading = self.get_robot_heading()
        angle_error = self.normalize_angle(target_angle - heading)

        if self.run_obstacle_recovery():
            return

        emergency, lidar_info = self.path_obstacle_emergency()
        if emergency:
            self.start_obstacle_recovery(lidar_info)
            self.run_obstacle_recovery()
            return

        if distance < WAYPOINT_REACHED_RADIUS:
            self.set_wheel_speeds(PATH_FORWARD_SPEED, PATH_FORWARD_SPEED)
            return

        abs_error = abs(angle_error)
        turn = max(-PATH_TURN_SPEED, min(PATH_TURN_SPEED, angle_error * 1.4))

        if abs_error > ANGLE_TOLERANCE:
            forward_speed = PATH_MIN_FORWARD_SPEED

            if abs_error < 1.0:
                forward_speed = PATH_FORWARD_SPEED * 0.45

            self.set_status_leds(True)
        else:
            forward_speed = PATH_FORWARD_SPEED
            self.set_status_leds(False)

        left_speed = forward_speed - turn
        right_speed = forward_speed + turn

        self.set_wheel_speeds(left_speed, right_speed)


    def follow_path_exp1(self, path):
        if not path or len(path) < 2:
            self.stop()
            return

        if self.navigation_exp1 is None or self.known_map is None:
            self.stop()
            return

        fallback_target = self.target_pos
        if self.state == STATE_RETURNING_TO_BASE:
            fallback_target = self.base_pos

        if self.run_obstacle_recovery():
            return

        emergency, lidar_info = self.path_obstacle_emergency()
        if emergency:
            self.start_obstacle_recovery(lidar_info)
            self.run_obstacle_recovery()
            return

        # impede overflow
        if self.navigation_exp1.current_waypoint_index >= len(path):
            self.stop()
            return

        waypoint = path[self.navigation_exp1.current_waypoint_index]
        
        waypoint_pos = self.known_map.grid_to_world(*waypoint)
        robot_pos = self.get_robot_position()

        if robot_pos is None or waypoint_pos is None:
            self.stop()
            return

        dx = waypoint_pos[0] - robot_pos[0]
        dy = waypoint_pos[1] - robot_pos[1]

        dist = math.hypot(dx, dy)

        if dist < WAYPOINT_REACHED_RADIUS:
            self.navigation_exp1.current_waypoint_index += 1
            self.stop()
            return

        angle = math.atan2(dy, dx)
        heading = self.get_robot_heading()
        error = self.normalize_angle(angle - heading)

        if abs(error) > ANGLE_TOLERANCE:
            if error > 0:
                self.set_wheel_speeds(-1.5, 1.5)
            else:
                self.set_wheel_speeds(1.5, -1.5)
            return

        self.set_wheel_speeds(2.5, 2.5)

    def empty_navigation_result(self):
        return {
            "path": [],
            "path_length": 0,
        }

    def navigation_step_exp1(self):
        if self.navigation_exp1 is None:
            return self.empty_navigation_result()

        if self.state == STATE_GOING_TO_TABLE:
            return self.navigation_exp1.step(self.target_candidates)

        if self.state == STATE_RETURNING_TO_BASE:
            return self.navigation_exp1.step([self.base_pos])

        return self.empty_navigation_result()

    def navigation_step_exp2(self):
        if self.navigation_exp2 is None:
            return self.empty_navigation_result()

        if self.state == STATE_GOING_TO_TABLE:
            target_candidates = self.target_candidates[1:] or self.target_candidates
            return self.navigation_exp2.step_to_candidates(target_candidates)
        elif self.state == STATE_RETURNING_TO_BASE:
            target_pos = self.base_pos
        else:
            target_pos = None

        return self.navigation_exp2.step(target_pos)

    def get_navigation_result(self):
        if self.experiment_mode == EXP1_MODE:
            return self.navigation_step_exp1()

        if self.experiment_mode == EXP2_MODE:
            return self.navigation_step_exp2()

        raise ValueError(f"Unsupported experiment mode: {self.experiment_mode}")

    def save_navigation_debug_image(self, nav_result):
        if nav_result is None:
            return

        robot_pos = self.get_robot_position()
        robot_cell = None
        goal_cell = None

        if self.experiment_mode == EXP1_MODE:
            if self.known_map is None:
                return

            if robot_pos is not None:
                robot_cell = self.known_map.world_to_grid(*robot_pos)

            if self.target_pos is not None:
                goal_cell = self.known_map.world_to_grid(*self.target_pos)

            save_grid_png(
                self.known_map.grid,
                robot_pos=robot_cell,
                goal_pos=goal_cell,
                path=nav_result["path"],
            )
            return

        if self.experiment_mode == EXP2_MODE:
            if self.navigation_exp2 is None:
                return

            if robot_pos is not None:
                robot_cell = self.navigation_exp2.world_to_grid(*robot_pos)

            target_pos = nav_result.get("target_pos")
            if target_pos is not None:
                goal_cell = self.navigation_exp2.world_to_grid(*target_pos)

            save_grid_png(
                self.navigation_exp2.occupancy_grid,
                robot_pos=robot_cell,
                goal_pos=goal_cell,
                path=nav_result["path"],
            )


    def process_requests(self):
        if self.receiver is None:
            return

        while self.receiver.getQueueLength() > 0:
            msg = self.receiver.getString().strip()
            self.receiver.nextPacket()

            parts = msg.split()

            if len(parts) not in (2, 3) or parts[0] != "REQ":
                print(f"[restaurant_epuck] invalid message ignored: {msg}")
                continue

            table_id = parts[1]
            requested_at = None

            if len(parts) == 3:
                try:
                    requested_at = float(parts[2])
                except ValueError:
                    print(f"[restaurant_epuck] invalid request timestamp ignored: {msg}")
                    requested_at = None

            if table_id not in self.TABLES:
                print(f"[restaurant_epuck] unknown table id ignored: {table_id}")
                continue

            if requested_at is None:
                requested_at = self.robot.getTime()

            self.request_created_times[table_id] = requested_at

            if self.state == STATE_IDLE:
                self.start_request(table_id, requested_at)
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
                f"requested_at={requested_at:.2f} "
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
        calibrated_proximity = self.calibrated_proximity_values()
        calibrated_left_front = max(
            calibrated_proximity[5],
            calibrated_proximity[6],
            calibrated_proximity[7],
        )
        calibrated_right_front = max(
            calibrated_proximity[0],
            calibrated_proximity[1],
            calibrated_proximity[2],
        )

        heading = self.get_robot_heading()
        target_angle = None
        angle_error = None

        if self.target_pos is not None:
            current_pos = self.get_robot_position()
            if current_pos is not None:
                dx = self.target_pos[0] - current_pos[0]
                dy = self.target_pos[1] - current_pos[1]
                target_angle = math.atan2(dy, dx)
                angle_error = self.normalize_angle(target_angle - heading)

        target_angle_text = f"{target_angle:.2f}" if target_angle is not None else "None"
        angle_error_text = f"{angle_error:.2f}" if angle_error is not None else "None"

        ground_truth_pos = self.get_ground_truth_position()
        position_error = None

        if position is not None and ground_truth_pos is not None:
            position_error = math.hypot(
                ground_truth_pos[0] - position[0],
                ground_truth_pos[1] - position[1],
            )

        lidar_info = self.lidar_navigation_info()

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
            f"front=({left_front:.1f}, {right_front:.1f}) "
            f"front_cal=({calibrated_left_front:.1f}, {calibrated_right_front:.1f}) "
            f"obs={self.obstacle_level():.1f} "
            f"lidar={lidar_info} "
            f"heading={heading:.2f} "
            f"target_angle={target_angle_text} "
            f"angle_error={angle_error_text} "
            f"gt_pos={ground_truth_pos} "
            f"pos_error={position_error} "
        )

        self.last_report_time = now

    
    def run_state_machine(self):
        
        while self.robot.step(self.time_step) != -1:
            self.process_requests()
            self.update_odometry()

            pos = self.get_robot_position()

            if pos is not None and self.prev_pos is not None:
                self.total_distance += math.hypot(
                    pos[0] - self.prev_pos[0],
                    pos[1] - self.prev_pos[1]
                )

            if pos is not None:
                self.prev_pos = pos

            nav_result = self.get_navigation_result()

            has_path = (
                nav_result is not None
                and nav_result.get("path_length", 0) > 1
            )

            if not has_path:
                self.reset_navigation_progress()

            elif self.detect_navigation_stuck():
                self.start_obstacle_recovery(
                    self.lidar_navigation_info(),
                    total_time=STUCK_RECOVERY_TOTAL_TIME,
                    reason="stuck",
                )

            now = self.robot.getTime()

            if self.state == STATE_IDLE:
                self.stop()
                self.set_status_leds(False)

            elif self.state == STATE_GOING_TO_TABLE:

                if self.has_reached_table_area(self.target_id):
                    served_table = self.target_id
                    self.stop()
                    self.service_start_time = now
                    self.state = STATE_SERVING

                    self.record_wait_time(served_table, now)
                    self.record_delivery_time(served_table, now)

                    print(
                        f"[restaurant_epuck] arrived at service area for "
                        f"{served_table}; serving"
                    )
                    continue

                if self.experiment_mode == EXP1_MODE:

                    if nav_result["path_length"] > 1:
                        self.follow_path_exp1(nav_result["path"])
                    else:
                        self.stop()

                else:
                    fallback_target = nav_result.get("target_pos") or self.target_pos

                    if nav_result["path_length"] > 1:
                        self.follow_path_step(nav_result["path"], fallback_target)
                    else:
                        self.navigation_step(fallback_target)

                if self.has_reached_table_area(self.target_id):
                    served_table = self.target_id
                    self.stop()
                    self.service_start_time = now
                    self.state = STATE_SERVING

                    self.record_wait_time(served_table, now)
                    self.record_delivery_time(served_table, now)

                    print(
                        f"[restaurant_epuck] arrived at service area for "
                        f"{served_table}; serving"
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
                    self.return_start_time = now

                    self.last_served_id = completed_table
                    self.target_id = None
                    self.target_pos = self.base_pos
                    self.current_request_created_time = None
                    self.request_created_times.pop(completed_table, None)
                    self.reset_navigation_progress()
                    self.state = STATE_RETURNING_TO_BASE

                    print(f"[restaurant_epuck] returning to base from {completed_table}")

            elif self.state == STATE_RETURNING_TO_BASE:
                
                if self.experiment_mode == EXP1_MODE:
                    if nav_result["path_length"] > 1:
                        self.follow_path_exp1(nav_result["path"])
                    else:
                        self.stop()

                else:
                    if nav_result["path_length"] > 1:
                        self.follow_path_step(nav_result["path"], self.base_pos)
                    else:
                        self.navigation_step(self.base_pos)

                if self.has_arrived(self.base_pos, self.base_arrival_radius):
                    self.completed_requests += 1
                    self.stop()

                    print("[restaurant_epuck] arrived at base; ready for new requests")
                    
                    self.record_return_time(self.last_served_id, now)
                    success_rate = 0.0
                    if self.total_requests > 0:
                        success_rate = self.completed_requests / self.total_requests

                    print(
                        "[METRIC success_rate] "
                        f"run={self.run_id} "
                        f"policy={self.request_policy} "
                        f"completed={self.completed_requests} "
                        f"total={self.total_requests} "
                        f"success_rate={success_rate:.2f}"
                    )
                                        
                    if self.mission_start_time is not None:
                        mission_time = now - self.mission_start_time

                        print(
                            "[METRIC mission_time] "
                            f"run={self.run_id} "
                            f"policy={self.request_policy} "
                            f"table={self.last_served_id} "
                            f"mission={mission_time:.2f}s"
                        )

                        print(
                            "[METRIC total_distance] "
                            f"run={self.run_id} "
                            f"policy={self.request_policy} "
                            f"table={self.last_served_id} "
                            f"distance={self.total_distance:.3f}"
                        )

                        print(
                            "[METRIC safety_interventions] "
                            f"run={self.run_id} "
                            f"policy={self.request_policy} "
                            f"count={self.near_collision_count}"
                        )

                        avg_speed = 0.0
                        if mission_time > 0:
                            avg_speed = self.total_distance / mission_time


                        print(
                            "[METRIC avg_speed] "
                            f"run={self.run_id} "
                            f"policy={self.request_policy} "
                            f"table={self.last_served_id} "
                            f"speed={avg_speed:.3f}m/s"
                        )
                                            
                    self.mission_start_time = None
                    self.delivery_start_time = None

                    if self.experiment_mode == EXP2_MODE:
                        self.reset_estimated_pose_to_base()

                    self.target_id = None
                    self.target_pos = None
                    self.service_start_time = None
                    self.reset_navigation_progress()

                    if self.request_queue:
                        next_table = self.select_next_request()
                        self.start_request(
                            next_table,
                            self.request_created_times.get(next_table),
                        )

                    else:
                        self.state = STATE_IDLE
                        self.stop()

            else:
                print(f"[restaurant_epuck] unknown state {self.state}; stopping")
                self.stop()
                self.state = STATE_IDLE

            self.report_debug()

            
            if self.robot.getTime() % 2 < 0.03:

                self.save_navigation_debug_image(nav_result)


if __name__ == "__main__":
    RestaurantEpuck().run_state_machine()
