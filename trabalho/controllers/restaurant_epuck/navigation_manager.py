import math
from map_utils import save_grid_png

# from restaurant_epuck import STATE_GOING_TO_TABLE, STATE_IDLE, STATE_RETURNING_TO_BASE, STATE_SERVING

EXP1_MODE = "EXP1"
EXP2_MODE = "EXP2"

STATE_IDLE = "IDLE"
STATE_GOING_TO_TABLE = "GOING_TO_TABLE"
STATE_SERVING = "SERVING"
STATE_RETURNING_TO_BASE = "RETURNING_TO_BASE"

# Navegação reativa fallback.
NAV_WARNING_THRESHOLD = 78.0
NAV_OBSTACLE_THRESHOLD = 85.0

CRUISE_SPEED = 3.2
TURN_SPEED = 2.4

LIDAR_CLEAR_DISTANCE = 0.18
LIDAR_CAUTION_DISTANCE = 0.25
NO_PATH_SCAN_TURN_SPEED = 0.9

PATH_EMERGENCY_LIDAR_DISTANCE = 0.095
PATH_MIN_FORWARD_SPEED = 0.35

WAYPOINT_LOOKAHEAD_INDEX = 2
WAYPOINT_LOOKAHEAD_MAX_INDEX = 6
WAYPOINT_LOOKAHEAD_DISTANCE = 0.09
WAYPOINT_MIN_LOOKAHEAD_DISTANCE = 0.055
WAYPOINT_REACHED_RADIUS = 0.04

ANGLE_TOLERANCE = 0.25
PATH_FORWARD_SPEED = 1.8
PATH_TURN_SPEED = 1.2
PATH_SLOW_TURN_ANGLE = 0.75
DIRECT_TARGET_FORWARD_SPEED = 0.9
DIRECT_TARGET_TURN_SPEED = 1.0

# Durante path following, só abandona o caminho se houver emergência real.
PATH_HARD_MIN_LIDAR_DISTANCE = 0.055
PATH_SIDE_HARD_MIN_LIDAR_DISTANCE = 0.055
PATH_CLEARANCE_FRONT_MIN_DISTANCE = 0.085
PATH_CLEARANCE_SIDE_MIN_DISTANCE = 0.065
PATH_CLEARANCE_TURN_SPEED = 1.05
RECOVERY_BACKUP_TIME = 0.5
RECOVERY_BACKUP_SPEED = -1.2
RECOVERY_TURN_SPEED = 1.8
RECOVERY_TOTAL_TIME = 1

# Detecao simples de bloqueio: se o robo nao reduz a distancia ao alvo
# durante alguns segundos, assume que ficou preso junto a uma cadeira/mesa.
STUCK_TIMEOUT = 6.0
STUCK_MIN_PROGRESS = 0.01


class NavigationManager:

    def __init__(self, epuck):
        self.epuck = epuck

        self.known_map = None
        self.navigation_exp1 = None
        self.navigation_exp2 = None

        self.obstacle_recovery_start_time = None
        self.obstacle_recovery_until = 0.0
        self.obstacle_recovery_direction = 1

        self.best_navigation_distance = None
        self.last_navigation_progress_time = None
        self.stuck_progress_kind = None
        self.stuck_progress_token = None
        self.last_navigation_result = None

        self.in_safety_recovery = False

        self.table_arrival_radius = getattr(epuck, "table_arrival_radius", 0.15)
        self.base_arrival_radius = getattr(epuck, "base_arrival_radius", 0.05)

    def configure_experiment(self):
        if self.epuck.experiment_mode == EXP1_MODE:
            from known_map import KnownMap
            from navigation_exp1 import NavigationExp1

            # Compatível com as duas versões usadas no projeto:
            # KnownMap() e KnownMap(map_config).
            try:
                self.known_map = KnownMap(self.epuck.map_config)
            except TypeError:
                self.known_map = KnownMap()

            # O NavigationExp1 precisa do controlador/e-puck para obter posição e heading.
            self.navigation_exp1 = NavigationExp1(self.epuck, self.known_map)
            return

        if self.epuck.experiment_mode == EXP2_MODE:
            from navigation_exp2 import NavigationExp2

            # O NavigationExp2 também deve receber o e-puck/controlador, não o manager.
            self.navigation_exp2 = NavigationExp2(self.epuck)
            return

        raise ValueError(f"Unsupported experiment mode: {self.epuck.experiment_mode}")

    def navigation_step(self, target_pos=None):
        """Fallback reativo quando não há caminho A* utilizável."""
        left_front, right_front = self.epuck.front_obstacle_levels()
        ps_max_front = max(left_front, right_front)

        lidar_info = self.epuck.lidar_navigation_info()

        if ps_max_front > NAV_OBSTACLE_THRESHOLD:
            if left_front >= right_front:
                left_speed = TURN_SPEED
                right_speed = -TURN_SPEED
            else:
                left_speed = -TURN_SPEED
                right_speed = TURN_SPEED

            self.epuck.set_status_leds(True)
            self.epuck.set_wheel_speeds(left_speed, right_speed)
            return

        if self.clearance_guard_step(lidar_info):
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

                self.epuck.set_status_leds(True)
                self.epuck.set_wheel_speeds(left_speed, right_speed)
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

                self.epuck.set_status_leds(True)
                self.epuck.set_wheel_speeds(left_speed, right_speed)
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

            self.epuck.set_status_leds(True)
            self.epuck.set_wheel_speeds(left_speed, right_speed)
            return

        if target_pos is not None:
            current_pos = self.epuck.get_robot_position()

            if current_pos is not None:
                robot_x, robot_y = current_pos
                target_x, target_y = target_pos

                dx = target_x - robot_x
                dy = target_y - robot_y
                distance = math.hypot(dx, dy)

                if distance > WAYPOINT_REACHED_RADIUS:
                    target_angle = math.atan2(dy, dx)
                    heading = self.epuck.get_robot_heading()
                    angle_error = self.epuck.normalize_angle(target_angle - heading)
                    abs_error = abs(angle_error)

                    turn = max(
                        -DIRECT_TARGET_TURN_SPEED,
                        min(DIRECT_TARGET_TURN_SPEED, angle_error * 1.3),
                    )

                    if abs_error > 1.0:
                        forward_speed = 0.0
                    elif abs_error > ANGLE_TOLERANCE:
                        forward_speed = DIRECT_TARGET_FORWARD_SPEED * 0.35
                    else:
                        forward_speed = DIRECT_TARGET_FORWARD_SPEED

                    if distance < 0.12:
                        forward_speed = min(
                            forward_speed,
                            DIRECT_TARGET_FORWARD_SPEED * 0.45,
                        )

                    left_speed = forward_speed - turn
                    right_speed = forward_speed + turn

                    self.epuck.set_status_leds(abs_error > ANGLE_TOLERANCE)
                    self.epuck.set_wheel_speeds(left_speed, right_speed)
                    return

        if lidar_info is not None:
            left = lidar_info.get("left_min") or lidar_info.get("left")
            right = lidar_info.get("right_min") or lidar_info.get("right")
            left_space = left if left is not None else 0.0
            right_space = right if right is not None else 0.0
            turn = NO_PATH_SCAN_TURN_SPEED if left_space >= right_space else -NO_PATH_SCAN_TURN_SPEED
        else:
            turn = NO_PATH_SCAN_TURN_SPEED

        self.epuck.set_status_leds(True)
        self.epuck.set_wheel_speeds(-turn, turn)

    def follow_path_exp1(self, path):
        if not path or len(path) < 2:
            self.epuck.stop()
            return

        if self.navigation_exp1 is None or self.known_map is None:
            self.epuck.stop()
            return

        fallback_target = self.epuck.target_pos
        if self.epuck.state == STATE_RETURNING_TO_BASE:
            fallback_target = self.epuck.base_pos

        if self.run_obstacle_recovery():
            return

        emergency, lidar_info = self.path_obstacle_emergency()
        if emergency:
            self.start_obstacle_recovery(lidar_info)
            self.run_obstacle_recovery()
            return

        # impede overflow
        if self.navigation_exp1.current_waypoint_index >= len(path):
            self.epuck.stop()
            return

        waypoint = path[self.navigation_exp1.current_waypoint_index]
        """print(
            "WAYPOINT:",
            self.navigation_exp1.current_waypoint_index,
            "CELL:",
            waypoint
        )
        """
        waypoint_pos = self.known_map.grid_to_world(*waypoint)
        robot_pos = self.epuck.get_robot_position()

        if robot_pos is None or waypoint_pos is None:
            self.epuck.stop()
            return

        dx = waypoint_pos[0] - robot_pos[0]
        dy = waypoint_pos[1] - robot_pos[1]

        dist = math.hypot(dx, dy)

        if dist < WAYPOINT_REACHED_RADIUS:
            self.navigation_exp1.current_waypoint_index += 1
            self.epuck.stop()
            return

        angle = math.atan2(dy, dx)
        heading = self.epuck.get_robot_heading()
        error = self.epuck.normalize_angle(angle - heading)

        if abs(error) > ANGLE_TOLERANCE:
            if error > 0:
                self.epuck.set_wheel_speeds(-1.5, 1.5)
            else:
                self.epuck.set_wheel_speeds(1.5, -1.5)
            return

        self.epuck.set_wheel_speeds(2.5, 2.5)

    def navigation_step_exp1(self):
        if self.navigation_exp1 is None:
            return self.empty_navigation_result()

        if self.epuck.state == STATE_GOING_TO_TABLE:
            return self.navigation_exp1.step(self.epuck.target_candidates)

        if self.epuck.state == STATE_RETURNING_TO_BASE:
            return self.navigation_exp1.step([self.epuck.base_pos])

        return self.empty_navigation_result()

    def navigation_step_exp2(self):
        if self.navigation_exp2 is None:
            return self.empty_navigation_result()

        if self.epuck.state == STATE_GOING_TO_TABLE:
            return self.navigation_exp2.step_to_candidates(self.epuck.target_candidates)
        elif self.epuck.state == STATE_RETURNING_TO_BASE:
            target_pos = self.epuck.base_pos
        else:
            target_pos = None

        return self.navigation_exp2.step(target_pos)

    def get_navigation_result(self):
        if self.epuck.experiment_mode == EXP1_MODE:
            self.last_navigation_result = self.navigation_step_exp1()
            return self.last_navigation_result

        if self.epuck.experiment_mode == EXP2_MODE:
            self.last_navigation_result = self.navigation_step_exp2()
            return self.last_navigation_result

        raise ValueError(f"Unsupported experiment mode: {self.epuck.experiment_mode}")

    def save_navigation_debug_image(self, nav_result):
        if nav_result is None:
            return

        robot_pos = self.epuck.get_robot_position()
        robot_cell = None
        goal_cell = None

        if self.epuck.experiment_mode == EXP1_MODE:
            if self.known_map is None:
                return

            if robot_pos is not None:
                robot_cell = self.known_map.world_to_grid(*robot_pos)

            if self.epuck.target_pos is not None:
                goal_cell = self.known_map.world_to_grid(*self.epuck.target_pos)

            save_grid_png(
                self.known_map.grid,
                robot_pos=robot_cell,
                goal_pos=goal_cell,
                path=nav_result["path"],
            )
            return

        if self.epuck.experiment_mode == EXP2_MODE:
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

    def start_obstacle_recovery(
        self,
        lidar_info=None,
        total_time=RECOVERY_TOTAL_TIME,
        reason="obstacle",
    ):
        now = self.epuck.robot.getTime()

        if now < self.obstacle_recovery_until:
            return

        if not self.in_safety_recovery:
            self.epuck.metrics.near_collision_count += 1
            self.in_safety_recovery = True

        left_space = 0.0
        right_space = 0.0

        if lidar_info is not None:
            left = lidar_info.get("left_min") or lidar_info["left"]
            right = lidar_info.get("right_min") or lidar_info["right"]
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

        self.invalidate_navigation_plan(keep_exp2_path=(reason != "stuck"))

        print(
            "[restaurant_epuck] obstacle recovery started "
            f"reason={reason} "
            f"duration={total_time:.2f}s"
        )

    def run_obstacle_recovery(self):
        now = self.epuck.robot.getTime()

        if now >= self.obstacle_recovery_until:
            self.obstacle_recovery_start_time = None
            self.in_safety_recovery = False
            return False

        start_time = self.obstacle_recovery_start_time
        if start_time is None:
            start_time = now
            self.obstacle_recovery_start_time = now

        self.epuck.set_status_leds(True)

        if now - start_time < RECOVERY_BACKUP_TIME:
            self.epuck.set_wheel_speeds(RECOVERY_BACKUP_SPEED, RECOVERY_BACKUP_SPEED)
            return True

        turn = RECOVERY_TURN_SPEED * self.obstacle_recovery_direction
        self.epuck.set_wheel_speeds(-turn, turn)
        return True

    def clearance_guard_step(self, lidar_info=None):
        if lidar_info is None:
            lidar_info = self.epuck.lidar_navigation_info()

        if lidar_info is None:
            return False

        front_min = lidar_info.get("front_min")
        left_min = lidar_info.get("left_min")
        right_min = lidar_info.get("right_min")

        close_front = (
            front_min is not None
            and front_min < PATH_CLEARANCE_FRONT_MIN_DISTANCE
        )

        side_distances = [
            distance
            for distance in (left_min, right_min)
            if distance is not None
        ]
        close_side = (
            bool(side_distances)
            and min(side_distances) < PATH_CLEARANCE_SIDE_MIN_DISTANCE
        )

        if not close_front and not close_side:
            return False

        left_space = left_min
        if left_space is None:
            left_space = lidar_info.get("left")
        if left_space is None:
            left_space = 0.0

        right_space = right_min
        if right_space is None:
            right_space = lidar_info.get("right")
        if right_space is None:
            right_space = 0.0

        turn = PATH_CLEARANCE_TURN_SPEED
        if right_space > left_space:
            turn = -PATH_CLEARANCE_TURN_SPEED

        self.epuck.set_status_leds(True)
        self.epuck.set_wheel_speeds(-turn, turn)
        return True

    def path_obstacle_emergency(self):
        left_front, right_front = self.epuck.front_obstacle_levels()
        calibrated_proximity = self.epuck.calibrated_proximity_values()

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
            print(
                f"[EMERGENCY] ps={max(left_front, right_front):.1f}"
            )
            return True, self.epuck.lidar_navigation_info()

        lidar_info = self.epuck.lidar_navigation_info()
        if lidar_info is None:
            return False, None

        front = lidar_info["front"]
        front_min = lidar_info.get("front_min")
        left_min = lidar_info.get("left_min")
        right_min = lidar_info.get("right_min")

        if front is not None and front < PATH_EMERGENCY_LIDAR_DISTANCE:
            print(
                f"[EMERGENCY] front={front:.3f} "
                f"threshold={PATH_EMERGENCY_LIDAR_DISTANCE}"
            )
            return True, lidar_info

        if front_min is not None and front_min < PATH_HARD_MIN_LIDAR_DISTANCE:
            print(
                f"[EMERGENCY] front_min={front_min:.3f} "
                f"threshold={PATH_HARD_MIN_LIDAR_DISTANCE}"
            )
            return True, lidar_info

        side_distances = [
            distance
            for distance in (left_min, right_min)
            if distance is not None
        ]

        if side_distances and min(side_distances) < PATH_SIDE_HARD_MIN_LIDAR_DISTANCE:
            print(
                f"[EMERGENCY] side_min={min(side_distances):.3f} "
                f"threshold={PATH_SIDE_HARD_MIN_LIDAR_DISTANCE}"
            )
            return True, lidar_info

        return False, lidar_info

    def current_navigation_distance(self):
        if self.epuck.experiment_mode == EXP2_MODE:
            if self.epuck.state == STATE_GOING_TO_TABLE and self.epuck.target_id is not None:
                return (
                    self.epuck.distance_to_table_area(self.epuck.target_id),
                    "target",
                    self.epuck.target_id,
                )

            if self.epuck.state == STATE_RETURNING_TO_BASE:
                return self.epuck.distance_to_point(self.epuck.base_pos), "base", "base"

            return None, None, None

        path = self.current_navigation_path()
        path_distance = self.current_path_remaining_distance(path)
        if path_distance is not None:
            return path_distance, "path", self.current_path_progress_token(path)

        if self.epuck.state == STATE_GOING_TO_TABLE and self.epuck.target_id is not None:
            return self.epuck.distance_to_table_area(self.epuck.target_id), "target", self.epuck.target_id

        if self.epuck.state == STATE_RETURNING_TO_BASE:
            return self.epuck.distance_to_point(self.epuck.base_pos), "base", "base"

        return None, None, None

    def current_navigation_path(self):
        nav_result = self.last_navigation_result

        if not nav_result:
            return None

        path = nav_result.get("path")
        if not path or len(path) < 2:
            return None

        return path

    def current_path_progress_token(self, path):
        if not path:
            return None

        return path[-1], len(path)

    def current_path_remaining_distance(self, path=None):
        if path is None:
            path = self.current_navigation_path()

        if not path:
            return None

        current_pos = self.epuck.get_robot_position()
        if current_pos is None:
            return None

        grid_mapper = self.navigation_exp2 or self.known_map
        if grid_mapper is None:
            return None

        path_points = [
            grid_mapper.grid_to_world(row, col)
            for row, col in path[1:]
        ]
        path_points = [point for point in path_points if point is not None]

        if not path_points:
            return None

        remaining = math.hypot(
            path_points[0][0] - current_pos[0],
            path_points[0][1] - current_pos[1],
        )

        previous = path_points[0]
        for point in path_points[1:]:
            remaining += math.hypot(point[0] - previous[0], point[1] - previous[1])
            previous = point

        return remaining

    def detect_navigation_stuck(self):
        if self.epuck.state not in (STATE_GOING_TO_TABLE, STATE_RETURNING_TO_BASE):
            self.reset_navigation_progress()
            return False

        if self.epuck.robot.getTime() < self.obstacle_recovery_until:
            return False

        distance, progress_kind, progress_token = self.current_navigation_distance()
        if distance is None:
            self.reset_navigation_progress()
            return False

        now = self.epuck.robot.getTime()

        if (
            self.best_navigation_distance is None
            or self.stuck_progress_kind != progress_kind
            or self.stuck_progress_token != progress_token
        ):
            self.best_navigation_distance = distance
            self.last_navigation_progress_time = now
            self.stuck_progress_kind = progress_kind
            self.stuck_progress_token = progress_token
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
            f"state={self.epuck.state} "
            f"target={self.epuck.target_id} "
            f"progress={progress_kind} "
            f"distance={distance:.3f} "
            f"best={self.best_navigation_distance:.3f}"
        )
        self.best_navigation_distance = distance
        self.last_navigation_progress_time = now
        return True

    def follow_path_step(self, path, fallback_target):
        if not path or len(path) < 2:
            self.navigation_step(fallback_target)
            return

        if self.navigation_exp2 is None:
            self.navigation_step(fallback_target)
            return

        current_pos = self.epuck.get_robot_position()

        if current_pos is None:
            self.navigation_step(fallback_target)
            return

        waypoint_pos = self.select_path_waypoint(path, current_pos)

        if waypoint_pos is None:
            self.navigation_step(fallback_target)
            return

        robot_x, robot_y = current_pos
        waypoint_x, waypoint_y = waypoint_pos

        dx = waypoint_x - robot_x
        dy = waypoint_y - robot_y

        distance = math.hypot(dx, dy)

        target_angle = math.atan2(dy, dx)
        heading = self.epuck.get_robot_heading()
        angle_error = self.epuck.normalize_angle(target_angle - heading)

        if self.run_obstacle_recovery():
            return

        emergency, lidar_info = self.path_obstacle_emergency()
        if emergency:
            self.start_obstacle_recovery(lidar_info)
            self.run_obstacle_recovery()
            return

        if self.clearance_guard_step(lidar_info):
            return

        if distance < WAYPOINT_REACHED_RADIUS:
            self.epuck.set_wheel_speeds(PATH_FORWARD_SPEED, PATH_FORWARD_SPEED)
            return

        abs_error = abs(angle_error)
        turn = max(-PATH_TURN_SPEED, min(PATH_TURN_SPEED, angle_error * 1.4))

        if abs_error > ANGLE_TOLERANCE:
            forward_speed = 0.0

            if abs_error < PATH_SLOW_TURN_ANGLE:
                forward_speed = PATH_FORWARD_SPEED * 0.35

            self.epuck.set_status_leds(True)
        else:
            forward_speed = PATH_FORWARD_SPEED
            self.epuck.set_status_leds(False)

        left_speed = forward_speed - turn
        right_speed = forward_speed + turn

        self.epuck.set_wheel_speeds(left_speed, right_speed)

    def select_path_waypoint(self, path, current_pos):
        if self.navigation_exp2 is None:
            return None

        robot_x, robot_y = current_pos
        max_index = min(WAYPOINT_LOOKAHEAD_MAX_INDEX, len(path) - 1)
        previous_pos = current_pos
        selected_pos = None
        travelled = 0.0

        for index in range(1, max_index + 1):
            waypoint_cell = path[index]
            waypoint_pos = self.navigation_exp2.grid_to_world(
                waypoint_cell[0],
                waypoint_cell[1],
            )

            if waypoint_pos is None:
                continue

            segment = math.hypot(
                waypoint_pos[0] - previous_pos[0],
                waypoint_pos[1] - previous_pos[1],
            )
            travelled += segment
            previous_pos = waypoint_pos

            dx = waypoint_pos[0] - robot_x
            dy = waypoint_pos[1] - robot_y
            distance = math.hypot(dx, dy)

            if (
                distance < WAYPOINT_MIN_LOOKAHEAD_DISTANCE
                and index < max_index
            ):
                continue

            selected_pos = waypoint_pos

            if travelled >= WAYPOINT_LOOKAHEAD_DISTANCE:
                return selected_pos

        if selected_pos is not None:
            return selected_pos

        waypoint_index = min(WAYPOINT_LOOKAHEAD_INDEX, len(path) - 1)
        waypoint_cell = path[waypoint_index]
        return self.navigation_exp2.grid_to_world(
            waypoint_cell[0],
            waypoint_cell[1],
        )

    def follow_direct_target_step(self, target_pos, current_pos=None):
        if target_pos is None:
            self.navigation_step(target_pos)
            return

        if current_pos is None:
            current_pos = self.epuck.get_robot_position()

        if current_pos is None:
            self.navigation_step(target_pos)
            return

        if self.run_obstacle_recovery():
            return

        emergency, lidar_info = self.path_obstacle_emergency()
        if emergency:
            self.start_obstacle_recovery(lidar_info)
            self.run_obstacle_recovery()
            return

        if self.clearance_guard_step(lidar_info):
            return

        robot_x, robot_y = current_pos
        target_x, target_y = target_pos

        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.hypot(dx, dy)

        if distance < WAYPOINT_REACHED_RADIUS:
            self.epuck.stop()
            return

        target_angle = math.atan2(dy, dx)
        heading = self.epuck.get_robot_heading()
        angle_error = self.epuck.normalize_angle(target_angle - heading)

        turn = max(
            -DIRECT_TARGET_TURN_SPEED,
            min(DIRECT_TARGET_TURN_SPEED, angle_error * 1.3),
        )

        if abs(angle_error) > ANGLE_TOLERANCE:
            forward_speed = 0.0
            self.epuck.set_status_leds(True)
        else:
            forward_speed = DIRECT_TARGET_FORWARD_SPEED
            self.epuck.set_status_leds(False)

        left_speed = forward_speed - turn
        right_speed = forward_speed + turn

        self.epuck.set_wheel_speeds(left_speed, right_speed)

    def empty_navigation_result(self):
        return {
            "path": [],
            "path_length": 0,
        }

    def invalidate_navigation_plan(self, keep_exp2_path=False):
        if self.navigation_exp1 is not None:
            self.navigation_exp1.last_planned_path = []
            self.navigation_exp1.cached_target_pos = None
            self.navigation_exp1.current_waypoint_index = 1

        if self.navigation_exp2 is not None:
            if keep_exp2_path:
                self.navigation_exp2.last_path_plan_time = -999.0
            else:
                self.navigation_exp2.last_planned_path = []
                self.navigation_exp2.cached_target_pos = None
                self.navigation_exp2.last_selected_target_pos = None
                self.navigation_exp2.last_path_plan_time = -999.0

    def reset_navigation_progress(self):
        self.best_navigation_distance = None
        self.last_navigation_progress_time = None
        self.stuck_progress_kind = None
        self.stuck_progress_token = None
