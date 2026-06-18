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

PATH_EMERGENCY_LIDAR_DISTANCE = 0.11
PATH_MIN_FORWARD_SPEED = 0.35

WAYPOINT_LOOKAHEAD_INDEX = 2
WAYPOINT_REACHED_RADIUS = 0.04

ANGLE_TOLERANCE = 0.25
PATH_FORWARD_SPEED = 1.8
PATH_TURN_SPEED = 1.2

# Durante path following, só abandona o caminho se houver emergência real.
PATH_HARD_MIN_LIDAR_DISTANCE = 0.07
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

        self.in_safety_recovery = False

        self.table_arrival_radius = 0.15
        self.base_arrival_radius = 0.05  # 0.02

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

    def navigation_step(self):
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

        self.epuck.set_status_leds(False)
        self.epuck.set_wheel_speeds(CRUISE_SPEED, CRUISE_SPEED)

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
            return self.navigation_step_exp1()

        if self.epuck.experiment_mode == EXP2_MODE:
            return self.navigation_step_exp2()

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

        return False, lidar_info

    def current_navigation_distance(self):
        if self.epuck.state == STATE_GOING_TO_TABLE and self.epuck.target_id is not None:
            return self.epuck.distance_to_table_area(self.epuck.target_id)

        if self.epuck.state == STATE_RETURNING_TO_BASE:
            return self.epuck.distance_to_point(self.epuck.base_pos)

        return None

    def detect_navigation_stuck(self):
        if self.epuck.state not in (STATE_GOING_TO_TABLE, STATE_RETURNING_TO_BASE):
            self.reset_navigation_progress()
            return False

        if self.epuck.robot.getTime() < self.obstacle_recovery_until:
            return False

        distance = self.current_navigation_distance()
        if distance is None:
            self.reset_navigation_progress()
            return False

        now = self.epuck.robot.getTime()

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
            f"state={self.epuck.state} "
            f"target={self.epuck.target_id} "
            f"distance={distance:.3f} "
            f"best={self.best_navigation_distance:.3f}"
        )
        self.best_navigation_distance = distance
        self.last_navigation_progress_time = now
        return True

    def follow_path_step(self, path, fallback_target):
        if not path or len(path) < 2:
            self.navigation_step()
            return

        if self.navigation_exp2 is None:
            self.navigation_step()
            return

        waypoint_index = min(WAYPOINT_LOOKAHEAD_INDEX, len(path) - 1)
        waypoint_cell = path[waypoint_index]

        waypoint_pos = self.navigation_exp2.grid_to_world(
            waypoint_cell[0],
            waypoint_cell[1],
        )

        current_pos = self.epuck.get_robot_position()

        if current_pos is None or waypoint_pos is None:
            self.navigation_step()
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

        if distance < WAYPOINT_REACHED_RADIUS:
            self.epuck.set_wheel_speeds(PATH_FORWARD_SPEED, PATH_FORWARD_SPEED)
            return

        abs_error = abs(angle_error)
        turn = max(-PATH_TURN_SPEED, min(PATH_TURN_SPEED, angle_error * 1.4))

        if abs_error > ANGLE_TOLERANCE:
            forward_speed = PATH_MIN_FORWARD_SPEED

            if abs_error < 1.0:
                forward_speed = PATH_FORWARD_SPEED * 0.45

            self.epuck.set_status_leds(True)
        else:
            forward_speed = PATH_FORWARD_SPEED
            self.epuck.set_status_leds(False)

        left_speed = forward_speed - turn
        right_speed = forward_speed + turn

        self.epuck.set_wheel_speeds(left_speed, right_speed)

    def empty_navigation_result(self):
        return {
            "path": [],
            "path_length": 0,
        }

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

    def reset_navigation_progress(self):
        self.best_navigation_distance = None
        self.last_navigation_progress_time = None
