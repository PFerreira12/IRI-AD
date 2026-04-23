"""Sensor and actuator configuration for the restaurant e-puck.

Week 2 objective: configure the robot sensors and actuators in Webots.
This controller centralizes the device setup and runs a small validation
behavior so the wiring can be checked immediately in the simulator.
"""
import random
from controller import Robot, Receiver


TIME_STEP = 32
MAX_SPEED = 6.28
CRUISE_SPEED = 3.2
TURN_SPEED = 2.4
OBSTACLE_THRESHOLD = 80.0
WHEEL_RADIUS = 0.0205


class RestaurantEpuck:
    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep()) or TIME_STEP

        self.left_motor = self._required_device("left wheel motor")
        self.right_motor = self._required_device("right wheel motor")
        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.set_wheel_speeds(0.0, 0.0)

        self.left_encoder = self._optional_sensor("left wheel sensor")
        self.right_encoder = self._optional_sensor("right wheel sensor")
        self.proximity_sensors = [
            self._required_sensor(f"ps{index}") for index in range(8)
        ]
        self.lidar = self._optional_sensor("lidar")
        self.camera = self._optional_sensor("camera", sampling_period=4 * self.time_step)
        self.accelerometer = self._optional_sensor("accelerometer")
        self.leds = [
            led
            for index in range(10)
            if (led := self._optional_device(f"led{index}")) is not None
        ]

        # Requests (Receiver)
        self.receiver = self._optional_device("receiver")
        if self.receiver is not None:
            self.receiver.enable(self.time_step)

        self.TABLES = {
            "T1": (-0.432, -0.312),
            "T2": (-0.168, -0.120),
            "T3": (-0.408, 0.204),
            "T4": (0.396, -0.252),
            "T5": (0.144, 0.084),
            "T6": (0.432, 0.336),
        }

        self.target_id = None
        self.target_pos = None

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
        optional_sensors = {
            "lidar": self.lidar is not None,
            "camera": self.camera is not None,
            "accelerometer": self.accelerometer is not None,
            "wheel_encoders": self.left_encoder is not None and self.right_encoder is not None,
        }
        enabled = ", ".join(name for name, available in optional_sensors.items() if available)
        missing = ", ".join(name for name, available in optional_sensors.items() if not available)
        print("[restaurant_epuck] Configured actuators: left/right wheel motors, LEDs")
        print("[restaurant_epuck] Configured sensors: ps0..ps7" + (f", {enabled}" if enabled else ""))
        if missing:
            print(f"[restaurant_epuck] Optional sensors not present in this robot: {missing}")

    def set_wheel_speeds(self, left_speed, right_speed):
        left_speed = max(-MAX_SPEED, min(MAX_SPEED, left_speed))
        right_speed = max(-MAX_SPEED, min(MAX_SPEED, right_speed))
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

    def set_status_leds(self, blocked):
        for index, led in enumerate(self.leds):
            led.set(1 if blocked or index % 2 == 0 else 0)

    def proximity_values(self):
        return [sensor.getValue() for sensor in self.proximity_sensors]

    def front_obstacle_levels(self):
        values = self.proximity_values()
        left_front = max(values[5], values[6], values[7])
        right_front = max(values[0], values[1], values[2])
        return left_front, right_front

    def wheel_odometry(self):
        if self.left_encoder is None or self.right_encoder is None:
            return None
        left_distance = self.left_encoder.getValue() * WHEEL_RADIUS
        right_distance = self.right_encoder.getValue() * WHEEL_RADIUS
        return left_distance, right_distance

    def process_requests(self):
        if self.receiver is None:
            return

        while self.receiver.getQueueLength() > 0:
            msg = self.receiver.getData().decode("utf-8").strip()
            self.receiver.nextPacket()

            parts = msg.split()
            if len(parts) == 2 and parts[0] == "REQ":
                tid = parts[1]
                if tid in self.TABLES:
                    self.target_id = tid
                    self.target_pos = self.TABLES[tid]
                    print(f"[restaurant_epuck] NEW REQUEST: {self.target_id} -> {self.target_pos}")
                else:
                    print(f"[restaurant_epuck] Unknown table id: {tid}")
            else:
                print(f"[restaurant_epuck] Unknown message: {msg}")

    def run_validation_behavior(self):
        last_report_time = -1.0

        while self.robot.step(self.time_step) != -1:

            self.process_requests()
                    
            left_front, right_front = self.front_obstacle_levels()
            blocked = max(left_front, right_front) > OBSTACLE_THRESHOLD

            if blocked and left_front >= right_front:
                left_speed, right_speed = TURN_SPEED, -TURN_SPEED
            elif blocked:
                left_speed, right_speed = -TURN_SPEED, TURN_SPEED
            else:
                # pequena aleatoriedade
                if random.random() < 0.02:
                    left_speed, right_speed = TURN_SPEED, -TURN_SPEED
                else:
                    left_speed, right_speed = CRUISE_SPEED, CRUISE_SPEED

            self.set_status_leds(blocked)
            self.set_wheel_speeds(left_speed, right_speed)

            now = self.robot.getTime()
            if now - last_report_time >= 1.0:
                
                if self.lidar:
                    ranges = self.lidar.getRangeImage()
                    print("[lidar] first values:", ranges[:5])
                
                last_report_time = now
                odometry = self.wheel_odometry()
                if odometry is not None:
                    print(
                        "[restaurant_epuck] odometry left/right = "
                        f"{odometry[0]:.3f} m / {odometry[1]:.3f} m"
                    )

                if self.target_id is not None:
                    print(f"[restaurant_epuck] current target = {self.target_id} at {self.target_pos}")



if __name__ == "__main__":
    RestaurantEpuck().run_validation_behavior()
