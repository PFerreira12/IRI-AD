"""Simple local controller for the restaurant world.

The robot drives forward by default and turns away when the proximity
sensors detect nearby obstacles, which is enough to validate the world
and controller wiring inside Webots.
"""

from controller import Robot


TIME_STEP = 32
CRUISE_SPEED = 4.0
TURN_SPEED = 2.0
OBSTACLE_THRESHOLD = 80.0


robot = Robot()

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

lidar = robot.getDevice("lidar")
lidar.enable(TIME_STEP)

proximity_sensors = []
for index in range(8):
    sensor = robot.getDevice(f"ps{index}")
    sensor.enable(TIME_STEP)
    proximity_sensors.append(sensor)


def compute_wheel_speeds():
    values = [sensor.getValue() for sensor in proximity_sensors]

    left_front = max(values[5], values[6], values[7])
    right_front = max(values[0], values[1], values[2])

    if left_front > OBSTACLE_THRESHOLD and right_front > OBSTACLE_THRESHOLD:
        return TURN_SPEED, -TURN_SPEED
    if left_front > OBSTACLE_THRESHOLD:
        return TURN_SPEED, -TURN_SPEED
    if right_front > OBSTACLE_THRESHOLD:
        return -TURN_SPEED, TURN_SPEED

    return CRUISE_SPEED, CRUISE_SPEED


while robot.step(TIME_STEP) != -1:
    left_speed, right_speed = compute_wheel_speeds()
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
