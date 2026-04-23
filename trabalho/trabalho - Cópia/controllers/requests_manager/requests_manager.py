from controller import Robot, Emitter
import random

robot = Robot()
timestep = int(robot.getBasicTimeStep())

emitter: Emitter = robot.getDevice("emitter")

TABLE_IDS = ["T1", "T2", "T3", "T4", "T5", "T6"]

MIN_PERIOD_S = 8.0
MAX_PERIOD_S = 20.0

next_time = robot.getTime() + random.uniform(MIN_PERIOD_S, MAX_PERIOD_S)

while robot.step(timestep) != -1:
    now = robot.getTime()
    if now >= next_time:
        table_id = random.choice(TABLE_IDS)
        msg = f"REQ {table_id}"
        emitter.send(msg.encode("utf-8"))
        print("[MANAGER] sent:", msg)

        # próximo pedido
        next_time = now + random.uniform(MIN_PERIOD_S, MAX_PERIOD_S)