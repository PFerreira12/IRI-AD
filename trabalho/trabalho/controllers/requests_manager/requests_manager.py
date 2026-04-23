from controller import Supervisor, Emitter
import random


robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

emitter: Emitter = robot.getDevice("emitter")

TABLE_IDS = ["T1", "T2", "T3", "T4", "T5", "T6"]
LAMP_DEFS = {table_id: f"TABLE_LAMP_{index}" for index, table_id in enumerate(TABLE_IDS, start=1)}

MIN_PERIOD_S = 8.0
MAX_PERIOD_S = 20.0


def get_lamp_fields():
    lamp_fields = {}
    for table_id, def_name in LAMP_DEFS.items():
        node = robot.getFromDef(def_name)
        if node is None:
            print(f"[MANAGER] lamp node not found for {table_id}: DEF {def_name}")
            continue

        on_field = node.getField("on")
        if on_field is None:
            print(f"[MANAGER] 'on' field missing for {table_id}: DEF {def_name}")
            continue

        lamp_fields[table_id] = on_field

    return lamp_fields


lamp_fields = get_lamp_fields()
for field in lamp_fields.values():
    field.setSFBool(False)

next_time = robot.getTime() + random.uniform(MIN_PERIOD_S, MAX_PERIOD_S)

while robot.step(timestep) != -1:
    now = robot.getTime()
    if now >= next_time:
        table_id = random.choice(TABLE_IDS)
        msg = f"REQ {table_id}"
        emitter.send(msg.encode("utf-8"))
        print("[MANAGER] sent:", msg)

        lamp_field = lamp_fields.get(table_id)
        if lamp_field is not None:
            lamp_field.setSFBool(True)
            print(f"[MANAGER] lamp on for {table_id}")

        next_time = now + random.uniform(MIN_PERIOD_S, MAX_PERIOD_S)
