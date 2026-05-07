from controller import Supervisor, Emitter, Receiver
import random


robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

emitter: Emitter = robot.getDevice("emitter")
receiver: Receiver = robot.getDevice("receiver")
receiver.enable(timestep)

TABLE_IDS = ["T1", "T2", "T3", "T4", "T5", "T6"]
LAMP_DEFS = {table_id: f"TABLE_LAMP_{index}" for index, table_id in enumerate(TABLE_IDS, start=1)}

MIN_PERIOD_S = 8.0
MAX_PERIOD_S = 20.0
REQUEST_CHANNEL = 1
DONE_CHANNEL = 2
pending_requests = set()

emitter.setChannel(REQUEST_CHANNEL)
receiver.setChannel(DONE_CHANNEL)
print(
    f"[MANAGER] communication channels: "
    f"REQ->{REQUEST_CHANNEL}, DONE<-{DONE_CHANNEL}"
)


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


def set_lamp(table_id, enabled):
    lamp_field = lamp_fields.get(table_id)
    if lamp_field is None:
        print(f"[MANAGER] cannot set lamp for {table_id}: lamp field unavailable")
        return
    lamp_field.setSFBool(enabled)
    print(f"[MANAGER] lamp {'on' if enabled else 'off'} for {table_id}")


def process_done_messages():
    while receiver.getQueueLength() > 0:
        msg = receiver.getString().strip()
        receiver.nextPacket()

        parts = msg.split()
        if len(parts) != 2 or parts[0] != "DONE":
            print(f"[MANAGER] invalid message ignored: {msg}")
            continue

        table_id = parts[1]
        if table_id not in TABLE_IDS:
            print(f"[MANAGER] DONE for unknown table ignored: {table_id}")
            continue

        if table_id in pending_requests:
            pending_requests.remove(table_id)
            set_lamp(table_id, False)
            print(f"[MANAGER] request completed: {table_id}")
        else:
            print(f"[MANAGER] DONE received for non-pending table: {table_id}")


def send_random_request():
    available_tables = [
        table_id for table_id in TABLE_IDS if table_id not in pending_requests
    ]
    if not available_tables:
        print("[MANAGER] all tables pending; waiting before sending new requests")
        return

    table_id = random.choice(available_tables)
    msg = f"REQ {table_id}"
    emitter.send(msg.encode("utf-8"))
    pending_requests.add(table_id)
    set_lamp(table_id, True)
    print(f"[MANAGER] sent: {msg} | pending={sorted(pending_requests)}")


next_time = robot.getTime() + random.uniform(MIN_PERIOD_S, MAX_PERIOD_S)

while robot.step(timestep) != -1:
    process_done_messages()

    now = robot.getTime()
    if now >= next_time:
        send_random_request()
        next_time = now + random.uniform(MIN_PERIOD_S, MAX_PERIOD_S)
