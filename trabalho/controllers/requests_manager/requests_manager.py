from controller import Supervisor, Emitter, Receiver
import os
import random
from pathlib import Path
import sys

CURRENT_DIR = Path(__file__).resolve().parent
CONTROLLERS_DIR = CURRENT_DIR.parent
COMMON_DIR = CONTROLLERS_DIR / "common"

sys.path.insert(0, str(COMMON_DIR))

from config_tables import get_map_config


robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

emitter: Emitter = robot.getDevice("emitter")
receiver: Receiver = robot.getDevice("receiver")
receiver.enable(timestep)

MAP_CONFIG = get_map_config()
TABLE_IDS = MAP_CONFIG["table_ids"]
LAMP_DEFS = {
    table_id: f"TABLE_LAMP_{index}"
    for index, table_id in enumerate(TABLE_IDS, start=1)
}

REQUEST_CHANNEL = 1
DONE_CHANNEL = 2


def env_float(name, default):
    raw_value = os.environ.get(name)
    if raw_value is None:
        return default

    try:
        return float(raw_value)
    except ValueError:
        print(f"[MANAGER] invalid {name}={raw_value!r}; using {default}")
        return default


def env_int(name, default):
    raw_value = os.environ.get(name)
    if raw_value is None:
        return default

    try:
        return int(raw_value)
    except ValueError:
        print(f"[MANAGER] invalid {name}={raw_value!r}; using {default}")
        return default


FIRST_PERIOD_S = env_float("RM_FIRST_PERIOD_S", 5.0)
MIN_PERIOD_S = env_float("RM_MIN_PERIOD_S", 25.0)
MAX_PERIOD_S = env_float("RM_MAX_PERIOD_S", 35.0)
if MAX_PERIOD_S < MIN_PERIOD_S:
    print("[MANAGER] RM_MAX_PERIOD_S below RM_MIN_PERIOD_S; using min value")
    MAX_PERIOD_S = MIN_PERIOD_S

MAX_COMPLETED_REQUESTS = env_int("RM_MAX_COMPLETED_REQUESTS", 0)
MAX_SIM_TIME_S = env_float("RM_MAX_SIM_TIME_S", 0.0)
STOP_DELAY_S = env_float("RM_STOP_DELAY_S", 0.0)
RUN_ID = os.environ.get("SIM_RUN_ID", "manual")
REQUEST_POLICY = os.environ.get("REQUEST_POLICY", "FIFO").upper()
RANDOM_SEED = os.environ.get("RM_RANDOM_SEED")

if RANDOM_SEED is not None:
    try:
        random.seed(int(RANDOM_SEED))
    except ValueError:
        random.seed(RANDOM_SEED)

pending_requests = set()
request_created_times = {}
completed_requests = 0
stop_requested_at = None
pending_metrics_reported = False

emitter.setChannel(REQUEST_CHANNEL)
receiver.setChannel(DONE_CHANNEL)
print(
    f"[MANAGER] communication channels: "
    f"REQ->{REQUEST_CHANNEL}, DONE<-{DONE_CHANNEL}"
)
print(
    "[MANAGER] simulation config "
    f"map={MAP_CONFIG['id']} "
    f"run={RUN_ID} "
    f"policy={REQUEST_POLICY} "
    f"seed={RANDOM_SEED} "
    f"first_period={FIRST_PERIOD_S:.2f} "
    f"period=({MIN_PERIOD_S:.2f},{MAX_PERIOD_S:.2f}) "
    f"max_completed={MAX_COMPLETED_REQUESTS} "
    f"max_time={MAX_SIM_TIME_S:.2f} "
    f"stop_delay={STOP_DELAY_S:.2f}"
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
    global completed_requests

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
            completed_at = robot.getTime()
            requested_at = request_created_times.pop(table_id, None)
            pending_requests.remove(table_id)
            completed_requests += 1
            set_lamp(table_id, False)

            if requested_at is None:
                print(f"[MANAGER] request completed: {table_id}")
            else:
                total_time = completed_at - requested_at
                print(
                    f"[MANAGER] request completed: {table_id} "
                    f"total_time={total_time:.2f}s"
                )
                print(
                    "[METRIC total_time] "
                    f"run={RUN_ID} "
                    f"policy={REQUEST_POLICY} "
                    f"table={table_id} "
                    f"requested_at={requested_at:.2f} "
                    f"completed_at={completed_at:.2f} "
                    f"total={total_time:.2f}s "
                    f"count={completed_requests}"
                )
        else:
            print(f"[MANAGER] DONE received for non-pending table: {table_id}")


def send_random_request():
    available_tables = [
        table_id for table_id in TABLE_IDS if table_id not in pending_requests
    ]
    if not available_tables:
        print("[MANAGER] all tables pending; waiting before sending new requests")
        return False

    table_id = random.choice(available_tables)
    requested_at = robot.getTime()
    msg = f"REQ {table_id} {requested_at:.3f}"
    emitter.send(msg.encode("utf-8"))
    pending_requests.add(table_id)
    request_created_times[table_id] = requested_at
    set_lamp(table_id, True)
    print(
        f"[MANAGER] sent: {msg} | "
        f"requested_at={requested_at:.2f} "
        f"pending={sorted(pending_requests)}"
    )
    return True


def should_stop():
    global pending_metrics_reported, stop_requested_at

    now = robot.getTime()

    if MAX_COMPLETED_REQUESTS > 0 and completed_requests >= MAX_COMPLETED_REQUESTS:
        if stop_requested_at is None:
            stop_requested_at = now
            print(
                "[MANAGER] completion limit reached; stopping new requests "
                f"completed_requests={completed_requests} "
                f"stop_delay={STOP_DELAY_S:.2f}s"
            )
            if not pending_metrics_reported:
                report_pending_metrics(now)
                pending_metrics_reported = True

        if now - stop_requested_at >= STOP_DELAY_S:
            print(
                "[MANAGER] stopping simulation: "
                f"completed_requests={completed_requests}"
            )
            return True

    if MAX_SIM_TIME_S > 0.0 and now >= MAX_SIM_TIME_S:
        if not pending_metrics_reported:
            report_pending_metrics(now)
            pending_metrics_reported = True

        print("[MANAGER] stopping simulation: max simulation time reached")
        return True

    return False


def report_pending_metrics(stop_at):
    for table_id in sorted(pending_requests):
        requested_at = request_created_times.get(table_id)
        if requested_at is None:
            continue

        pending_wait = max(0.0, stop_at - requested_at)
        print(
            "[METRIC pending_wait] "
            f"run={RUN_ID} "
            f"policy={REQUEST_POLICY} "
            f"table={table_id} "
            f"requested_at={requested_at:.2f} "
            f"stop_at={stop_at:.2f} "
            f"pending_wait={pending_wait:.2f}s"
        )


# Initialize the first request time
next_time = robot.getTime() + FIRST_PERIOD_S

while robot.step(timestep) != -1:
    process_done_messages()

    if should_stop():
        robot.simulationQuit(0)
        break

    now = robot.getTime()
    if stop_requested_at is None and now >= next_time:
        send_random_request()
        next_time = now + random.uniform(MIN_PERIOD_S, MAX_PERIOD_S)
