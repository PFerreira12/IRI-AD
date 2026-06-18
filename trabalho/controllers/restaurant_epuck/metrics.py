import csv, os, json, math


class MetricsManager:
    
    def __init__(
        self,
        request_policy="FIFO",
        experiment_mode="EXP1",
        dynamic_env=False,
        map_id="map1"
    ):

        self.metrics_dir = os.environ.get("METRICS_DIR", "metrics_results")
        os.makedirs(self.metrics_dir, exist_ok=True)

        self.request_policy = request_policy

        self.simulation_id = self._next_simulation_id()

        self.events_file = os.path.join(self.metrics_dir, "events.csv")
        self.runs_file = os.path.join(self.metrics_dir, "runs.csv")

        self.results_file = os.path.join(
            self.metrics_dir,
            f"simulation_{self.simulation_id}.json"
        )

        # state
        self.mission_start_time = None
        self.prev_pos = None
        self.total_distance = 0.0

        self.total_requests = 0
        self.completed_requests = 0

        self.wait_times = []
        self.delivery_times = []
        self.return_times = []

        self.near_collision_count = 0

        self.request_policy = request_policy
        self.experiment_mode = experiment_mode
        self.dynamic_env = dynamic_env
        self.map_id = map_id

    # -------------------------
    # CORE LIFECYCLE
    # -------------------------

    def start_mission(self, start_time):
        self.mission_start_time = start_time
        self.total_distance = 0.0
        self.prev_pos = None


    def end_mission(self):
        self.mission_start_time = None

    # -------------------------
    # REQUEST TRACKING
    # -------------------------

    def register_request(self):
        self.total_requests += 1
        self.log_event("request", value=self.total_requests)


    def register_completion(self):
        self.completed_requests += 1
        self.log_event("completion", value=self.completed_requests)

    # -------------------------
    # DISTANCE
    # -------------------------

    def update_distance(self, current_pos):
        if current_pos is None:
            return

        if self.prev_pos is not None:
            self.total_distance += math.hypot(
                current_pos[0] - self.prev_pos[0],
                current_pos[1] - self.prev_pos[1]
            )

        self.prev_pos = current_pos

    # -------------------------
    # METRICS RECORDING
    # -------------------------

    def record_wait_time(self, table_id, requested_at, served_at):
        t = max(0.0, served_at - requested_at)
        self.wait_times.append(t)

        self.log_event("wait_time", table_id, t)

        print(
            f"[METRIC wait] run={self.simulation_id} "
            f"table={table_id} wait={t:.2f}s avg={sum(self.wait_times)/len(self.wait_times):.2f}s"
        )


    def record_delivery_time(self, table_id, start_at, arrived_at):
        t = max(0.0, arrived_at - start_at)
        self.delivery_times.append(t)

        self.log_event("delivery_time", table_id, t)

        print(
            f"[METRIC delivery] run={self.simulation_id} "
            f"table={table_id} delivery={t:.2f}s"
        )


    def record_return_time(self, table_id, start_at, returned_at):
        t = max(0.0, returned_at - start_at)
        self.return_times.append(t)

        self.log_event("return_time", table_id, t)

        print(
            f"[METRIC return] run={self.simulation_id} "
            f"table={table_id} return={t:.2f}s"
        )


    def get_avg_speed(self, mission_time):
        if mission_time<=0: return 0.0
        return self.total_distance/mission_time
    

    # -------------------------
    # SUMMARY
    # -------------------------

    def print_success_rate(self):
        if self.total_requests == 0:
            rate = 0.0
        else:
            rate = self.completed_requests / self.total_requests

        print(
            f"[METRIC success_rate] run={self.simulation_id} "
            f"completed={self.completed_requests} "
            f"total={self.total_requests} "
            f"rate={rate:.2f}"
        )

        return rate


    def print_mission_metrics(self, mission_time):
        avg_speed = 0.0
        if mission_time > 0:
            avg_speed = self.total_distance / mission_time

        print(
            f"[METRIC mission] run={self.simulation_id} "
            f"time={mission_time:.2f}s distance={self.total_distance:.3f} "
            f"avg_speed={avg_speed:.3f}"
        )

        return avg_speed

    # -------------------------
    # SIMULATION ID
    # -------------------------

    def _next_simulation_id(self):
        counter_file = os.path.join(self.metrics_dir, "simulation_counter.json")

        if os.path.exists(counter_file):
            with open(counter_file, "r") as f:
                data = json.load(f)
        else:
            data = {"last_id": 0}

        data["last_id"] += 1

        with open(counter_file, "w") as f:
            json.dump(data, f, indent=2)

        return data["last_id"]


    def log_event(self, event_type, table_id=None, value=None, timestamp=None):
        
        file_exists = os.path.isfile(self.events_file)

        row = {
            "simulation_id": self.simulation_id,
            "event_type": event_type,
            "table_id": table_id,
            "value": value,
            "timestamp": timestamp
        }

        with open(self.events_file, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=row.keys())

            if not file_exists:
                writer.writeheader()

            writer.writerow(row)
    

    def save_results(self, last_table_id, mission_time):
        
        avg_wait = sum(self.wait_times) / len(self.wait_times) if self.wait_times else 0.0
        avg_delivery = sum(self.delivery_times) / len(self.delivery_times) if self.delivery_times else 0.0
        avg_return = sum(self.return_times) / len(self.return_times) if self.return_times else 0.0

        avg_speed = self.get_avg_speed(mission_time)

        success_rate = (
            self.completed_requests/self.total_requests
            if self.total_requests > 0
            else 0.0
        )

        row = {
            "simulation_id": self.simulation_id,
            "experiment": self.experiment_mode,
            "map_id": self.map_id,
            "dynamic_environment": self.dynamic_env,
            "policy": self.request_policy,

            "last_table": last_table_id,

            "total_requests": self.total_requests,
            "completed_requests": self.completed_requests,
            "success_rate": success_rate,

            "mission_time": mission_time,
            "distance": self.total_distance,
            "avg_speed": avg_speed,

            "collisions": self.near_collision_count,

            "avg_wait": avg_wait,
            "avg_delivery": avg_delivery,
            "avg_return": avg_return,
        }

        file_exists = os.path.isfile(self.runs_file)

        with open(self.runs_file, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=row.keys())

            if not file_exists:
                writer.writeheader()

            writer.writerow(row)
