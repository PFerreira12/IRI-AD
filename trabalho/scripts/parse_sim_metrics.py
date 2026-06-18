import argparse
import csv
import re
from collections import defaultdict
from pathlib import Path


METRIC_RE = re.compile(r"\[METRIC (?P<metric>[^\]]+)\]\s+(?P<body>.*)")
KV_RE = re.compile(r"(?P<key>[A-Za-z_][A-Za-z0-9_]*)=(?P<value>[^\s]+)")
VALUE_FIELD_BY_METRIC = {
    "pending_wait": "pending_wait",
    "wait_time": "wait",
    "delivery_time": "delivery",
    "return_time": "return",
    "total_time": "total",
}
METRIC_ALIASES = {
    "wait": "wait_time",
    "delivery": "delivery_time",
    "return": "return_time",
}
CONFIG_FIELDS = [
    "experiment_label",
    "map_id",
    "experiment_mode",
    "dynamic_environment",
    "policy",
    "seed",
    "repeat",
    "max_completed",
    "max_sim_time",
    "stop_delay",
    "first_period",
    "min_period",
    "max_period",
    "hybrid_wait_weight",
    "hybrid_distance_weight",
]


def parse_number(value):
    if value is None:
        return None

    if isinstance(value, (int, float)):
        return float(value)

    value = str(value)
    value = value.rstrip(",")
    if value.endswith("s"):
        value = value[:-1]

    try:
        return float(value)
    except ValueError:
        return None


def parse_metric_line(line):
    match = METRIC_RE.search(line)
    if match is None:
        return None

    metric = match.group("metric")
    row = {"metric": METRIC_ALIASES.get(metric, metric)}
    for kv_match in KV_RE.finditer(match.group("body")):
        row[kv_match.group("key")] = kv_match.group("value").rstrip(",")

    return row


def read_config(results_dir):
    config_path = results_dir / "run_config.csv"
    if not config_path.exists():
        return {}

    with config_path.open("r", newline="", encoding="utf-8-sig") as file:
        rows = list(csv.DictReader(file))

    return {row["log"]: row for row in rows if row.get("log")}


def experiment_label(row):
    label = row.get("experiment_label")
    if label:
        return label

    policy = row.get("policy", "unknown")
    if policy == "HYBRID":
        wait_weight = row.get("hybrid_wait_weight")
        distance_weight = row.get("hybrid_distance_weight")
        if wait_weight and distance_weight:
            return f"HYBRID_w{wait_weight}_d{distance_weight}"

    return policy


def read_rows(results_dir):
    rows = []
    config_by_log = read_config(results_dir)

    for log_path in sorted(results_dir.glob("*.log")):
        encoding = "utf-16" if log_path.read_bytes().startswith(b"\xff\xfe") else "utf-8"
        config = config_by_log.get(log_path.name, {})

        with log_path.open("r", encoding=encoding, errors="replace") as file:
            for line_number, line in enumerate(file, start=1):
                row = parse_metric_line(line)
                if row is None:
                    continue

                row["source_log"] = log_path.name
                row["line"] = line_number
                for field in CONFIG_FIELDS:
                    if field in config and field not in row:
                        row[field] = config[field]

                row["experiment_label"] = experiment_label(row)
                rows.append(row)

    return rows


def write_raw_csv(rows, output_path):
    fieldnames = sorted({key for row in rows for key in row})
    priority = ["metric", "policy", "run", "table", "source_log", "line"]
    fieldnames = [name for name in priority if name in fieldnames] + [
        name for name in fieldnames if name not in priority
    ]

    with output_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def summarize_rows(rows):
    grouped = defaultdict(lambda: {"values": [], "meta": {}})

    for row in rows:
        metric = row.get("metric")
        value_field = VALUE_FIELD_BY_METRIC.get(metric)
        if value_field is None:
            continue

        value = parse_number(row.get(value_field, ""))
        if value is None:
            continue

        policy = row.get("policy", "unknown")
        run = row.get("run", "unknown")
        experiment = experiment_label(row)

        hybrid_wait_weight = ""
        hybrid_distance_weight = ""
        if policy == "HYBRID":
            hybrid_wait_weight = row.get("hybrid_wait_weight", "")
            hybrid_distance_weight = row.get("hybrid_distance_weight", "")

        meta = {
            "experiment_label": experiment,
            "hybrid_wait_weight": hybrid_wait_weight,
            "hybrid_distance_weight": hybrid_distance_weight,
        }

        for key in [
            ("run", policy, experiment, run, metric),
            ("experiment", policy, experiment, "ALL", metric),
            ("policy", policy, policy, "ALL", metric),
        ]:
            grouped[key]["values"].append(value)
            grouped[key]["meta"].update(meta)

    summary_rows = []
    for (scope, policy, experiment, run, metric), group in sorted(grouped.items()):
        values = group["values"]
        total = sum(values)
        count = len(values)
        summary_rows.append(
            {
                "scope": scope,
                "policy": policy,
                "experiment_label": experiment,
                "run": run,
                "metric": metric,
                "hybrid_wait_weight": group["meta"].get("hybrid_wait_weight", ""),
                "hybrid_distance_weight": group["meta"].get(
                    "hybrid_distance_weight",
                    "",
                ),
                "count": count,
                "total": f"{total:.3f}",
                "mean": f"{total / count:.3f}",
                "min": f"{min(values):.3f}",
                "max": f"{max(values):.3f}",
            }
        )

    return summary_rows


def write_summary_csv(rows, output_path):
    fieldnames = [
        "scope",
        "policy",
        "experiment_label",
        "run",
        "metric",
        "hybrid_wait_weight",
        "hybrid_distance_weight",
        "count",
        "total",
        "mean",
        "min",
        "max",
    ]

    with output_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def rank_experiments(summary_rows, max_wait_weight):
    metrics_by_experiment = {}

    for row in summary_rows:
        if row["scope"] != "experiment":
            continue

        key = (
            row["policy"],
            row["experiment_label"],
            row.get("hybrid_wait_weight", ""),
            row.get("hybrid_distance_weight", ""),
        )

        target = metrics_by_experiment.setdefault(
            key,
            {
                "policy": row["policy"],
                "experiment_label": row["experiment_label"],
                "hybrid_wait_weight": row.get("hybrid_wait_weight", ""),
                "hybrid_distance_weight": row.get("hybrid_distance_weight", ""),
            },
        )

        metric = row["metric"]
        target[f"{metric}_count"] = row["count"]
        target[f"{metric}_total"] = row["total"]
        target[f"{metric}_mean"] = row["mean"]
        target[f"{metric}_max"] = row["max"]

    ranked_rows = list(metrics_by_experiment.values())

    for row in ranked_rows:
        wait_count = int(parse_number(row.get("wait_time_count", "")) or 0)
        pending_count = int(parse_number(row.get("pending_wait_count", "")) or 0)
        wait_total = parse_number(row.get("wait_time_total", "")) or 0.0
        pending_total = parse_number(row.get("pending_wait_total", "")) or 0.0
        wait_max = parse_number(row.get("wait_time_max", "")) or 0.0
        pending_max = parse_number(row.get("pending_wait_max", "")) or 0.0

        effective_count = wait_count + pending_count
        effective_total = wait_total + pending_total
        effective_max = max(wait_max, pending_max)

        if effective_count == 0:
            row["effective_wait_count"] = ""
            row["effective_wait_total"] = ""
            row["effective_wait_mean"] = ""
            row["effective_wait_max"] = ""
        else:
            row["effective_wait_count"] = effective_count
            row["effective_wait_total"] = f"{effective_total:.3f}"
            row["effective_wait_mean"] = f"{effective_total / effective_count:.3f}"
            row["effective_wait_max"] = f"{effective_max:.3f}"

        wait_mean = parse_number(row.get("effective_wait_mean", ""))
        wait_max = parse_number(row.get("effective_wait_max", ""))

        if wait_mean is None or wait_max is None:
            row["fair_wait_objective"] = ""
        else:
            row["fair_wait_objective"] = f"{wait_mean + max_wait_weight * wait_max:.3f}"

        row["max_wait_weight"] = f"{max_wait_weight:.3f}"

    ranked_by_objective = sorted(
        ranked_rows,
        key=lambda row: (
            parse_number(row.get("fair_wait_objective", "")) is None,
            parse_number(row.get("fair_wait_objective", "")) or 0.0,
            parse_number(row.get("wait_time_mean", "")) or 0.0,
        ),
    )

    for index, row in enumerate(ranked_by_objective, start=1):
        row["rank_by_fair_wait"] = index

    ranked_by_mean = sorted(
        ranked_rows,
        key=lambda row: (
            parse_number(row.get("wait_time_mean", "")) is None,
            parse_number(row.get("wait_time_mean", "")) or 0.0,
            parse_number(row.get("fair_wait_objective", "")) or 0.0,
        )
    )

    for index, row in enumerate(ranked_by_mean, start=1):
        row["rank_by_mean_wait"] = index

    return ranked_by_objective


def write_ranked_csv(rows, output_path):
    fieldnames = [
        "rank_by_fair_wait",
        "rank_by_mean_wait",
        "policy",
        "experiment_label",
        "hybrid_wait_weight",
        "hybrid_distance_weight",
        "max_wait_weight",
        "fair_wait_objective",
        "effective_wait_count",
        "effective_wait_total",
        "effective_wait_mean",
        "effective_wait_max",
        "wait_time_count",
        "wait_time_total",
        "wait_time_mean",
        "wait_time_max",
        "pending_wait_count",
        "pending_wait_total",
        "pending_wait_mean",
        "pending_wait_max",
        "total_time_mean",
        "total_time_max",
        "return_time_mean",
        "return_time_max",
    ]

    with output_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(
        description="Extract simulation metric lines into raw and summary CSV files."
    )
    parser.add_argument(
        "--results-dir",
        required=True,
        help="Directory containing Webots .log files.",
    )
    parser.add_argument(
        "--max-wait-weight",
        type=float,
        default=0.25,
        help=(
            "Penalty multiplier for max wait time in the fair objective: "
            "mean_wait + max_wait_weight * max_wait."
        ),
    )
    args = parser.parse_args()

    results_dir = Path(args.results_dir)
    rows = read_rows(results_dir)

    raw_path = results_dir / "metrics_raw.csv"
    summary_path = results_dir / "metrics_summary.csv"
    ranked_path = results_dir / "metrics_ranked.csv"
    summary_rows = summarize_rows(rows)

    write_raw_csv(rows, raw_path)
    write_summary_csv(summary_rows, summary_path)
    write_ranked_csv(
        rank_experiments(summary_rows, args.max_wait_weight),
        ranked_path,
    )

    print(f"Parsed {len(rows)} metric rows")
    print(f"Wrote {raw_path}")
    print(f"Wrote {summary_path}")
    print(f"Wrote {ranked_path}")


if __name__ == "__main__":
    main()
