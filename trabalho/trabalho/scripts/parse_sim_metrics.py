import argparse
import csv
import re
from collections import defaultdict
from pathlib import Path


METRIC_RE = re.compile(r"\[METRIC (?P<metric>[^\]]+)\]\s+(?P<body>.*)")
KV_RE = re.compile(r"(?P<key>[A-Za-z_][A-Za-z0-9_]*)=(?P<value>[^\s]+)")
VALUE_FIELD_BY_METRIC = {
    "wait_time": "wait",
    "return_time": "return",
    "total_time": "total",
}


def parse_number(value):
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

    row = {"metric": match.group("metric")}
    for kv_match in KV_RE.finditer(match.group("body")):
        row[kv_match.group("key")] = kv_match.group("value").rstrip(",")

    return row


def read_rows(results_dir):
    rows = []

    for log_path in sorted(results_dir.glob("*.log")):
        encoding = "utf-16" if log_path.read_bytes().startswith(b"\xff\xfe") else "utf-8"

        with log_path.open("r", encoding=encoding, errors="replace") as file:
            for line_number, line in enumerate(file, start=1):
                row = parse_metric_line(line)
                if row is None:
                    continue

                row["source_log"] = log_path.name
                row["line"] = line_number
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
    grouped = defaultdict(list)

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
        grouped[("run", policy, run, metric)].append(value)
        grouped[("policy", policy, "ALL", metric)].append(value)

    summary_rows = []
    for (scope, policy, run, metric), values in sorted(grouped.items()):
        total = sum(values)
        count = len(values)
        summary_rows.append(
            {
                "scope": scope,
                "policy": policy,
                "run": run,
                "metric": metric,
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
        "run",
        "metric",
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


def main():
    parser = argparse.ArgumentParser(
        description="Extract simulation metric lines into raw and summary CSV files."
    )
    parser.add_argument(
        "--results-dir",
        required=True,
        help="Directory containing Webots .log files.",
    )
    args = parser.parse_args()

    results_dir = Path(args.results_dir)
    rows = read_rows(results_dir)

    raw_path = results_dir / "metrics_raw.csv"
    summary_path = results_dir / "metrics_summary.csv"

    write_raw_csv(rows, raw_path)
    write_summary_csv(summarize_rows(rows), summary_path)

    print(f"Parsed {len(rows)} metric rows")
    print(f"Wrote {raw_path}")
    print(f"Wrote {summary_path}")


if __name__ == "__main__":
    main()
