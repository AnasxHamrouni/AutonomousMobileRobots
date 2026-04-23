#!/usr/bin/env python3
import argparse
import csv
import os
from collections import defaultdict


def load_rows(csv_path):
    if not os.path.exists(csv_path):
        return []
    with open(csv_path, "r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def to_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def to_int(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def summarize_bug(rows, planner):
    grouped = defaultdict(list)
    for r in rows:
        scenario = r.get("scenario", "default")
        grouped[scenario].append(r)

    lines = []
    lines.append(f"## {planner.upper()} Summary")
    lines.append("| Scenario | Runs | Success Rate | Avg Time (s) | Avg Path (m) | Avg Wall Follow Entries | Avg Recovery Events |")
    lines.append("|---|---:|---:|---:|---:|---:|---:|")

    for scenario in sorted(grouped):
        g = grouped[scenario]
        n = len(g)
        success = sum(to_int(r.get("success", "0")) for r in g)
        avg_time = sum(to_float(r.get("duration_s", "0")) for r in g) / max(1, n)
        avg_path = sum(to_float(r.get("path_length_m", "0")) for r in g) / max(1, n)
        avg_wf = sum(to_float(r.get("wall_follow_entries", "0")) for r in g) / max(1, n)
        avg_rec = sum(to_float(r.get("recovery_events", "0")) for r in g) / max(1, n)
        success_rate = 100.0 * success / max(1, n)
        lines.append(
            f"| {scenario} | {n} | {success_rate:.1f}% | {avg_time:.2f} | {avg_path:.2f} | {avg_wf:.2f} | {avg_rec:.2f} |"
        )

    lines.append("")
    return "\n".join(lines)


def summarize_astar(rows):
    grouped = defaultdict(list)
    for r in rows:
        scenario = r.get("scenario", "default")
        grouped[scenario].append(r)

    lines = []
    lines.append("## A* Summary")
    lines.append("| Scenario | Runs | Success Rate | Avg Planning (ms) | Avg Path (m) | Avg Expanded Nodes |")
    lines.append("|---|---:|---:|---:|---:|---:|")

    for scenario in sorted(grouped):
        g = grouped[scenario]
        n = len(g)
        success = sum(to_int(r.get("success", "0")) for r in g)
        avg_plan_ms = sum(to_float(r.get("planning_time_ms", "0")) for r in g) / max(1, n)
        avg_path = sum(to_float(r.get("path_length_m", "0")) for r in g) / max(1, n)
        avg_expanded = sum(to_float(r.get("expanded_nodes", "0")) for r in g) / max(1, n)
        success_rate = 100.0 * success / max(1, n)
        lines.append(
            f"| {scenario} | {n} | {success_rate:.1f}% | {avg_plan_ms:.2f} | {avg_path:.2f} | {avg_expanded:.1f} |"
        )

    lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Summarize planner metrics CSV files into markdown tables.")
    parser.add_argument("--metrics-dir", default="metrics", help="Directory containing bug0_metrics.csv, bug1_metrics.csv, astar_metrics.csv")
    parser.add_argument("--output", default="", help="Optional output markdown file")
    args = parser.parse_args()

    metrics_dir = args.metrics_dir
    bug0_rows = load_rows(os.path.join(metrics_dir, "bug0_metrics.csv"))
    bug1_rows = load_rows(os.path.join(metrics_dir, "bug1_metrics.csv"))
    astar_rows = load_rows(os.path.join(metrics_dir, "astar_metrics.csv"))

    out_lines = []
    out_lines.append("# Planner Metrics Summary")
    out_lines.append("")

    if bug0_rows:
        out_lines.append(summarize_bug(bug0_rows, "bug0"))
    else:
        out_lines.append("## BUG0 Summary\nNo data found.\n")

    if bug1_rows:
        out_lines.append(summarize_bug(bug1_rows, "bug1"))
    else:
        out_lines.append("## BUG1 Summary\nNo data found.\n")

    if astar_rows:
        out_lines.append(summarize_astar(astar_rows))
    else:
        out_lines.append("## A* Summary\nNo data found.\n")

    text = "\n".join(out_lines)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"Wrote summary to {args.output}")
    else:
        print(text)


if __name__ == "__main__":
    main()
