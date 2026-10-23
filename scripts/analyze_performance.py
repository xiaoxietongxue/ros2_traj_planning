#!/usr/bin/env python3
"""读取实验 CSV，打印对比表并画出 jerk 代价。"""

import argparse
import csv
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv")
    parser.add_argument("png")
    args = parser.parse_args()
    with open(args.csv, newline="") as handle:
        rows = list(csv.DictReader(handle))
    experiments = []
    for row in rows:
        if row["experiment"] not in experiments:
            experiments.append(row["experiment"])
    algorithms = []
    for row in rows:
        if row["algorithm"] not in algorithms:
            algorithms.append(row["algorithm"])

    print("| experiment | algorithm | compute_ms | trajectory_s | jerk | max_vel | max_acc | collision_free |")
    print("|---|---|---:|---:|---:|---:|---:|---|")
    for row in rows:
        print(
            f"| {row['experiment']} | {row['algorithm']} | {float(row['compute_ms']):.4f} | "
            f"{float(row['trajectory_time_s']):.3f} | {float(row['jerk_cost']):.4f} | "
            f"{float(row['max_velocity']):.4f} | {float(row['max_acceleration']):.4f} | "
            f"{row['collision_free']} |"
        )

    fig, ax = plt.subplots(figsize=(8, 4))
    width = 0.25
    for index, algorithm in enumerate(algorithms):
        values = []
        for experiment in experiments:
            match = next(row for row in rows if row["experiment"] == experiment and row["algorithm"] == algorithm)
            values.append(float(match["jerk_cost"]))
        positions = [i + (index - 1) * width for i in range(len(experiments))]
        ax.bar(positions, values, width=width, label=algorithm)
    ax.set_xticks(range(len(experiments)))
    ax.set_xticklabels(experiments)
    ax.set_ylabel("jerk cost")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.png, dpi=120)
    print(f"wrote {args.png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
