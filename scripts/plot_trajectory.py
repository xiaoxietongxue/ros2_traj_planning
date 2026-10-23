#!/usr/bin/env python3
"""绘制关节轨迹的位置、速度、加速度和 jerk。"""

import argparse
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv")
    parser.add_argument("png")
    args = parser.parse_args()
    data = np.genfromtxt(args.csv, delimiter=",", names=True)
    time = data["time"]
    pos = np.vstack([data[name] for name in data.dtype.names if name.startswith("pos_")])
    vel = np.vstack([data[name] for name in data.dtype.names if name.startswith("vel_")])
    acc = np.vstack([data[name] for name in data.dtype.names if name.startswith("acc_")])
    jerk = np.vstack([data[name] for name in data.dtype.names if name.startswith("jerk_")])

    fig, axes = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
    series = (
        (axes[0], pos, "position (rad)"),
        (axes[1], vel, "velocity (rad/s)"),
        (axes[2], acc, "acceleration (rad/s^2)"),
        (axes[3], jerk, "jerk (rad/s^3)"),
    )
    for ax, values, title in series:
        for joint in range(values.shape[0]):
            ax.plot(time, values[joint], label=f"joint {joint}")
        ax.set_ylabel(title)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("time (s)")
    axes[0].legend(ncol=3, fontsize=8)
    fig.tight_layout()
    fig.savefig(args.png, dpi=120)
    print(f"wrote {args.png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
