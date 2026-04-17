"""Port of ``real-time_analysis/realtime_analysis.m``.

Plots stored ``timer`` data from previous simulation runs to compare compute
time across scenarios and environment conditions.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from scipy.io import loadmat

REPO_ROOT = Path(__file__).resolve().parents[1]
RTA_DIR = REPO_ROOT / "data" / "real-time_analysis"


def _load_timer(name: str) -> dict | None:
    path = RTA_DIR / name
    if not path.exists():
        print(f"[warn] {path} not found")
        return None
    d = loadmat(str(path))
    return d


def run() -> None:
    names = {
        "t4": "time_test4.mat", "t5": "time_test5.mat",
        "t10": "time_test10.mat", "t11": "time_test11.mat",
        "t12": "time_test12.mat", "t13": "time_test13.mat",
        "t14": "time_test14.mat", "t15": "time_test15.mat",
        "t16": "time_test16.mat", "t12sp": "time_test12sp.mat",
    }
    timers = {k: d["timer"].ravel() for k, v in names.items() if (d := _load_timer(v)) is not None}

    lw = 2

    if all(k in timers for k in ("t12", "t12sp", "t13", "t14")):
        fig, ax = plt.subplots()
        for k, label in zip(
            ("t12", "t12sp", "t13", "t14"),
            ("No objects", "1 Object", "3 Objects", "12 Objects"),
        ):
            ax.plot(timers[k], "o-", linewidth=lw, label=label)
        ax.set_xlabel("Elapsed simulation time (s)")
        ax.set_ylabel("Computed time (s)")
        ax.set_title("Various scenarios with dynamic environmental constraints")
        ax.legend(); ax.grid(True)

    if all(k in timers for k in ("t4", "t10", "t15")):
        fig, ax = plt.subplots()
        for k, label in zip(
            ("t4", "t10", "t15"),
            ("No env. constraints", "Static env. constraints", "Dynamic env. constraints"),
        ):
            ax.plot(timers[k], "o-", linewidth=lw, label=label)
        ax.set_xlabel("Elapsed simulation time (s)")
        ax.set_ylabel("Computed time (s)")
        ax.set_title("1 static + 2 dynamic obstacles")
        ax.legend(); ax.grid(True)

    if all(k in timers for k in ("t5", "t11", "t16")):
        fig, ax = plt.subplots()
        for k, label in zip(
            ("t5", "t11", "t16"),
            ("No env. constraints", "Static env. constraints", "Dynamic env. constraints"),
        ):
            ax.plot(timers[k], "o-", linewidth=lw, label=label)
        ax.set_xlabel("Elapsed simulation time (s)")
        ax.set_ylabel("Computed time (s)")
        ax.set_title("2 static + 2 dynamic obstacles")
        ax.legend(); ax.grid(True)

    plt.show()


def main() -> None:
    argparse.ArgumentParser(description="Realtime analysis plots").parse_args()
    run()


if __name__ == "__main__":
    main()
