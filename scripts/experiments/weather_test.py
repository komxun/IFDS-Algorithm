"""Port of ``for_experimenting/weather_test.m`` — minimal weather+IFDS sanity demo."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from ifds.config import OptimizerMode, Param, Scene, SimMode
from ifds.ifds import run_ifds
from ifds.scenes import allocate_objects
from ifds.weather import load_weather_field
from viz.plotting import plot_objects_mpl

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_WEATHER = REPO_ROOT / "data" / "WeatherMat_321.mat"


def run(weather_path: Path = DEFAULT_WEATHER, *, plot: bool = True) -> dict:
    param = Param(
        tsim=400, rtsim=1, dt=0.1, sim_mode=SimMode.BY_TIME,
        scene=Scene(0), multi_target=False, sf=False,
        rho0_initial=0.5, sigma0_initial=0.01,
        use_optimizer=OptimizerMode.OFF, rg=5.0,
        c=30.0, target_thresh=2.5,
        xini=0.0, yini=0.0, zini=0.0,
        xfinal=200.0, yfinal=0.0, zfinal=10.0,
        env="dynamic", k=1.0, b_u=0.7, b_l=0.0,
        show_disp=True,
    )

    weather = load_weather_field(weather_path, b_l=param.b_l, b_u=param.b_u) \
        if weather_path.exists() else None

    objects = allocate_objects(param.scene)
    wp = np.zeros((3, param.tsim + 1))
    wp[:, 0] = [param.xini, param.yini, param.zini]
    paths: list[list[np.ndarray | None]] = [[None]]
    paths, objects, length, _found = run_ifds(
        rho0=param.rho0_initial, sigma0=param.sigma0_initial,
        alpha_deg=0.0, loc_final=np.array([param.xfinal, param.yfinal, param.zfinal]),
        rt=0, wp=wp, paths=paths, param=param, line_index=0,
        objects=objects, weather=weather,
    )
    print(f"Total path length: {length:.2f} m")
    print(f"Total flight time: {length / param.c:.2f} s")

    if plot and paths[0][0] is not None:
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection="3d")
        w = paths[0][0]
        ax.plot(w[0], w[1], w[2], 'b', linewidth=1.5)
        ax.scatter([0], [0], [0], c='r')
        ax.scatter([param.xfinal], [param.yfinal], [param.zfinal], marker='x', c='r', s=150)
        plot_objects_mpl(ax, objects)
        if weather is not None:
            ax.contourf(
                np.arange(weather.mat.shape[0]) + 1,
                np.arange(weather.mat.shape[1]) - 100,
                weather.mat[:, :, 0].T, 30, cmap="turbo", offset=0,
            )
        plt.show()

    return dict(paths=paths, objects=objects, length=length)


def main() -> None:
    argparse.ArgumentParser().parse_args()
    run()


if __name__ == "__main__":
    main()
