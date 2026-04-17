"""Port of ``main_sg_generation.m``.

Pure IFDS path generation demo with optional multi-target destinations, without
the CCA path-follower or weather. Useful for visualizing the Gamma distribution.
"""

from __future__ import annotations

import argparse

import matplotlib.pyplot as plt
import numpy as np

from ifds.config import OptimizerMode, Param, Scene, SimMode
from ifds.ifds import run_ifds
from ifds.scenes import allocate_objects
from viz.gamma_plot import plot_gamma_distribution
from viz.plotting import plot_objects_mpl, plot_path_2d


def run(scene: int = 2, multi_target: bool = True, *, plot: bool = True) -> dict:
    param = Param(
        tsim=400, rtsim=1, dt=0.1, sim_mode=SimMode.BY_TIME,
        scene=Scene(scene), multi_target=multi_target, sf=False,
        rho0_initial=1.0, sigma0_initial=0.01,
        use_optimizer=OptimizerMode.OFF, rg=10.0,
        k=0.0, c=30.0, target_thresh=2.5,
        xini=0.0, yini=0.0, zini=0.0,
        xfinal=200.0, yfinal=0.0, zfinal=10.0,
        env="static", show_disp=True,
    )

    if multi_target:
        destin = np.array(
            [
                [200, 0, 20], [200, 20, 20], [200, -20, 20],
                [200, 20, 30], [200, -20, 30], [200, 0, 30],
                [200, 0, 40], [200, 20, 40], [200, -20, 40],
            ], dtype=float,
        )
    else:
        destin = np.array([[param.xfinal, param.yfinal, param.zfinal]], dtype=float)
    num_line = destin.shape[0]

    objects = allocate_objects(param.scene)
    wp = np.zeros((3, param.tsim + 1))
    wp[:, 0] = [param.xini, param.yini, param.zini]
    paths: list[list[np.ndarray | None]] = [[None] for _ in range(num_line)]

    for line in range(num_line):
        paths, objects, _length, _found = run_ifds(
            rho0=param.rho0_initial, sigma0=param.sigma0_initial,
            alpha_deg=0.0, loc_final=destin[line],
            rt=0, wp=wp.copy(), paths=paths, param=param, line_index=line,
            objects=objects, weather=None,
        )

    if plot:
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection="3d")
        plot_path_2d(ax, paths, 0, param.xini, param.yini, param.zini, destin, multi_target)
        plot_objects_mpl(ax, objects)
        ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_zlabel("Z [m]")
        plt.tight_layout()
        plt.show()
        plot_gamma_distribution(objects)
        plt.show()

    return dict(paths=paths, objects=objects, destin=destin)


def main() -> None:
    p = argparse.ArgumentParser(description="SG generation demo (port of main_sg_generation.m)")
    p.add_argument("--scene", type=int, default=2)
    p.add_argument("--single", dest="multi", action="store_false")
    p.add_argument("--no-plot", dest="plot", action="store_false")
    args = p.parse_args()
    run(scene=args.scene, multi_target=args.multi, plot=args.plot)


if __name__ == "__main__":
    main()
