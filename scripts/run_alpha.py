"""Port of ``main_ALPHA.m``.

Sweeps the tangential rotation angle ``alpha_deg`` and overlays the resulting
IFDS paths for a single obstacle scenario.
"""

from __future__ import annotations

import argparse

import matplotlib.pyplot as plt
import numpy as np

from ifds.config import OptimizerMode, Param, Scene, SimMode, cca_preset
from ifds.ifds import run_ifds
from ifds.scenes import allocate_objects
from viz.plotting import plot_objects_mpl


DEFAULT_ALPHAS = [0.0, 75.0, 90.0, 105.0, 180.0, 255.0, 270.0, 285.0]


def run(scene: int = 1, alphas=None, *, plot: bool = True) -> list[np.ndarray]:
    alphas = list(alphas or DEFAULT_ALPHAS)
    param = Param(
        tsim=100, rtsim=1, dt=0.1, sim_mode=SimMode.BY_DISTANCE,
        scene=Scene(scene), multi_target=False, sf=False,
        rho0_initial=1.0, sigma0_initial=1.0,
        use_optimizer=OptimizerMode.OFF, rg=0.0,
        k=0.0, env="static",
        xini=0.0, yini=0.0, zini=50.0,
        xfinal=200.0, yfinal=0.0, zfinal=50.0,
        x_i=0.0, y_i=0.0, z_i=50.0,
        cca=cca_preset(5), show_disp=False,
    )

    routes: list[np.ndarray] = []
    objects = allocate_objects(param.scene)
    for alpha in alphas:
        print(f"Alpha = {alpha} deg")
        wp = np.zeros((3, param.tsim + 1))
        wp[:, 0] = [param.xini, param.yini, param.zini]
        paths: list[list[np.ndarray | None]] = [[None]]
        paths, objects, _length, found = run_ifds(
            rho0=param.rho0_initial, sigma0=param.sigma0_initial,
            alpha_deg=alpha, loc_final=np.array([param.xfinal, param.yfinal, param.zfinal]),
            rt=0, wp=wp, paths=paths, param=param, line_index=0,
            objects=objects, weather=None,
        )
        if found and paths[0][0] is not None:
            routes.append(paths[0][0])

    if plot and routes:
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection="3d")
        colors = ["#0072BD", "#EDB120", "#77AC30", "#A2142F",
                  "#D95319", "#7E2F8E", "#4DBEEE", "#000000"]
        for i, route in enumerate(routes):
            ax.plot(route[0], route[1], route[2], color=colors[i % len(colors)],
                    linewidth=1.8, label=f"α={alphas[i]}°")
        plot_objects_mpl(ax, objects)
        ax.set_xlim(0, 200); ax.set_ylim(-50, 50); ax.set_zlim(0, 100)
        ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_zlabel("Z [m]")
        ax.legend()
        plt.tight_layout()
        plt.show()
    return routes


def main() -> None:
    p = argparse.ArgumentParser(description="Alpha sweep (port of main_ALPHA.m)")
    p.add_argument("--scene", type=int, default=1)
    p.add_argument("--no-plot", dest="plot", action="store_false")
    args = p.parse_args()
    run(scene=args.scene, plot=args.plot)


if __name__ == "__main__":
    main()
