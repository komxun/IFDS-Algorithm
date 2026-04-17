"""Port of ``for_experimenting/test_optimize_dist_fmincon.m``.

Minimize path length as a function of ``(rho0, sigma0)`` using SLSQP
(scipy replacement for MATLAB ``fmincon`` interior-point).
"""

from __future__ import annotations

import argparse

import numpy as np
from scipy.optimize import minimize

from ifds.config import OptimizerMode, Param, Scene, SimMode
from scripts.path_dist_objective_v2 import path_dist_objective_v2


def run() -> None:
    param = Param(
        tsim=400, rtsim=1, dt=0.1, sim_mode=SimMode.BY_TIME,
        scene=Scene(2), multi_target=False, sf=True,
        rho0_initial=1.3, sigma0_initial=0.01,
        use_optimizer=OptimizerMode.OFF, rg=0.0,
        c=30.0, target_thresh=2.5,
        xini=0.0, yini=0.0, zini=0.0,
        xfinal=200.0, yfinal=0.0, zfinal=50.0,
        env="static", show_disp=False,
    )

    x0 = np.array([1.0, 0.01])
    bounds = [(0.0, 2.0), (0.0, 1.0)]
    res = minimize(
        lambda x: path_dist_objective_v2(float(x[0]), float(x[1]), param),
        x0=x0, method="SLSQP", bounds=bounds, options={"disp": True},
    )
    print("Optimized decision variables (rho0, sigma0):", res.x)
    print("Optimized path distance (m):", res.fun)
    print("Original path distance =",
          path_dist_objective_v2(float(x0[0]), float(x0[1]), param), "m")


def main() -> None:
    argparse.ArgumentParser().parse_args()
    run()


if __name__ == "__main__":
    main()
