"""Port of ``path_distance_objective.m`` — legacy single-call path-length objective.

Kept for reference. For modern usage prefer :func:`ifds.optimizer.make_global_objective`.
"""

from __future__ import annotations

import argparse

import numpy as np

from ifds.config import OptimizerMode, Param, Scene, SimMode
from ifds.ifds import run_ifds
from ifds.scenes import allocate_objects


def path_distance_objective(rho0: float, sigma0: float, scene: int = 2) -> float:
    param = Param(
        tsim=400, rtsim=1, dt=0.1, sim_mode=SimMode.BY_TIME,
        scene=Scene(scene), multi_target=False, sf=False,
        rho0_initial=rho0, sigma0_initial=sigma0,
        use_optimizer=OptimizerMode.OFF, rg=0.0,
        c=30.0, target_thresh=2.5,
        xini=0.0, yini=0.0, zini=0.0,
        xfinal=200.0, yfinal=0.0, zfinal=50.0,
        env="static", show_disp=False,
    )
    objects = allocate_objects(param.scene)
    wp = np.zeros((3, param.tsim + 1))
    wp[:, 0] = [param.xini, param.yini, param.zini]
    paths: list[list[np.ndarray | None]] = [[None]]
    _paths, _obj, length, _found = run_ifds(
        rho0=rho0, sigma0=sigma0, alpha_deg=0.0,
        loc_final=np.array([param.xfinal, param.yfinal, param.zfinal]),
        rt=0, wp=wp, paths=paths, param=param, line_index=0,
        objects=objects, weather=None,
    )
    return length


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--rho0", type=float, default=1.0)
    p.add_argument("--sigma0", type=float, default=1.0)
    p.add_argument("--scene", type=int, default=2)
    args = p.parse_args()
    length = path_distance_objective(args.rho0, args.sigma0, args.scene)
    print(f"Total path length: {length:.4f} m")


if __name__ == "__main__":
    main()
