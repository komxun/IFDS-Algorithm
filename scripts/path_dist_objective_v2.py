"""Port of ``path_dist_objective_v2.m`` — parameterized path-length objective.

Takes a ``Param`` object (matching the MATLAB signature) and returns the total
Euclidean path length obtained by running IFDS once.
"""

from __future__ import annotations

import numpy as np

from ifds.config import OptimizerMode, Param
from ifds.ifds import run_ifds
from ifds.scenes import allocate_objects


def path_dist_objective_v2(rho0: float, sigma0: float, param: Param) -> float:
    """Return the total path length for the first destination under ``param``."""
    p = param
    multi_target = p.multi_target
    if multi_target:
        destin = np.array(
            [
                [200, 0, 20], [200, 20, 20], [200, -20, 20],
                [200, 20, 30], [200, -20, 30], [200, 0, 30],
                [200, 0, 40], [200, 20, 40], [200, -20, 40],
            ], dtype=float,
        )
    else:
        destin = np.array([[p.xfinal, p.yfinal, p.zfinal]], dtype=float)
    num_line = destin.shape[0]

    objects = allocate_objects(p.scene)
    wp = np.zeros((3, p.tsim + 1))
    wp[:, 0] = [p.xini, p.yini, p.zini]
    paths: list[list[np.ndarray | None]] = [[None] for _ in range(num_line)]

    import dataclasses
    p_quiet = dataclasses.replace(p, show_disp=False, use_optimizer=OptimizerMode.OFF)
    for line in range(num_line):
        paths, objects, _length, _found = run_ifds(
            rho0=rho0, sigma0=sigma0, alpha_deg=0.0, loc_final=destin[line],
            rt=0, wp=wp.copy(), paths=paths, param=p_quiet, line_index=line,
            objects=objects, weather=None,
        )

    wp_first = paths[0][0]
    if wp_first is None:
        return 0.0
    diffs = np.diff(wp_first, axis=1)
    return float(np.sqrt((diffs**2).sum(axis=0)).sum())
