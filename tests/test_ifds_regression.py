"""Regression tests against the reference MATLAB trajectory files.

Runs the default scene-3 static configuration (no optimizer, no weather) and
checks that the produced trajectory length is within a tolerance of the stored
``allTraj_opt.mat`` / ``allTraj_opt2.mat`` references. The stored references
come from MATLAB so we allow a relatively wide tolerance (algorithmic
differences between SciPy SLSQP and MATLAB interior-point, plus floating-point
drift, usually stay below ~5%).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from scipy.io import loadmat

from ifds.config import OptimizerMode, Param, Scene, SimMode, cca_preset
from scripts.run_main import run

REPO_ROOT = Path(__file__).resolve().parents[1]


def _traj_length(traj: np.ndarray) -> float:
    diffs = np.diff(traj, axis=1)
    return float(np.sqrt((diffs**2).sum(axis=0)).sum())


@pytest.mark.parametrize("ref_name,tol", [("allTraj_opt.mat", 0.5), ("allTraj_opt2.mat", 0.5)])
def test_trajectory_length_within_tolerance(ref_name: str, tol: float) -> None:
    ref_path = REPO_ROOT / "data" / ref_name
    if not ref_path.exists():
        pytest.skip(f"reference file {ref_name} missing")
    ref = np.asarray(loadmat(str(ref_path))["allTraj"], dtype=float)
    ref_len = _traj_length(ref)

    param = Param(
        tsim=100, rtsim=50, dt=0.1, sim_mode=SimMode.BY_DISTANCE,
        scene=Scene.THREE_OBJECTS, multi_target=False, sf=False,
        rho0_initial=1.0, sigma0_initial=1.0,
        use_optimizer=OptimizerMode.OFF, rg=10.0,
        k=0.0, env="static",
        xini=0.0, yini=0.0, zini=0.0,
        xfinal=200.0, yfinal=0.0, zfinal=0.0,
        x_i=0.0, y_i=-20.0, z_i=5.0,
        cca=cca_preset(5), show_disp=False,
    )
    result = run(param, plot=False)
    segments = [t for t in result["traj"] if t is not None and t.size]
    assert segments, "no trajectory generated"
    full = np.concatenate(segments, axis=1)
    our_len = _traj_length(full)

    rel = abs(our_len - ref_len) / ref_len
    assert rel <= tol, f"length mismatch: ours={our_len:.2f} ref={ref_len:.2f} rel={rel:.3f}"


def test_scene3_static_produces_nonempty_trajectory() -> None:
    param = Param(
        tsim=100, rtsim=10, dt=0.1, sim_mode=SimMode.BY_DISTANCE,
        scene=Scene.THREE_OBJECTS, multi_target=False, sf=False,
        rho0_initial=1.0, sigma0_initial=1.0,
        use_optimizer=OptimizerMode.OFF, rg=10.0,
        k=0.0, env="static",
        xini=0.0, yini=0.0, zini=0.0,
        xfinal=200.0, yfinal=0.0, zfinal=0.0,
        x_i=0.0, y_i=-20.0, z_i=5.0,
        cca=cca_preset(5), show_disp=False,
    )
    result = run(param, plot=False)
    segments = [t for t in result["traj"] if t is not None and t.size]
    assert segments, "UAV produced no trajectory segments"
