"""Optimization helpers for (rho0, sigma0).

Ports:
- ``norm_ubar.m`` (the local optimization objective)
- ``path_opt2`` nested function in ``IFDS.m`` (local, per-obstacle, every 5 steps)
- ``path_optimizing`` nested function in ``main.m`` (global, full-path length)

MATLAB's ``fmincon(..., 'interior-point')`` is replaced with
``scipy.optimize.minimize(method='SLSQP')`` which supports bounds and is a common
substitute for a trust-region-style constrained optimizer.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from scipy.optimize import minimize

if TYPE_CHECKING:
    from collections.abc import Callable

    from ifds.objects import Object
    from ifds.params import Param
    from ifds.weather import WeatherField


def norm_ubar(
    rho0: float,
    sigma0: float,
    gamma: float,
    n: np.ndarray,
    t: np.ndarray,
    u: np.ndarray,
    d: float,
    d0: float,
) -> float:
    """Squared norm of the modulated velocity for a single obstacle.

    Port of ``norm_ubar.m`` — used as the local optimization objective.
    """
    rho = rho0 * np.exp(1.0 - 1.0 / (d0 * d))
    sigma = sigma0 * np.exp(1.0 - 1.0 / (d0 * d))
    nn = n.reshape(3, 1)
    tt = t.reshape(3, 1)
    M = (
        np.eye(3)
        - (nn @ nn.T) / (np.abs(gamma) ** (1.0 / rho) * (nn.T @ nn))
        + (tt @ nn.T) / (np.abs(gamma) ** (1.0 / sigma) * np.linalg.norm(tt) * np.linalg.norm(nn))
    )
    v = M @ u.reshape(3, 1)
    return float((v.T @ v).item())


def local_opt_rho_sigma(
    gamma: float,
    n: np.ndarray,
    t: np.ndarray,
    u: np.ndarray,
    d: float,
    d0: float,
    rho0: float,
    sigma0: float,
    bounds: tuple[tuple[float, float], tuple[float, float]] = ((0.05, 2.0), (0.0, 1.0)),
) -> tuple[float, float]:
    """Local (per-obstacle) optimization: port of ``path_opt2``."""
    res = minimize(
        lambda x: norm_ubar(x[0], x[1], gamma, n, t, u, d, d0),
        x0=np.array([rho0, sigma0]),
        method="SLSQP",
        bounds=bounds,
        options={"disp": False},
    )
    return float(res.x[0]), float(res.x[1])


def global_optimize_path(
    objective: "Callable[[float, float], float]",
    rho0: float,
    sigma0: float,
    bounds: tuple[tuple[float, float], tuple[float, float]] = ((0.05, 2.5), (0.0, 2.0)),
    max_iter: int = 1,
) -> tuple[float, float]:
    """Global path optimization: port of ``path_optimizing`` in ``main.m``.

    ``objective(rho0, sigma0)`` must return a scalar path length (or similar cost).
    MATLAB uses ``MaxIterations=1`` for speed; we preserve that default.
    """
    res = minimize(
        lambda x: objective(float(x[0]), float(x[1])),
        x0=np.array([rho0, sigma0]),
        method="SLSQP",
        bounds=bounds,
        options={"disp": False, "maxiter": max_iter},
    )
    return float(res.x[0]), float(res.x[1])


def make_global_objective(
    loc_final: np.ndarray,
    rt: int,
    wp: np.ndarray,
    paths: list[list[np.ndarray | None]],
    param: "Param",
    objects: list["Object"],
    weather: "WeatherField | None",
) -> "Callable[[float, float], float]":
    """Build the objective used by :func:`global_optimize_path`.

    Returns a closure that runs :func:`ifds.ifds.run_ifds` (without plotting, with
    optimization disabled) and returns ``totalLength``.
    """
    import dataclasses

    from ifds.ifds import run_ifds

    def _obj(rho0: float, sigma0: float) -> float:
        p2 = dataclasses.replace(param, show_disp=False)
        p2.use_optimizer = type(param.use_optimizer)(0)  # OFF
        _, _, total_length, _ = run_ifds(
            rho0=rho0, sigma0=sigma0, alpha_deg=0.0, loc_final=loc_final,
            rt=rt, wp=wp.copy(), paths=paths, param=p2, line_index=0,
            objects=[dataclasses.replace(o) for o in objects], weather=weather,
        )
        return total_length

    return _obj
