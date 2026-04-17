"""IFDS core algorithm.

Ports the main routine and the ``calc_ubar`` nested helper from ``IFDS.m``.

Public API:

- :func:`calc_ubar`  – modulated velocity for a single UAV position.
- :func:`ifds_step`  – single-step update applied in the outer time loop.
- :func:`run_ifds`   – full path-generation loop for one outer ``rt`` iteration,
  reproducing ``IFDS.m``'s ``case 1`` (by time) and ``case 2`` (by distance).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from ifds.enums import OptimizerMode, SimMode
from ifds.optimizer import local_opt_rho_sigma
from ifds.params import Param
from ifds.scenes import build_scene

if TYPE_CHECKING:
    from ifds.objects import Object
    from ifds.weather import WeatherField


def _apply_weather_modification(
    objects: list["Object"],
    xx: float,
    yy: float,
    weather: "WeatherField",
    t_idx: int,
    k: float,
    b_l: float,
    b_u: float,
) -> None:
    """Deform each obstacle's Gamma/n/t by the weather field (port of MATLAB block)."""
    omega, dwdx_now, dwdy_now = weather.sample(t_idx, xx, yy)
    for obj in objects:
        gm = obj.gamma
        dgdx, dgdy, dgdz = obj.n
        denom = (b_l - b_u)
        exp_factor = k * np.exp((b_l - omega) / denom * np.log((gm - 1) / k + 1))
        log_term = np.log((gm - 1) / k + 1) / denom
        scale = (b_l - omega) / ((gm - 1 + k) * denom)
        dgx_p = dgdx + exp_factor * (log_term * dwdx_now - scale * dgdx)
        dgy_p = dgdy + exp_factor * (log_term * dwdy_now - scale * dgdy)
        dgz_p = dgdz + exp_factor * (-scale * dgdz)
        obj.gamma = gm - k * (np.exp((b_l - omega) / denom * np.log((gm - 1) / k + 1)) - 1)
        obj.n = np.array([dgx_p, dgy_p, dgz_p])
        obj.t = np.array([dgy_p, -dgx_p, 0.0])


def calc_ubar(
    pos: np.ndarray,
    target: np.ndarray,
    objects: list["Object"],
    rho0: float,
    sigma0: float,
    use_optimizer: OptimizerMode,
    delta_g: float,
    c_speed: float,
    sf: bool,
    time_step: int,
) -> tuple[np.ndarray, float, float, bool]:
    """Compute the modulated velocity ``UBar``.

    Returns ``(ubar, rho0, sigma0, err_flag)``. ``err_flag`` mirrors MATLAB's
    ``errFlag==1`` escape in ``IFDS.m`` (when all obstacles have ``n'u >= 0`` and
    shape-following is disabled, the unmodified goal-seeking ``u`` is returned).
    """
    x, y, z = pos
    xd, yd, zd = target
    dist = float(np.linalg.norm(pos - target))
    u = -c_speed * (pos - target) / dist
    u = u.reshape(3, 1)

    mm = np.zeros((3, 3))
    sum_w = 0.0
    err_flag = False

    for j, obj in enumerate(objects):
        gamma = obj.gamma
        n = obj.n.reshape(3, 1)
        t = obj.t.reshape(3, 1)
        x0, y0, z0 = obj.origin
        dist_obj = float(np.linalg.norm(np.array([x - x0, y - y0, z - z0])))

        ntu = float((n.T @ u).item())
        if ntu < 0 or sf:
            if use_optimizer == OptimizerMode.LOCAL and time_step % 5 == 0:
                rho0, sigma0 = local_opt_rho_sigma(
                    gamma, n.ravel(), t.ravel(), u.ravel(), dist, dist_obj, rho0, sigma0,
                )
            rstar = obj.rstar
            try:
                rho0_star = (
                    np.log(np.abs(gamma))
                    / np.log(np.abs(gamma - ((rstar + delta_g) / rstar) ** 2 + 1))
                    * rho0
                )
            except (ZeroDivisionError, FloatingPointError):
                rho0_star = rho0
            rho = rho0_star * np.exp(1 - 1 / (dist_obj * dist))
            sigma = sigma0 * np.exp(1 - 1 / (dist_obj * dist))
            M = (
                np.eye(3)
                - (n @ n.T) / (np.abs(gamma) ** (1.0 / rho) * (n.T @ n))
                + (t @ n.T) / (np.abs(gamma) ** (1.0 / sigma) * np.linalg.norm(t) * np.linalg.norm(n))
            )
        elif ntu >= 0 and not sf:
            M = np.eye(3)
        else:
            err_flag = True
            return u.ravel(), rho0, sigma0, err_flag

        # Weight (product over other obstacles)
        w = 1.0
        for i, other in enumerate(objects):
            if i == j:
                continue
            w *= (other.gamma - 1) / ((obj.gamma - 1) + (other.gamma - 1))
        sum_w += w

        obj.dist = dist_obj
        obj.M = M
        obj.w = w

    if sum_w == 0:
        return u.ravel(), rho0, sigma0, err_flag

    for obj in objects:
        obj.w_tilde = obj.w / sum_w
        mm = mm + obj.w_tilde * obj.M

    ubar = (mm @ u).ravel()
    return ubar, rho0, sigma0, err_flag


def ifds_step(
    pos: np.ndarray,
    target: np.ndarray,
    objects: list["Object"],
    rho0: float,
    sigma0: float,
    param: Param,
    time_step: int,
) -> tuple[np.ndarray, float, float, bool]:
    """One outer step: returns ``(next_pos, rho0, sigma0, err_flag)``.

    The caller is responsible for scene building and weather modification.
    """
    ubar, rho0, sigma0, err_flag = calc_ubar(
        pos=pos, target=target, objects=objects, rho0=rho0, sigma0=sigma0,
        use_optimizer=param.use_optimizer, delta_g=param.rg,
        c_speed=param.c, sf=param.sf, time_step=time_step,
    )
    next_pos = pos + ubar * param.dt
    return next_pos, rho0, sigma0, err_flag


def run_ifds(
    rho0: float,
    sigma0: float,
    alpha_deg: float,
    loc_final: np.ndarray,
    rt: int,
    wp: np.ndarray,
    paths: list[list[np.ndarray | None]],
    param: Param,
    line_index: int,
    objects: list["Object"],
    weather: "WeatherField | None",
) -> tuple[list[list[np.ndarray | None]], list["Object"], float, bool]:
    """Full IFDS loop for one outer iteration.

    Returns ``(paths, objects, total_length, found_path)``, matching the MATLAB
    ``[Paths, Object, totalLength, foundPath] = IFDS(...)`` signature.
    """
    xd, yd, zd = loc_final
    target = np.array([xd, yd, zd], dtype=float)
    # MATLAB: case 1 (BY_TIME) runs for tsim iterations. Case 2 (BY_DISTANCE)
    # uses `while true` with a break at t>1000. We allocate a buffer big enough
    # for the distance mode and grow `wp` if the caller's buffer is too small.
    max_steps = param.tsim if param.sim_mode == SimMode.BY_TIME else 1001
    if wp.shape[1] < max_steps + 1:
        wp_big = np.zeros((3, max_steps + 1))
        wp_big[:, : wp.shape[1]] = wp
        wp = wp_big
    found_path = False
    err_flag = False
    t = 0  # index of "current" waypoint

    # Use param.static_weather_index under static env (matches main.m)
    weather_t = rt if param.env == "dynamic" else param.static_weather_index

    if param.sim_mode == SimMode.BY_TIME:
        for t in range(max_steps):
            wp[:, t] = np.real(wp[:, t])
            xx, yy, zz = wp[:, t]
            build_scene(param.scene, objects, xx, yy, zz, rt, alpha_deg)

            if np.linalg.norm(np.array([xx, yy, zz]) - target) < param.target_thresh:
                wp_trunc = wp[:, : t + 1]
                paths[line_index][rt] = wp_trunc.copy()
                found_path = True
                break

            if param.k != 0 and weather is not None:
                _apply_weather_modification(objects, xx, yy, weather, weather_t,
                                            param.k, param.b_l, param.b_u)

            ubar, rho0, sigma0, err_flag = calc_ubar(
                pos=wp[:, t], target=target, objects=objects,
                rho0=rho0, sigma0=sigma0, use_optimizer=param.use_optimizer,
                delta_g=param.rg, c_speed=param.c, sf=param.sf, time_step=t + 1,
            )
            if err_flag:
                break
            wp[:, t + 1] = wp[:, t] + ubar * param.dt

        wp_trunc = wp[:, : t + 1]
        paths[line_index][rt] = wp_trunc.copy()
        found_path = not err_flag and found_path

    else:  # BY_DISTANCE
        t = 0
        while True:
            if t >= max_steps - 1:
                break
            wp[:, t] = np.real(wp[:, t])
            xx, yy, zz = wp[:, t]
            build_scene(param.scene, objects, xx, yy, zz, rt, alpha_deg)

            if np.linalg.norm(np.array([xx, yy, zz]) - target) < param.target_thresh:
                wp_trunc = wp[:, : t + 1]
                paths[line_index][rt] = wp_trunc.copy()
                found_path = True
                break

            if param.k != 0 and weather is not None:
                _apply_weather_modification(objects, xx, yy, weather, weather_t,
                                            param.k, param.b_l, param.b_u)

            ubar, rho0, sigma0, err_flag = calc_ubar(
                pos=wp[:, t], target=target, objects=objects,
                rho0=rho0, sigma0=sigma0, use_optimizer=param.use_optimizer,
                delta_g=param.rg, c_speed=param.c, sf=param.sf, time_step=t + 1,
            )
            wp[:, t + 1] = wp[:, t] + ubar * param.dt
            t += 1
        wp_trunc = wp[:, : t + 1]
        paths[line_index][rt] = wp_trunc.copy()

    # Post-calculation: path length
    if found_path and paths[line_index][rt] is not None:
        waypoints = paths[line_index][rt]
        diffs = np.diff(waypoints, axis=1)
        total_length = float(np.sqrt((diffs**2).sum(axis=0)).sum())
        if param.show_disp:
            print(f"Total path length: {total_length:.2f} m")
            print(f"Total flight time: {total_length / param.c:.2f} s")
    else:
        total_length = 0.0

    return paths, objects, total_length, found_path
