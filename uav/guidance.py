"""Carrot-Chasing Algorithm (CCA) 3D guidance for straight-line segments.

Direct port of ``CCA3D_straight.m``.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class CCAResult:
    """Result of a single CCA segment run."""

    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    psi: np.ndarray
    gamma: np.ndarray
    time_spent: float


def cca3d_straight(
    wi: np.ndarray,
    wf: np.ndarray,
    x0: float,
    y0: float,
    z0: float,
    psi0: float,
    gamma0: float,
    va: float,
    kappa: float,
    delta: float,
    kd: float,
    dt: float = 0.01,
    r_min: float = 10.0,
    max_iter: int = 10_000,
) -> CCAResult:
    """Follow a straight-line segment from ``wi`` to ``wf`` using CCA.

    Args:
        wi, wf: start and end waypoints of the segment (3-vectors).
        x0, y0, z0: current UAV position.
        psi0, gamma0: current yaw and pitch [rad].
        va: UAV speed [m/s].
        kappa, delta, kd: CCA gains.
        dt: integration time step [s].
        r_min: minimum turn radius [m] (limits lateral accel).
        max_iter: safety cap on integration steps.
    """
    umax = va**2 / r_min

    x = [x0]
    y = [y0]
    z = [z0]
    psi = [psi0]
    gamma = [gamma0]

    rw_vect = wf - wi
    a, b, c = rw_vect
    ox, oy, oz = wf

    del_psi_prev = 0.0
    del_gam_prev = 0.0
    time_spent = 0.0

    for i in range(max_iter):
        xi, yi, zi = x[-1], y[-1], z[-1]
        # Stopping: UAV has crossed the plane through wf perpendicular to (wf-wi).
        if a * (xi - ox) + b * (yi - oy) + c * (zi - oz) >= 0:
            break

        # Step 1-4 : carrot geometry
        pi = np.array([xi, yi, zi])
        ru_vect = wi - pi
        ru = float(np.linalg.norm(ru_vect))

        theta1 = np.arctan2(wf[1] - wi[1], wf[0] - wi[0])
        theta2 = np.arctan2(wf[2] - wi[2], np.hypot(wf[0] - wi[0], wf[1] - wi[1]))

        if np.linalg.norm(ru_vect) != 0 and np.linalg.norm(rw_vect) != 0:
            alpha = np.real(
                np.arccos(
                    np.clip(
                        np.dot(ru_vect, rw_vect)
                        / (np.linalg.norm(ru_vect) * np.linalg.norm(rw_vect)),
                        -1.0, 1.0,
                    )
                )
            )
        else:
            alpha = 0.0
        R = np.sqrt(max(ru**2 - (ru * np.sin(alpha)) ** 2, 0.0))

        # Step 5: carrot position
        xt = wi[0] + (R + delta) * np.cos(theta2) * np.cos(theta1)
        yt = wi[1] + (R + delta) * np.cos(theta2) * np.sin(theta1)
        zt = wi[2] + (R + delta) * np.sin(theta2)

        # Step 6: desired yaw/pitch, wrapped and limited to [-pi/2, pi/2]
        psi_d = np.arctan2(yt - pi[1], xt - pi[0]) % (2 * np.pi)
        gamma_d = np.arctan2(zt - pi[2], np.hypot(xt - pi[0], yt - pi[1])) % (2 * np.pi)
        if psi_d > np.pi:
            psi_d -= 2 * np.pi
        if gamma_d > np.pi:
            gamma_d -= 2 * np.pi
        psi_d = np.clip(psi_d, -np.pi / 2, np.pi / 2)
        gamma_d = np.clip(gamma_d, -np.pi / 2, np.pi / 2)

        # Step 7: PID-ish command
        del_psi = psi_d - psi[-1]
        u1 = (kappa * del_psi + kd * (del_psi - del_psi_prev) / dt) * va
        del_gam = gamma_d - gamma[-1]
        u2 = (kappa * del_gam + kd * (del_gam - del_gam_prev) / dt) * va
        u1 = float(np.clip(u1, -umax, umax))
        u2 = float(np.clip(u2, -umax, umax))
        del_psi_prev, del_gam_prev = del_psi, del_gam

        # Kinematic integration
        dx = va * np.cos(gamma[-1]) * np.cos(psi[-1])
        dy = va * np.cos(gamma[-1]) * np.sin(psi[-1])
        dz = va * np.sin(gamma[-1])
        dpsi = u1 / (va * np.cos(gamma[-1]))
        dgam = u2 / va

        x.append(x[-1] + dx * dt)
        y.append(y[-1] + dy * dt)
        z.append(z[-1] + dz * dt)
        psi.append(psi[-1] + dpsi * dt)
        gamma.append(gamma[-1] + dgam * dt)
        time_spent += dt

    return CCAResult(
        x=np.asarray(x), y=np.asarray(y), z=np.asarray(z),
        psi=np.asarray(psi), gamma=np.asarray(gamma), time_spent=time_spent,
    )
