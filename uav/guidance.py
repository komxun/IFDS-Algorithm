"""UAV path-following guidance laws.

Contains:

- **CCA3D** — Carrot-Chasing Algorithm for kinematic 3-DoF (port of ``CCA3D_straight.m``).
- **L1 guidance** — lateral/vertical acceleration commands for 6-DoF quadrotors.
  Computes a reference point on the path segment at look-ahead distance *L1* and
  derives the required lateral acceleration to intercept it (Park, Deyst, How 2004;
  widely used in PX4/ArduPilot).
- **PD attitude controller** — converts desired accelerations into quadrotor
  ``[T, τx, τy, τz]`` commands suitable for :class:`~uav.dynamics.SixDoF`.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from uav.dynamics import (
    IQW, IQX, IQY, IQZ, IP, IQ, IR, IU, IV, IW, IX, IY, IZ,
    SixDoF,
    quat_to_euler,
    quat_to_rotmat,
)


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


# ---------------------------------------------------------------------------
# L1 Guidance (Pure Pursuit with lateral-acceleration output)
# ---------------------------------------------------------------------------

@dataclass
class L1Params:
    """Tuning knobs for L1 path-following guidance.

    Attributes
    ----------
    L1 : float
        Look-ahead distance [m].  Larger → smoother tracking, more overshoot.
    damping : float
        Damping ratio for the lateral acceleration law (0.7 is a good default).
    """

    L1: float = 15.0
    damping: float = 0.7


def l1_guidance(
    pos: np.ndarray,
    vel: np.ndarray,
    wi: np.ndarray,
    wf: np.ndarray,
    params: L1Params | None = None,
) -> np.ndarray:
    """Compute desired inertial acceleration via L1 navigation guidance.

    Parameters
    ----------
    pos : (3,) current inertial position.
    vel : (3,) current inertial velocity.
    wi, wf : (3,) start / end of the current path segment.
    params : L1 tuning (uses defaults if ``None``).

    Returns
    -------
    a_cmd : (3,) desired inertial acceleration [m/s²].
    """
    if params is None:
        params = L1Params()
    L1 = params.L1
    zeta = params.damping

    seg = wf - wi
    seg_len = float(np.linalg.norm(seg))
    if seg_len < 1e-9:
        return np.zeros(3)
    seg_hat = seg / seg_len

    # Project current position onto the segment to find the closest point,
    # then advance by L1 along the segment to get the reference point.
    t_proj = float(np.dot(pos - wi, seg_hat))
    t_ref = np.clip(t_proj + L1, 0.0, seg_len)
    ref = wi + t_ref * seg_hat

    # Lateral error vector (perpendicular to velocity direction).
    err = ref - pos
    speed = float(np.linalg.norm(vel))
    if speed < 1e-6:
        # Nearly stationary — just point toward the reference.
        return 2.0 * zeta * err

    vel_hat = vel / speed
    # L1 acceleration: a = 2 * V² / L1 * sin(eta), where eta is the angle
    # between velocity and the line-of-sight to the reference.
    los = ref - pos
    los_norm = float(np.linalg.norm(los))
    if los_norm < 1e-9:
        return np.zeros(3)
    sin_eta = float(np.linalg.norm(np.cross(vel_hat, los / los_norm)))
    a_mag = 2.0 * speed**2 / max(L1, 1e-3) * sin_eta

    # Direction: perpendicular to velocity, toward reference.
    lateral = err - np.dot(err, vel_hat) * vel_hat
    lat_norm = float(np.linalg.norm(lateral))
    if lat_norm < 1e-9:
        return np.zeros(3)
    a_cmd = a_mag * lateral / lat_norm

    # Add an along-track altitude correction.
    z_err = ref[2] - pos[2]
    a_cmd[2] += 2.0 * zeta * z_err

    return a_cmd


# ---------------------------------------------------------------------------
# PD Attitude Controller (acceleration → quadrotor wrench)
# ---------------------------------------------------------------------------

@dataclass
class AttitudePDGains:
    """PD gains for the inner-loop attitude controller.

    Attributes
    ----------
    kp_att : float
        Proportional gain for roll/pitch angle error → torque.
    kd_att : float
        Derivative gain for angular rate damping.
    kp_yaw : float
        Proportional gain for yaw angle error.
    kd_yaw : float
        Derivative gain for yaw rate.
    kp_alt : float
        Proportional gain for thrust adjustment on altitude error.
    """

    kp_att: float = 8.0
    kd_att: float = 2.5
    kp_yaw: float = 4.0
    kd_yaw: float = 1.0
    kp_alt: float = 5.0


def attitude_pd_control(
    state: np.ndarray,
    a_des: np.ndarray,
    yaw_des: float,
    dyn: SixDoF,
    gains: AttitudePDGains | None = None,
) -> np.ndarray:
    """Convert a desired inertial acceleration into ``[T, τx, τy, τz]``.

    The controller:

    1. Computes the required total thrust to achieve ``a_des + g``.
    2. Derives the desired roll/pitch from the thrust direction.
    3. Applies PD control on attitude error → body torques.

    Parameters
    ----------
    state : (13,) SixDoF state vector.
    a_des : (3,) desired inertial acceleration [m/s²].
    yaw_des : desired yaw angle [rad].
    dyn : SixDoF dynamics instance (for mass/gravity).
    gains : PD tuning (uses defaults if ``None``).

    Returns
    -------
    ctrl : (4,) ``[T, τx, τy, τz]``.
    """
    if gains is None:
        gains = AttitudePDGains()

    m = dyn.mass
    g = dyn.gravity
    q = state[IQW:IQZ + 1]
    omega = state[IP:IR + 1]
    R = quat_to_rotmat(q)

    # Desired thrust vector in inertial frame: F_des = m * (a_des + [0,0,g])
    F_des = m * (a_des + np.array([0.0, 0.0, g]))
    T = float(np.dot(F_des, R[:, 2]))  # project onto body z-axis
    T = max(T, 0.0)  # thrust can't be negative

    # Desired body z-axis (normalized thrust direction).
    F_norm = float(np.linalg.norm(F_des))
    if F_norm < 1e-9:
        z_des = np.array([0.0, 0.0, 1.0])
    else:
        z_des = F_des / F_norm

    # Desired yaw → x-axis direction in the horizontal plane.
    x_c = np.array([np.cos(yaw_des), np.sin(yaw_des), 0.0])
    y_des = np.cross(z_des, x_c)
    y_norm = float(np.linalg.norm(y_des))
    if y_norm < 1e-9:
        y_des = np.array([0.0, 1.0, 0.0])
    else:
        y_des /= y_norm
    x_des = np.cross(y_des, z_des)
    R_des = np.column_stack([x_des, y_des, z_des])

    # Attitude error (SO(3) → so(3) via skew-symmetric).
    R_err = R_des.T @ R - R.T @ R_des
    e_att = 0.5 * np.array([R_err[2, 1], R_err[0, 2], R_err[1, 0]])

    # PD torques.
    tau_x = -gains.kp_att * e_att[0] - gains.kd_att * omega[0]
    tau_y = -gains.kp_att * e_att[1] - gains.kd_att * omega[1]
    tau_z = -gains.kp_yaw * e_att[2] - gains.kd_yaw * omega[2]

    return np.array([T, tau_x, tau_y, tau_z])


# ---------------------------------------------------------------------------
# 6-DoF segment follower (L1 + PD, mirrors cca3d_straight for SixDoF)
# ---------------------------------------------------------------------------

@dataclass
class SixDoFResult:
    """Result of a 6-DoF segment-following run."""

    states: np.ndarray   # (N, 13) — full state history
    time_spent: float

    @property
    def x(self) -> np.ndarray:
        return self.states[:, IX]

    @property
    def y(self) -> np.ndarray:
        return self.states[:, IY]

    @property
    def z(self) -> np.ndarray:
        return self.states[:, IZ]

    @property
    def quat(self) -> np.ndarray:
        """(N, 4) quaternion history [qw, qx, qy, qz]."""
        return self.states[:, IQW:IQZ + 1]


def sixdof_follow_segment(
    wi: np.ndarray,
    wf: np.ndarray,
    state0: np.ndarray,
    dyn: SixDoF,
    *,
    l1_params: L1Params | None = None,
    att_gains: AttitudePDGains | None = None,
    dt: float = 0.01,
    max_iter: int = 10_000,
) -> SixDoFResult:
    """Follow a straight-line segment ``wi→wf`` with a 6-DoF quadrotor.

    Combines :func:`l1_guidance` (outer loop) with :func:`attitude_pd_control`
    (inner loop) and integrates via :meth:`SixDoF.step`.

    Stopping condition: the UAV crosses the plane through ``wf`` perpendicular
    to the segment (same as CCA3D).

    Parameters
    ----------
    wi, wf : (3,) start and end waypoints.
    state0 : (13,) initial SixDoF state.
    dyn : SixDoF dynamics instance.
    l1_params : L1 tuning.
    att_gains : PD attitude tuning.
    dt : integration time step [s].
    max_iter : safety iteration cap.
    """
    seg = wf - wi
    a, b, c = seg
    ox, oy, oz = wf

    states = [state0.copy()]
    time_spent = 0.0

    yaw_des = float(np.arctan2(seg[1], seg[0]))

    for _ in range(max_iter):
        s = states[-1]
        xi, yi, zi = s[IX], s[IY], s[IZ]

        # Stopping condition (same as CCA3D).
        if a * (xi - ox) + b * (yi - oy) + c * (zi - oz) >= 0:
            break

        pos = s[IX:IZ + 1]
        R = quat_to_rotmat(s[IQW:IQZ + 1])
        vel_body = s[IU:IW + 1]
        vel_inertial = R @ vel_body

        a_des = l1_guidance(pos, vel_inertial, wi, wf, l1_params)
        ctrl = attitude_pd_control(s, a_des, yaw_des, dyn, att_gains)
        s_new = dyn.step(s, ctrl, dt)
        states.append(s_new)
        time_spent += dt

    return SixDoFResult(states=np.array(states), time_spent=time_spent)
