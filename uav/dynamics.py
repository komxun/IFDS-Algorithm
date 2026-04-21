"""UAV dynamics models.

A small ``Dynamics`` Protocol lets planners operate on an arbitrary state vector.

Models
------
- ``Kinematic3DoF`` — constant-speed point-mass (matches ``CCA3D_straight.m``).
- ``SixDoF`` — rigid-body quadrotor 6-DoF with quaternion attitude.

Quaternion convention: **scalar-first** ``[qw, qx, qy, qz]``, stored at
state indices 6–9.

State layout (``SixDoF``, length 13)::

    [x, y, z, u, v, w, qw, qx, qy, qz, p, q, r]
     0  1  2  3  4  5   6   7   8   9  10 11 12

- ``(x, y, z)`` — inertial position (NED or ENU, user's choice).
- ``(u, v, w)`` — body-frame linear velocity.
- ``(qw, qx, qy, qz)`` — attitude quaternion (body→inertial).
- ``(p, q, r)`` — body angular rates [rad/s].

Control vector (length 4): ``[T, tau_x, tau_y, tau_z]``
  Total thrust along body-z [N] and body torques [N·m].
  Use :class:`QuadMixer` to convert per-rotor thrusts to this form.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Protocol

import numpy as np
from scipy.integrate import solve_ivp

if TYPE_CHECKING:
    from uav.profiles import QuadRotorProfile

# ---------------------------------------------------------------------------
# Quaternion helpers (scalar-first: [qw, qx, qy, qz])
# ---------------------------------------------------------------------------

def quat_mult(q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Hamilton product of two scalar-first quaternions."""
    qw, qx, qy, qz = q
    rw, rx, ry, rz = r
    return np.array([
        qw * rw - qx * rx - qy * ry - qz * rz,
        qw * rx + qx * rw + qy * rz - qz * ry,
        qw * ry - qx * rz + qy * rw + qz * rx,
        qw * rz + qx * ry - qy * rx + qz * rw,
    ])


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """Return the 3×3 rotation matrix (body→inertial) for a unit quaternion."""
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)],
    ])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Return a normalized copy of a quaternion."""
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])


def quat_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX Euler angles [rad] → scalar-first quaternion."""
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def quat_to_euler(q: np.ndarray) -> tuple[float, float, float]:
    """Scalar-first quaternion → (roll, pitch, yaw) in radians."""
    qw, qx, qy, qz = q
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx**2 + qy**2)
    roll = float(np.arctan2(sinr_cosp, cosr_cosp))

    sinp = 2 * (qw * qy - qz * qx)
    sinp = float(np.clip(sinp, -1.0, 1.0))
    pitch = float(np.arcsin(sinp))

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy**2 + qz**2)
    yaw = float(np.arctan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# State index constants
# ---------------------------------------------------------------------------

IX, IY, IZ = 0, 1, 2
IU, IV, IW = 3, 4, 5
IQW, IQX, IQY, IQZ = 6, 7, 8, 9
IP, IQ, IR = 10, 11, 12
STATE_SIZE = 13


# ---------------------------------------------------------------------------
# Dynamics Protocol
# ---------------------------------------------------------------------------

@dataclass
class Kinematic3DoFState:
    """State vector for the 3-DoF kinematic UAV model."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    psi: float = 0.0
    gamma: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z, self.psi, self.gamma])


class Dynamics(Protocol):
    """Minimal dynamics interface."""

    def step(self, state: np.ndarray, ctrl: np.ndarray, dt: float) -> np.ndarray:
        """Advance the state one time step and return the new state."""
        ...


# ---------------------------------------------------------------------------
# Kinematic 3-DoF
# ---------------------------------------------------------------------------

@dataclass
class Kinematic3DoF:
    """Constant-speed kinematic model: state=[x,y,z,psi,gamma], ctrl=[u1,u2]."""

    va: float = 10.0

    def step(self, state: np.ndarray, ctrl: np.ndarray, dt: float) -> np.ndarray:
        x, y, z, psi, gamma = state
        u1, u2 = ctrl
        dx = self.va * np.cos(gamma) * np.cos(psi)
        dy = self.va * np.cos(gamma) * np.sin(psi)
        dz = self.va * np.sin(gamma)
        dpsi = u1 / (self.va * np.cos(gamma))
        dgam = u2 / self.va
        return np.array([x + dx * dt, y + dy * dt, z + dz * dt,
                         psi + dpsi * dt, gamma + dgam * dt])


# ---------------------------------------------------------------------------
# Quad Mixer
# ---------------------------------------------------------------------------

@dataclass
class QuadMixer:
    """X-configuration quadrotor mixer: per-rotor thrusts → (T, τx, τy, τz).

    Rotor layout (looking down, X-config)::

        1(CW)   2(CCW)
            \\ /
             X
            / \\
        3(CCW)  4(CW)

    Parameters
    ----------
    arm_length : float
        Centre-to-motor distance [m].
    k_thrust : float
        Thrust coefficient ``T_i = k_thrust * ω_i²``.
    k_torque : float
        Torque coefficient ``Q_i = k_torque * ω_i²``.
    """

    arm_length: float = 0.25
    k_thrust: float = 1.0e-5
    k_torque: float = 1.2e-7

    @property
    def mixing_matrix(self) -> np.ndarray:
        """4×4 matrix: ``[T, τx, τy, τz]ᵀ = M @ [f1, f2, f3, f4]ᵀ``.

        Each ``f_i = k_thrust * ω_i²`` is the scalar thrust of rotor *i*.
        """
        L = self.arm_length * np.sqrt(2) / 2  # moment arm for X-config
        c = self.k_torque / self.k_thrust      # torque-to-thrust ratio
        return np.array([
            [1.0,  1.0,  1.0,  1.0],
            [-L,   L,    L,   -L  ],
            [-L,  -L,    L,    L  ],
            [-c,   c,   -c,    c  ],
        ])

    def mix(self, rotor_thrusts: np.ndarray) -> np.ndarray:
        """Convert 4 rotor thrusts to ``[T, τx, τy, τz]``."""
        return self.mixing_matrix @ rotor_thrusts

    def unmix(self, wrench: np.ndarray) -> np.ndarray:
        """Inverse: ``[T, τx, τy, τz]`` → 4 rotor thrusts (may go negative)."""
        return np.linalg.solve(self.mixing_matrix, wrench)

    @classmethod
    def from_profile(cls, profile: "QuadRotorProfile") -> "QuadMixer":
        """Create a mixer from a :class:`~uav.profiles.QuadRotorProfile`."""
        return cls(
            arm_length=profile.arm_length,
            k_thrust=profile.k_thrust,
            k_torque=profile.k_torque,
        )


# ---------------------------------------------------------------------------
# 6-DoF Rigid-Body Quadrotor
# ---------------------------------------------------------------------------

@dataclass
class SixDoF:
    """Rigid-body 6-DoF quadrotor dynamics.

    State layout (13)::

        [x, y, z, u, v, w, qw, qx, qy, qz, p, q, r]

    - ``(x, y, z)`` — inertial position.
    - ``(u, v, w)`` — body-frame linear velocity.
    - ``(qw, qx, qy, qz)`` — attitude quaternion (body→inertial, scalar-first).
    - ``(p, q, r)`` — body angular rates [rad/s].

    Control (4): ``[T, τx, τy, τz]`` — total thrust along body z-axis [N]
    and body-frame torques [N·m].

    Parameters
    ----------
    mass : float
    inertia : np.ndarray
        3×3 body-frame inertia tensor.
    gravity : float
    """

    mass: float = 1.5
    inertia: np.ndarray = field(default_factory=lambda: np.diag([0.0232, 0.0232, 0.0468]))
    gravity: float = 9.81

    @classmethod
    def from_profile(cls, profile: "QuadRotorProfile") -> "SixDoF":
        """Create from a :class:`~uav.profiles.QuadRotorProfile`."""
        return cls(mass=profile.mass, inertia=profile.inertia.copy(), gravity=profile.gravity)

    # -- internal ODE ----------------------------------------------------------

    def _deriv(self, _t: float, s: np.ndarray, ctrl: np.ndarray) -> np.ndarray:
        """Continuous-time derivative ``ds/dt``."""
        u, v, w = s[IU], s[IV], s[IW]
        q = s[IQW:IQZ + 1]
        p_rate, q_rate, r_rate = s[IP], s[IQ], s[IR]
        omega = np.array([p_rate, q_rate, r_rate])

        T = ctrl[0]
        tau = ctrl[1:4]

        R = quat_to_rotmat(q)  # body → inertial
        I = self.inertia
        m = self.mass
        g = self.gravity

        # Translational: m * (dv_body/dt + ω × v_body) = R^T @ [0,0,-mg] + [0,0,T]
        gravity_body = R.T @ np.array([0.0, 0.0, -m * g])
        thrust_body = np.array([0.0, 0.0, T])
        vel_body = np.array([u, v, w])
        dvel = (gravity_body + thrust_body) / m - np.cross(omega, vel_body)

        # Rotational: I * dω/dt = τ - ω × (I @ ω)
        domega = np.linalg.solve(I, tau - np.cross(omega, I @ omega))

        # Quaternion kinematics: dq/dt = 0.5 * q ⊗ [0, p, q, r]
        omega_quat = np.array([0.0, p_rate, q_rate, r_rate])
        dq = 0.5 * quat_mult(q, omega_quat)

        # Position kinematics: dp/dt = R @ v_body
        dpos = R @ vel_body

        ds = np.zeros(STATE_SIZE)
        ds[IX:IZ + 1] = dpos
        ds[IU:IW + 1] = dvel
        ds[IQW:IQZ + 1] = dq
        ds[IP:IR + 1] = domega
        return ds

    # -- Dynamics Protocol -----------------------------------------------------

    def step(self, state: np.ndarray, ctrl: np.ndarray, dt: float) -> np.ndarray:
        """Advance the state by ``dt`` using RK45 integration.

        Parameters
        ----------
        state : np.ndarray, shape (13,)
        ctrl : np.ndarray, shape (4,) — ``[T, τx, τy, τz]``
        dt : float — time step [s].
        """
        sol = solve_ivp(
            self._deriv, [0, dt], state, args=(ctrl,),
            method="RK45", rtol=1e-8, atol=1e-10,
            dense_output=False,
        )
        s_new = sol.y[:, -1].copy()
        # Re-normalize quaternion to prevent drift.
        s_new[IQW:IQZ + 1] = quat_normalize(s_new[IQW:IQZ + 1])
        return s_new

    # -- Convenience -----------------------------------------------------------

    @staticmethod
    def make_state(
        x: float = 0.0, y: float = 0.0, z: float = 0.0,
        u: float = 0.0, v: float = 0.0, w: float = 0.0,
        roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
        p: float = 0.0, q: float = 0.0, r: float = 0.0,
    ) -> np.ndarray:
        """Build a 13-element state vector from human-readable values."""
        quat = quat_from_euler(roll, pitch, yaw)
        return np.array([x, y, z, u, v, w, *quat, p, q, r])

    def hover_control(self) -> np.ndarray:
        """Return the control vector for steady hover (zero torque)."""
        return np.array([self.mass * self.gravity, 0.0, 0.0, 0.0])
