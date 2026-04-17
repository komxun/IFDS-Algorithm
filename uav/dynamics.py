"""UAV dynamics models.

A small ``Dynamics`` Protocol lets planners operate on an arbitrary state vector.
Today we ship a ``Kinematic3DoF`` model (matching ``CCA3D_straight.m``); a stub
``SixDoF`` class is provided as a scaffold for the next phase.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

import numpy as np


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


@dataclass
class SixDoF:
    """Rigid-body 6-DoF placeholder.

    State layout (13): ``[pN, pE, pD, u, v, w, qx, qy, qz, qw, p, q, r]``
    (NED position, body velocity, unit quaternion attitude, body angular rate).
    Control: ``[Fx, Fy, Fz, Mx, My, Mz]`` in body frame.

    Integration will use ``scipy.integrate.solve_ivp`` (DOP853 recommended).
    TODO: populate mass, inertia tensor, aero model.
    """

    mass: float = 1.0
    inertia: np.ndarray = field(default_factory=lambda: np.eye(3))

    def step(self, state: np.ndarray, ctrl: np.ndarray, dt: float) -> np.ndarray:  # pragma: no cover - stub
        raise NotImplementedError(
            "SixDoF is a scaffold; implement EOM + solve_ivp integration "
            "before enabling --dynamics sixdof."
        )
