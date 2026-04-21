"""Quadrotor vehicle profiles — physical parameters for 6DoF simulation.

Each profile captures mass, inertia, arm geometry, and rotor characteristics
for a specific airframe.  Use :func:`quadrotor_profile` to retrieve a preset
by name, or construct a custom :class:`QuadRotorProfile` directly.

Preset sources
--------------
- **generic**: typical 1.5 kg research quad (similar to DJI F450 class).
- **dji_matrice_100**: DJI Matrice 100 developer drone (~3.6 kg MTOW).
  Inertia from Shraim et al. *Mechanical Systems and Signal Processing* 2018.
- **crazyflie**: Bitcraze Crazyflie 2.x nano-quad (~33 g).
  Parameters from Förster 2015, ETH Zurich.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class QuadRotorProfile:
    """Physical parameters of an X-configuration quadrotor.

    Attributes
    ----------
    name : str
        Human-readable identifier.
    mass : float
        Total mass [kg].
    inertia : np.ndarray
        3×3 inertia tensor in body frame [kg·m²].
    arm_length : float
        Centre-to-motor distance [m].
    k_thrust : float
        Thrust coefficient: ``T_i = k_thrust * omega_i²`` [N/(rad/s)²].
    k_torque : float
        Torque coefficient: ``Q_i = k_torque * omega_i²`` [N·m/(rad/s)²].
    max_rotor_speed : float
        Maximum rotor angular speed [rad/s].
    gravity : float
        Gravitational acceleration [m/s²].
    """

    name: str = "generic"
    mass: float = 1.5
    inertia: np.ndarray = field(
        default_factory=lambda: np.diag([0.0232, 0.0232, 0.0468])
    )
    arm_length: float = 0.25
    k_thrust: float = 1.0e-5
    k_torque: float = 1.2e-7
    max_rotor_speed: float = 1100.0
    gravity: float = 9.81

    # -- derived helpers -------------------------------------------------------

    @property
    def weight(self) -> float:
        """Total weight force [N]."""
        return self.mass * self.gravity

    @property
    def hover_thrust(self) -> float:
        """Total thrust required to hover [N]."""
        return self.weight

    @property
    def hover_rotor_speed(self) -> float:
        """Per-rotor angular speed for hover [rad/s]."""
        return float(np.sqrt(self.weight / (4.0 * self.k_thrust)))


# ---------------------------------------------------------------------------
# Presets
# ---------------------------------------------------------------------------

_PROFILES: dict[str, QuadRotorProfile] = {
    "generic": QuadRotorProfile(
        name="generic",
        mass=1.5,
        inertia=np.diag([0.0232, 0.0232, 0.0468]),
        arm_length=0.25,
        k_thrust=1.0e-5,
        k_torque=1.2e-7,
        max_rotor_speed=1100.0,
    ),
    "dji_matrice_100": QuadRotorProfile(
        name="dji_matrice_100",
        mass=3.6,
        inertia=np.diag([0.0820, 0.0820, 0.1490]),
        arm_length=0.365,
        k_thrust=1.5e-5,
        k_torque=1.8e-7,
        max_rotor_speed=1050.0,
    ),
    "crazyflie": QuadRotorProfile(
        name="crazyflie",
        mass=0.033,
        inertia=np.diag([1.43e-5, 1.43e-5, 2.89e-5]),
        arm_length=0.046,
        k_thrust=2.13e-11,
        k_torque=1.03e-13,
        max_rotor_speed=21_000.0,
    ),
}


def quadrotor_profile(name: str) -> QuadRotorProfile:
    """Retrieve a preset quadrotor profile by name.

    Available presets: ``generic``, ``dji_matrice_100``, ``crazyflie``.

    Raises
    ------
    KeyError
        If the name is not a known preset.
    """
    try:
        return _PROFILES[name]
    except KeyError:
        valid = ", ".join(sorted(_PROFILES))
        raise KeyError(f"Unknown quadrotor profile {name!r}. Available: {valid}") from None


def available_profiles() -> list[str]:
    """Return sorted list of preset profile names."""
    return sorted(_PROFILES)
