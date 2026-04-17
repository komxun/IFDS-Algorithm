"""Obstacle object model.

Replaces the MATLAB ``Object(j)`` struct array with a list of ``Object`` dataclasses,
each carrying the implicit-surface parameters (a,b,c,p,q,r), current `Gamma`,
normal `n`, tangent `t`, and cached fields populated by the IFDS inner loop
(`dist`, `M`, `w`, `w_tilde`).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Object:
    """A single implicit-surface obstacle.

    The shape equation is

        Gamma(X,Y,Z) = ((X-x0)/a)^(2p) + ((Y-y0)/b)^(2q) + ((Z-z0)/c)^(2r)

    with the surface defined by ``Gamma == 1``.
    """

    # Geometry
    origin: np.ndarray = field(default_factory=lambda: np.zeros(3))
    a: float = 0.0
    b: float = 0.0
    c: float = 0.0
    p: float = 1.0
    q: float = 1.0
    r: float = 1.0
    rstar: float = 0.0  # safeguard baseline radius = min(a,b,c)
    alpha_deg: float = 0.0

    # Dynamic (updated every IFDS step)
    gamma: float = 0.0
    n: np.ndarray = field(default_factory=lambda: np.zeros(3))
    t: np.ndarray = field(default_factory=lambda: np.zeros(3))
    dist: float = 0.0
    M: np.ndarray = field(default_factory=lambda: np.eye(3))
    w: float = 0.0
    w_tilde: float = 0.0

    def gamma_at(self, x: float, y: float, z: float) -> float:
        """Evaluate the (bare, no-weather) Gamma at an arbitrary point."""
        x0, y0, z0 = self.origin
        return (
            ((x - x0) / self.a) ** (2 * self.p)
            + ((y - y0) / self.b) ** (2 * self.q)
            + ((z - z0) / self.c) ** (2 * self.r)
        )

    def gamma_star_at(self, x: float, y: float, z: float, rg: float) -> float:
        """Safeguard (gap-shifted) Gamma: ``Gamma - ((Rstar+Rg)/Rstar)^2 + 1``."""
        return self.gamma_at(x, y, z) - ((self.rstar + rg) / self.rstar) ** 2 + 1.0
