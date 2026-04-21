"""UAV marker drawing for 3D matplotlib axes.

Two styles:

- :func:`draw_quadrotor` — wireframe X-config quadrotor, oriented by a
  quaternion. Used when ``--dynamics sixdof``.
- :func:`draw_arrow` — simple 3D arrow along the heading direction, oriented
  by ``(psi, gamma)``. Used when ``--dynamics kinematic``.
"""

from __future__ import annotations

import numpy as np

from uav.dynamics import quat_to_rotmat


# ---------------------------------------------------------------------------
# Quadrotor wireframe
# ---------------------------------------------------------------------------

def _rotor_circle(center: np.ndarray, R: np.ndarray, radius: float,
                  n_pts: int = 16) -> np.ndarray:
    """Return (3, n_pts+1) points for a circle in the body-XY plane at *center*."""
    theta = np.linspace(0, 2 * np.pi, n_pts + 1)
    circle_body = np.zeros((3, n_pts + 1))
    circle_body[0] = radius * np.cos(theta)
    circle_body[1] = radius * np.sin(theta)
    # Rotate into inertial and translate.
    return R @ circle_body + center[:, np.newaxis]


def draw_quadrotor(
    ax,
    pos: np.ndarray,
    quat: np.ndarray,
    arm_length: float = 5.0,
    rotor_radius: float = 2.0,
    color: str = "tab:blue",
    linewidth: float = 2.0,
) -> list:
    """Draw a wireframe X-config quadrotor on a 3D axis.

    Parameters
    ----------
    ax : mpl_toolkits.mplot3d.axes3d.Axes3D
    pos : (3,) inertial position.
    quat : (4,) scalar-first quaternion [qw, qx, qy, qz].
    arm_length : visual arm length [plot units] (not physical metres).
    rotor_radius : visual rotor circle radius [plot units].
    color : line colour.
    linewidth : line width.

    Returns
    -------
    artists : list of matplotlib artist handles (for removal in animations).
    """
    R = quat_to_rotmat(quat)
    L = arm_length * np.sqrt(2) / 2  # half-arm in X/Y
    artists: list = []

    # Four motor positions in body frame (X-config).
    tips_body = np.array([
        [-L, -L, 0],
        [ L, -L, 0],
        [ L,  L, 0],
        [-L,  L, 0],
    ]).T  # (3, 4)

    tips_inertial = R @ tips_body + pos[:, np.newaxis]

    # Draw two cross arms.
    for i, j in [(0, 2), (1, 3)]:
        h, = ax.plot(
            [tips_inertial[0, i], tips_inertial[0, j]],
            [tips_inertial[1, i], tips_inertial[1, j]],
            [tips_inertial[2, i], tips_inertial[2, j]],
            color=color, linewidth=linewidth,
        )
        artists.append(h)

    # Draw rotor circles at each tip.
    for k in range(4):
        circ = _rotor_circle(tips_inertial[:, k], R, rotor_radius)
        h, = ax.plot(circ[0], circ[1], circ[2], color=color, linewidth=linewidth * 0.6)
        artists.append(h)

    # Centre dot.
    h = ax.scatter([pos[0]], [pos[1]], [pos[2]], c=color, s=30, zorder=5)
    artists.append(h)

    return artists


# ---------------------------------------------------------------------------
# Arrow / cone marker (kinematic mode)
# ---------------------------------------------------------------------------

def draw_arrow(
    ax,
    pos: np.ndarray,
    psi: float,
    gamma: float,
    length: float = 8.0,
    color: str = "tab:red",
    linewidth: float = 2.5,
) -> list:
    """Draw a 3D arrow along the heading direction on a 3D axis.

    Parameters
    ----------
    ax : mpl_toolkits.mplot3d.axes3d.Axes3D
    pos : (3,) inertial position.
    psi : yaw angle [rad].
    gamma : pitch angle [rad].
    length : visual arrow length [plot units].
    color : line colour.
    linewidth : line width.

    Returns
    -------
    artists : list of matplotlib artist handles.
    """
    dx = length * np.cos(gamma) * np.cos(psi)
    dy = length * np.cos(gamma) * np.sin(psi)
    dz = length * np.sin(gamma)
    tip = pos + np.array([dx, dy, dz])

    artists: list = []
    # Main shaft.
    h, = ax.plot(
        [pos[0], tip[0]], [pos[1], tip[1]], [pos[2], tip[2]],
        color=color, linewidth=linewidth,
    )
    artists.append(h)

    # Small arrowhead (two short lines angled back).
    head_len = length * 0.3
    for angle_off in [0.4, -0.4]:
        psi_h = psi + np.pi + angle_off
        gamma_h = gamma + 0.15
        hx = head_len * np.cos(gamma_h) * np.cos(psi_h)
        hy = head_len * np.cos(gamma_h) * np.sin(psi_h)
        hz = head_len * np.sin(gamma_h)
        head = tip + np.array([hx, hy, hz])
        h, = ax.plot(
            [tip[0], head[0]], [tip[1], head[1]], [tip[2], head[2]],
            color=color, linewidth=linewidth * 0.8,
        )
        artists.append(h)

    # Base dot.
    h = ax.scatter([pos[0]], [pos[1]], [pos[2]], c=color, s=40, zorder=5)
    artists.append(h)

    return artists
