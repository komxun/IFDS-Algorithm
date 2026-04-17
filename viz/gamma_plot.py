"""Gamma distribution plot (port of the MATLAB ``PlotGamma`` nested function).

Shows the implicit surface on 3D + three slicing planes (XY top, YZ front, XZ side)
with optional weather-modified Gamma.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import matplotlib.pyplot as plt
import numpy as np

if TYPE_CHECKING:
    from ifds.objects import Object


def _gamma_sum(objects: list["Object"], X: np.ndarray, Y: np.ndarray, Z: np.ndarray) -> np.ndarray:
    """Minimum Gamma across all obstacles (so that Gamma==1 is their union surface)."""
    result = np.full_like(X, np.inf, dtype=float)
    for obj in objects:
        x0, y0, z0 = obj.origin
        g = (
            ((X - x0) / obj.a) ** (2 * obj.p)
            + ((Y - y0) / obj.b) ** (2 * obj.q)
            + ((Z - z0) / obj.c) ** (2 * obj.r)
        )
        result = np.minimum(result, g)
    return result


def plot_gamma_distribution(
    objects: list["Object"],
    weather_mat: np.ndarray | None = None,
    *,
    x_fixed: float = 100.0,
    y_fixed: float = 0.0,
    z_fixed: float = 0.0,
    num_levels: int = 30,
    figsize: tuple[float, float] = (15, 10),
) -> plt.Figure:
    """Port of ``PlotGamma``: 2×3 grid of slicing contours."""
    xr = np.arange(0, 201)
    yr = np.arange(-100, 101)
    zr = np.arange(0, 201)
    Xxy, Yxy = np.meshgrid(xr, yr, indexing="ij")
    Yyz, Zyz = np.meshgrid(yr, zr, indexing="ij")
    Xxz, Zxz = np.meshgrid(xr, zr, indexing="ij")
    G_xy = _gamma_sum(objects, Xxy, Yxy, np.full_like(Xxy, z_fixed))
    G_yz = _gamma_sum(objects, np.full_like(Yyz, x_fixed), Yyz, Zyz)
    G_xz = _gamma_sum(objects, Xxz, np.full_like(Xxz, y_fixed), Zxz)

    fig, axes = plt.subplots(2, 3, figsize=figsize)
    axes[0, 0].set_title("XY (Z=const)")
    axes[0, 0].contourf(Xxy, Yxy, G_xy, num_levels, cmap="turbo_r")
    axes[0, 0].contour(Xxy, Yxy, G_xy, levels=[1], colors="w", linewidths=2)

    if weather_mat is not None:
        axes[0, 1].set_title("Weather")
        xx = np.arange(1, weather_mat.shape[0] + 1)
        yy = np.arange(1, weather_mat.shape[1] + 1)
        XX, YY = np.meshgrid(xx, yy, indexing="ij")
        axes[0, 1].contourf(XX, YY, weather_mat[:, :, 0], num_levels, cmap="turbo")

    axes[0, 2].set_title("Gamma XY modified")
    axes[0, 2].contourf(Xxy, Yxy, G_xy, num_levels, cmap="turbo_r")
    axes[1, 0].set_title("Gamma YZ (X=const)")
    axes[1, 0].contourf(Yyz, Zyz, G_yz, num_levels, cmap="turbo_r")
    axes[1, 1].set_title("Gamma XZ (Y=const)")
    axes[1, 1].contourf(Xxz, Zxz, G_xz, num_levels, cmap="turbo_r")
    axes[1, 2].axis("off")

    for ax in axes.flat:
        ax.set_aspect("equal", adjustable="box")
    fig.tight_layout()
    return fig
