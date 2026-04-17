"""Plotting helpers.

Replaces:
- ``PlotPath.m``       -> :func:`plot_path_2d`
- ``PlotObject.m``     -> :func:`plot_objects_pyvista` / :func:`plot_objects_mpl`
- ``plotting_everything.m`` -> :func:`plot_scene_mpl`
- weather overlays in ``main.m`` -> :func:`plot_weather_contour`
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import matplotlib.pyplot as plt
import numpy as np

if TYPE_CHECKING:
    from ifds.objects import Object


def _set_equal_aspect_3d(ax) -> None:
    """Force equal scaling on a 3D axis based on its current limits."""
    x_span = ax.get_xlim3d()[1] - ax.get_xlim3d()[0]
    y_span = ax.get_ylim3d()[1] - ax.get_ylim3d()[0]
    z_span = ax.get_zlim3d()[1] - ax.get_zlim3d()[0]
    ax.set_box_aspect([x_span, y_span, z_span])


def plot_path_2d(ax, paths, rt: int, xini: float, yini: float, zini: float,
                 destin: np.ndarray, multi_target: bool) -> None:
    """Plot the IFDS waypoint path for outer iteration ``rt``."""
    n_lines = destin.shape[0]
    if multi_target:
        for li in range(n_lines):
            wp = paths[li][rt]
            if wp is None:
                continue
            ax.plot(wp[0], wp[1], wp[2], 'b', linewidth=1.8)
    else:
        wp = paths[0][rt]
        if wp is not None:
            ax.plot(wp[0], wp[1], wp[2], 'b--', linewidth=1.8)
            ax.scatter(wp[0, -1], wp[1, -1], wp[2, -1], marker='s', s=150, edgecolors='b', facecolors='none')
    ax.scatter([xini], [yini], [zini], c='r', marker='o', s=80)
    for d in destin:
        ax.scatter([d[0]], [d[1]], [d[2]], c='r', marker='x', s=150)
    ax.set_xlim(0, 200)
    ax.set_ylim(-100, 100)
    ax.set_zlim(0, 100)
    _set_equal_aspect_3d(ax)


def plot_objects_mpl(ax, objects: list["Object"], alpha_face: float = 0.4) -> None:
    """Render each obstacle's ``Gamma=1`` surface on a matplotlib 3D axis.

    Samples the implicit surface via parametric (u,v) for spheres/cylinders/cones.
    For a faithful MATLAB ``fimplicit3`` replacement, prefer
    :func:`plot_objects_pyvista`.
    """
    for obj in objects:
        u = np.linspace(0, 2 * np.pi, 40)
        v = np.linspace(-np.pi / 2, np.pi / 2, 20)
        U, V = np.meshgrid(u, v)
        x = obj.origin[0] + obj.a * np.cos(V) * np.cos(U)
        y = obj.origin[1] + obj.b * np.cos(V) * np.sin(U)
        z = obj.origin[2] + obj.c * np.sin(V)
        z = np.clip(z, 0, None)
        ax.plot_surface(x, y, z, alpha=alpha_face, color='gray', linewidth=0)


def plot_objects_pyvista(objects: list["Object"], rg: float = 0.0,
                         plotter=None, show: bool = True):
    """Render obstacles as implicit ``Gamma=1`` isosurfaces via PyVista.

    Parameters
    ----------
    objects:
        Obstacles to draw.
    rg:
        If >0, also render the safeguard ``Gamma*=1`` surface with transparency.
    plotter:
        Existing ``pyvista.Plotter`` to draw into. A new one is created if ``None``.
    show:
        Whether to call ``plotter.show()`` before returning.
    """
    import pyvista as pv

    if plotter is None:
        plotter = pv.Plotter()

    grid = pv.ImageData(
        dimensions=(101, 101, 51),
        spacing=(2.0, 2.0, 2.0),
        origin=(0.0, -100.0, 0.0),
    )
    pts = grid.points
    X, Y, Z = pts[:, 0], pts[:, 1], pts[:, 2]

    for obj in objects:
        x0, y0, z0 = obj.origin
        gamma = (
            ((X - x0) / obj.a) ** (2 * obj.p)
            + ((Y - y0) / obj.b) ** (2 * obj.q)
            + ((Z - z0) / obj.c) ** (2 * obj.r)
        )
        grid["Gamma"] = gamma
        iso = grid.contour(isosurfaces=[1.0], scalars="Gamma")
        if iso.n_points > 0:
            plotter.add_mesh(iso, color="lightgray", opacity=1.0)
        if rg > 0 and obj.rstar > 0:
            gstar = gamma - ((obj.rstar + rg) / obj.rstar) ** 2 + 1.0
            grid["GammaStar"] = gstar
            iso_s = grid.contour(isosurfaces=[1.0], scalars="GammaStar")
            if iso_s.n_points > 0:
                plotter.add_mesh(iso_s, color="lightgray", opacity=0.2)

    plotter.add_axes()
    plotter.show_grid()
    if show:
        plotter.show()
    return plotter


def plot_weather_contour(ax, weather_mat: np.ndarray, b_u: float,
                         frame: int = 0, n_levels: int = 30) -> None:
    """Filled contour plot of a weather slice with a ``B_U`` iso-line (2D axes)."""
    x = np.arange(1, weather_mat.shape[0] + 1)
    y = np.arange(-100, weather_mat.shape[1] - 100)
    X, Y = np.meshgrid(x, y, indexing="ij")
    ax.contourf(X, Y, weather_mat[:, :, frame], n_levels, cmap="turbo")
    ax.contour(X, Y, weather_mat[:, :, frame], levels=[b_u], colors="white", linewidths=2)


def plot_weather_ground(ax, weather, *, frame: int = 0,
                        n_levels: int = 30, alpha: float = 0.6) -> None:
    """Render a weather contour on the z=0 ground plane of a 3D axis.

    Parameters
    ----------
    weather:
        A :class:`~ifds.weather.WeatherField` instance.
    frame:
        Time-slice index into ``weather.mat``.
    """
    mat = weather.mat
    x = np.arange(1, mat.shape[0] + 1)
    y = np.arange(-100, mat.shape[1] - 100)
    X, Y = np.meshgrid(x, y, indexing="ij")
    ax.contourf(X, Y, mat[:, :, frame], n_levels, zdir='z', offset=0,
                cmap="turbo", alpha=alpha)
    ax.contour(X, Y, mat[:, :, frame], levels=[weather.b_u], zdir='z',
               offset=0, colors="white", linewidths=2)


def plot_scene_mpl(ax, traj, rt: int, paths, objects: list["Object"],
                   xini: float, yini: float, zini: float,
                   destin: np.ndarray, multi_target: bool) -> None:
    """Compose path + obstacles + trajectory trail — mirrors ``plotting_everything.m``."""
    cur = traj[rt]
    if cur is not None and cur.shape[1] > 0:
        ax.scatter([cur[0, 0]], [cur[1, 0]], [cur[2, 0]], c='k', marker='o', s=60)
    if paths[0][rt] is not None:
        plot_path_2d(ax, paths, rt, xini, yini, zini, destin, multi_target)
    if rt > 0:
        prev = np.concatenate([t for t in traj[:rt] if t is not None], axis=1)
        if prev.size:
            ax.plot(prev[0], prev[1], prev[2], 'k', linewidth=1.2)
    plot_objects_mpl(ax, objects)
    ax.set_xlim(0, 200)
    ax.set_ylim(-100, 100)
    ax.set_zlim(0, 100)
    _set_equal_aspect_3d(ax)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")


def quick_show_path(paths, rt: int, objects: list["Object"], destin: np.ndarray,
                    title: str = "IFDS path") -> None:
    """Convenience plot: single figure with path + obstacles."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    plot_path_2d(ax, paths, rt, 0, 0, 0, destin, multi_target=False)
    plot_objects_mpl(ax, objects)
    ax.set_title(title)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    plt.tight_layout()
    plt.show()
