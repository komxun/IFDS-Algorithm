"""Real-time-step animation & video export.

Replaces the ``figure(69)`` animation loop and the ``VideoWriter`` block in
``main.m``.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import matplotlib.animation as manim
import matplotlib.pyplot as plt
import numpy as np

from viz.plotting import _set_equal_aspect_3d, plot_objects_mpl, plot_path_2d, plot_weather_ground

if TYPE_CHECKING:
    from ifds.objects import Object


def animate_trajectory(
    traj: list[np.ndarray | None],
    paths,
    objects_per_rt: list[list["Object"]] | list["Object"],
    destin: np.ndarray,
    *,
    weather=None,
    multi_target: bool = False,
    xini: float = 0.0,
    yini: float = 0.0,
    zini: float = 0.0,
    save_path: str | Path | None = None,
    fps: int = 2,
) -> manim.FuncAnimation:
    """Animate per-``rt`` frames; optionally save to ``.mp4``."""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")

    def _frame(rt: int):
        ax.clear()
        if paths[0][rt] is not None:
            plot_path_2d(ax, paths, rt, xini, yini, zini, destin, multi_target)
        objs = objects_per_rt[rt] if isinstance(objects_per_rt, list) and objects_per_rt \
            and isinstance(objects_per_rt[0], list) else objects_per_rt
        plot_objects_mpl(ax, objs)  # type: ignore[arg-type]
        if weather is not None:
            frame = min(rt, weather.n_frames - 1)
            plot_weather_ground(ax, weather, frame=frame)
        if rt > 0:
            prev = [t for t in traj[:rt] if t is not None and t.size]
            if prev:
                stacked = np.concatenate(prev, axis=1)
                ax.plot(stacked[0], stacked[1], stacked[2], 'k', linewidth=1.2)
        ax.set_xlim(0, 200)
        ax.set_ylim(-100, 100)
        ax.set_zlim(0, 100)
        _set_equal_aspect_3d(ax)
        ax.set_title(f"t = {rt}")

    anim = manim.FuncAnimation(fig, _frame, frames=len(traj), interval=1000 // max(fps, 1))
    if save_path is not None:
        anim.save(str(save_path), fps=fps, writer="ffmpeg")
    return anim
